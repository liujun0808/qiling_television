#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
std::filesystem::path resolveModelPath(const std::string & path_text)
{
  const std::string package_prefix = "package://";
  if (path_text.rfind(package_prefix, 0) == 0) {
    const std::string relative = path_text.substr(package_prefix.size());
    const auto split_pos = relative.find('/');
    if (split_pos == std::string::npos || split_pos == 0 || split_pos + 1 >= relative.size()) {
      throw std::runtime_error("非法package路径: " + path_text);
    }

    const std::string package_name = relative.substr(0, split_pos);
    const std::string package_path = relative.substr(split_pos + 1);
    return std::filesystem::path(
      ament_index_cpp::get_package_share_directory(package_name)) / package_path;
  }

  std::filesystem::path path(path_text);
  if (path.is_relative()) {
    path = std::filesystem::current_path() / path;
  }
  return std::filesystem::absolute(path);
}

void throwIfMujocoError(const mjModel * model, const std::string & message)
{
  if (model == nullptr) {
    throw std::runtime_error(message);
  }
}
}  // namespace

class D435CameraPublisher : public rclcpp::Node
{
public:
  D435CameraPublisher()
  : Node("d435_camera_publisher")
  {
    model_path_text_ = declare_parameter<std::string>(
      "model_path",
      "package://qi_robot_description/new_scene/scene_S4_40DOF_fullbody.xml");
    camera_name_ = declare_parameter<std::string>("camera_name", "d435_camera");
    frame_id_ = declare_parameter<std::string>("frame_id", "d435_camera");
    image_topic_ = declare_parameter<std::string>("image_topic", "/d435/color/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/d435/color/camera_info");
    enable_depth_ = declare_parameter<bool>("enable_depth", true);
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/d435/aligned_depth_to_color/image_raw");
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 408);
    fps_ = declare_parameter<double>("fps", 30.0);
    reset_to_home_ = declare_parameter<bool>("reset_to_home", true);
    reset_keyframe_ = declare_parameter<std::string>("reset_keyframe", "teleop_home");
    qpos_topic_ = declare_parameter<std::string>("qpos_topic", "/mujoco/qpos");

    if (width_ <= 0 || height_ <= 0) {
      throw std::runtime_error("width和height必须大于0");
    }
    if (fps_ <= 0.0) {
      throw std::runtime_error("fps必须大于0");
    }

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);
    if (enable_depth_) {
      depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, 10);
    }

    initGlfw();
    loadModel();
    initRenderer();

    qpos_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      qpos_topic_, rclcpp::QoS(1),
      std::bind(&D435CameraPublisher::qposCallback, this, std::placeholders::_1));

    camera_info_ = buildCameraInfo();

    const auto period = std::chrono::duration<double>(1.0 / fps_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&D435CameraPublisher::publishFrame, this));

    RCLCPP_INFO(
      get_logger(),
      "D435相机发布节点已启动: model=%s, camera=%s, %dx%d@%.1fHz, image=%s, info=%s, qpos=%s",
      model_path_.string().c_str(), camera_name_.c_str(), width_, height_, fps_,
      image_topic_.c_str(), camera_info_topic_.c_str(), qpos_topic_.c_str());
    if (enable_depth_) {
      RCLCPP_INFO(get_logger(), "对齐深度图发布已开启: depth=%s", depth_topic_.c_str());
    }
  }

  ~D435CameraPublisher() override
  {
    mjr_freeContext(&context_);
    mjv_freeScene(&scene_);
    if (data_ != nullptr) {
      mj_deleteData(data_);
      data_ = nullptr;
    }
    if (model_ != nullptr) {
      mj_deleteModel(model_);
      model_ = nullptr;
    }
    if (window_ != nullptr) {
      glfwDestroyWindow(window_);
      window_ = nullptr;
    }
    if (glfw_initialized_) {
      glfwTerminate();
    }
  }

private:
  void initGlfw()
  {
    if (!glfwInit()) {
      throw std::runtime_error("GLFW初始化失败，请检查图形环境或DISPLAY设置");
    }
    glfw_initialized_ = true;

    // 隐藏窗口仅用于创建OpenGL上下文；ROS侧只发布图像话题。
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(width_, height_, "mujoco_d435_offscreen", nullptr, nullptr);
    if (window_ == nullptr) {
      throw std::runtime_error("GLFW隐藏窗口创建失败");
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);
  }

  void loadModel()
  {
    model_path_ = resolveModelPath(model_path_text_);
    char error[1024] = "";
    model_ = mj_loadXML(model_path_.string().c_str(), nullptr, error, sizeof(error));
    throwIfMujocoError(model_, "MuJoCo模型加载失败: " + std::string(error));

    data_ = mj_makeData(model_);
    if (data_ == nullptr) {
      throw std::runtime_error("MuJoCo数据创建失败");
    }

    camera_id_ = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
    if (camera_id_ < 0) {
      throw std::runtime_error("MuJoCo模型中未找到相机: " + camera_name_);
    }

    if (reset_to_home_) {
      const int key_id = mj_name2id(model_, mjOBJ_KEY, reset_keyframe_.c_str());
      if (key_id >= 0) {
        mj_resetDataKeyframe(model_, data_, key_id);
      } else {
        RCLCPP_WARN(
          get_logger(), "未找到MuJoCo keyframe '%s'，使用默认状态",
          reset_keyframe_.c_str());
        mj_resetData(model_, data_);
      }
    } else {
      mj_resetData(model_, data_);
    }
    mj_forward(model_, data_);
  }

  void initRenderer()
  {
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultPerturb(&perturb_);

    camera_.type = mjCAMERA_FIXED;
    camera_.fixedcamid = camera_id_;

    mjv_makeScene(model_, &scene_, 10000);
    mjr_defaultContext(&context_);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_OFFSCREEN, &context_);

    rgb_buffer_.resize(static_cast<std::size_t>(width_) * height_ * 3);
    flipped_buffer_.resize(rgb_buffer_.size());
    if (enable_depth_) {
      depth_buffer_.resize(static_cast<std::size_t>(width_) * height_);
      depth_metric_buffer_.resize(depth_buffer_.size());
      depth_msg_buffer_.resize(depth_buffer_.size() * sizeof(float));
    }
  }

  sensor_msgs::msg::CameraInfo buildCameraInfo() const
  {
    sensor_msgs::msg::CameraInfo msg;
    msg.header.frame_id = frame_id_;
    msg.width = static_cast<uint32_t>(width_);
    msg.height = static_cast<uint32_t>(height_);
    msg.distortion_model = "plumb_bob";
    msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // MuJoCo固定相机使用垂直视场角fovy，按针孔模型估算内参。
    const double fovy_rad = model_->cam_fovy[camera_id_] * M_PI / 180.0;
    const double fy = static_cast<double>(height_) / (2.0 * std::tan(fovy_rad / 2.0));
    const double fx = fy;
    const double cx = (static_cast<double>(width_) - 1.0) / 2.0;
    const double cy = (static_cast<double>(height_) - 1.0) / 2.0;

    msg.k = {
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0};
    msg.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
    msg.p = {
      fx, 0.0, cx, 0.0,
      0.0, fy, cy, 0.0,
      0.0, 0.0, 1.0, 0.0};
    return msg;
  }

  void qposCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != static_cast<std::size_t>(model_->nq)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "收到的qpos长度=%zu，与当前模型nq=%d不匹配，忽略该帧",
        msg->data.size(), model_->nq);
      return;
    }

    std::copy_n(msg->data.begin(), model_->nq, data_->qpos);
    // 渲染仅依赖由 qpos 推导出的运动学量；无需复制 qvel/act。
    mj_forward(model_, data_);
    if (!has_received_qpos_) {
      has_received_qpos_ = true;
      RCLCPP_INFO(get_logger(), "已接收MuJoCo实时qpos，相机图像开始与主仿真同步");
    }
  }

  void publishFrame()
  {
    mj_forward(model_, data_);

    const mjrRect viewport{0, 0, width_, height_};
    mjv_updateScene(model_, data_, &option_, &perturb_, &camera_, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &context_);
    mjr_readPixels(
      rgb_buffer_.data(),
      enable_depth_ ? depth_buffer_.data() : nullptr,
      viewport,
      &context_);

    // OpenGL读取结果原点在左下角，ROS图像约定按左上角逐行存储。
    const std::size_t row_bytes = static_cast<std::size_t>(width_) * 3;
    for (int row = 0; row < height_; ++row) {
      const auto src_offset = static_cast<std::size_t>(height_ - 1 - row) * row_bytes;
      const auto dst_offset = static_cast<std::size_t>(row) * row_bytes;
      std::copy_n(
        rgb_buffer_.begin() + static_cast<std::ptrdiff_t>(src_offset),
        row_bytes,
        flipped_buffer_.begin() + static_cast<std::ptrdiff_t>(dst_offset));
    }

    const auto stamp = now();

    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = stamp;
    image_msg.header.frame_id = frame_id_;
    image_msg.height = static_cast<uint32_t>(height_);
    image_msg.width = static_cast<uint32_t>(width_);
    image_msg.encoding = "rgb8";
    image_msg.is_bigendian = false;
    image_msg.step = static_cast<uint32_t>(width_ * 3);
    image_msg.data = flipped_buffer_;

    camera_info_.header.stamp = stamp;

    image_pub_->publish(image_msg);
    info_pub_->publish(camera_info_);

    if (enable_depth_) {
      publishDepthFrame(stamp);
    }
  }

  void publishDepthFrame(const rclcpp::Time & stamp)
  {
    const std::size_t row_pixels = static_cast<std::size_t>(width_);
    const std::size_t row_bytes = row_pixels * sizeof(float);
    const double extent = model_->stat.extent;
    const double near = model_->vis.map.znear * extent;
    const double far = model_->vis.map.zfar * extent;

    // MuJoCo/OpenGL深度缓冲是非线性的[0,1]值，这里转换为相机光轴方向的米制深度。
    for (int row = 0; row < height_; ++row) {
      const auto src_row = static_cast<std::size_t>(height_ - 1 - row);
      const auto dst_row = static_cast<std::size_t>(row);
      for (int col = 0; col < width_; ++col) {
        const auto src_index = src_row * row_pixels + static_cast<std::size_t>(col);
        const auto dst_index = dst_row * row_pixels + static_cast<std::size_t>(col);
        const float z_buffer = depth_buffer_[src_index];
        if (z_buffer >= 1.0f) {
          depth_metric_buffer_[dst_index] = 0.0f;
          continue;
        }
        const double z_ndc = 2.0 * static_cast<double>(z_buffer) - 1.0;
        const double depth_m = (2.0 * near * far) / (far + near - z_ndc * (far - near));
        depth_metric_buffer_[dst_index] = static_cast<float>(depth_m);
      }
    }

    for (int row = 0; row < height_; ++row) {
      const auto offset = static_cast<std::size_t>(row) * row_bytes;
      const auto * row_ptr = reinterpret_cast<const uint8_t *>(
        depth_metric_buffer_.data() + static_cast<std::size_t>(row) * row_pixels);
      std::copy_n(row_ptr, row_bytes, depth_msg_buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
    }

    sensor_msgs::msg::Image depth_msg;
    depth_msg.header.stamp = stamp;
    depth_msg.header.frame_id = frame_id_;
    depth_msg.height = static_cast<uint32_t>(height_);
    depth_msg.width = static_cast<uint32_t>(width_);
    depth_msg.encoding = "32FC1";
    depth_msg.is_bigendian = false;
    depth_msg.step = static_cast<uint32_t>(width_ * sizeof(float));
    depth_msg.data = depth_msg_buffer_;

    depth_pub_->publish(depth_msg);
  }

  std::string model_path_text_;
  std::string camera_name_;
  std::string frame_id_;
  std::string image_topic_;
  std::string camera_info_topic_;
  bool enable_depth_{true};
  std::string depth_topic_;
  int width_{640};
  int height_{408};
  double fps_{30.0};
  bool reset_to_home_{true};
  std::string reset_keyframe_;
  std::string qpos_topic_;
  bool has_received_qpos_{false};

  std::filesystem::path model_path_;
  bool glfw_initialized_{false};
  GLFWwindow * window_{nullptr};
  mjModel * model_{nullptr};
  mjData * data_{nullptr};
  int camera_id_{-1};
  mjvCamera camera_;
  mjvOption option_;
  mjvPerturb perturb_;
  mjvScene scene_;
  mjrContext context_;
  std::vector<unsigned char> rgb_buffer_;
  std::vector<unsigned char> flipped_buffer_;
  std::vector<float> depth_buffer_;
  std::vector<float> depth_metric_buffer_;
  std::vector<uint8_t> depth_msg_buffer_;
  sensor_msgs::msg::CameraInfo camera_info_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr qpos_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<D435CameraPublisher>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("d435_camera_publisher"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
