#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <PXREARobotSDK.h>

namespace
{
using json = nlohmann::json;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using JoyMsg = sensor_msgs::msg::Joy;
using Clock = std::chrono::steady_clock;

struct ControllerState
{
  std::array<double, 7> pose{};  // x,y,z,qx,qy,qz,qw
  double trigger{0.0};
  double grip{0.0};
  bool primary_button{false};
  bool pose_valid{false};
};

struct FrameState
{
  ControllerState left;
  ControllerState right;
  Clock::time_point received_at{};
  bool valid{false};
};

bool finite(double value)
{
  return std::isfinite(value);
}

bool parseBool(const json & value, bool & result)
{
  try {
    if (value.is_boolean()) {
      result = value.get<bool>();
      return true;
    }
    if (value.is_number()) {
      result = value.get<double>() != 0.0;
      return true;
    }
    if (value.is_string()) {
      const std::string text = value.get<std::string>();
      if (text == "true" || text == "1") {
        result = true;
        return true;
      }
      if (text == "false" || text == "0") {
        result = false;
        return true;
      }
    }
  } catch (const std::exception &) {
    return false;
  }
  return false;
}

bool parsePose(const json & value, std::array<double, 7> & pose)
{
  try {
    if (value.is_string()) {
      std::string text = value.get<std::string>();
      std::size_t begin = 0;
      for (std::size_t index = 0; index < pose.size(); ++index) {
        const std::size_t end = text.find(',', begin);
        const std::string token = text.substr(begin, end == std::string::npos ? end : end - begin);
        pose[index] = std::stod(token);
        if (end == std::string::npos) {
          if (index + 1 != pose.size()) {
            return false;
          }
          break;
        }
        begin = end + 1;
      }
    } else if (value.is_array() && value.size() >= pose.size()) {
      for (std::size_t index = 0; index < pose.size(); ++index) {
        pose[index] = value[index].get<double>();
      }
    } else {
      return false;
    }
  } catch (const std::exception &) {
    return false;
  }

  for (const double component : pose) {
    if (!finite(component)) {
      return false;
    }
  }
  const double quaternion_norm = std::sqrt(
    pose[3] * pose[3] + pose[4] * pose[4] + pose[5] * pose[5] + pose[6] * pose[6]);
  return quaternion_norm > 1e-8;
}

bool parseController(const json & value, ControllerState & controller)
{
  if (!value.is_object() || !value.contains("pose")) {
    return false;
  }
  if (!parsePose(value.at("pose"), controller.pose)) {
    return false;
  }
  try {
    if (value.contains("trigger")) {
      controller.trigger = std::clamp(value.at("trigger").get<double>(), 0.0, 1.0);
    }
    if (value.contains("grip")) {
      controller.grip = std::clamp(value.at("grip").get<double>(), 0.0, 1.0);
    }
    if (value.contains("primaryButton")) {
      parseBool(value.at("primaryButton"), controller.primary_button);
    }
  } catch (const std::exception &) {
    return false;
  }
  controller.pose_valid = true;
  return true;
}

PoseMsg makePoseMessage(
  const ControllerState & controller, const std::string & frame, rclcpp::Time stamp)
{
  PoseMsg message;
  message.header.stamp = stamp;
  message.header.frame_id = frame;
  message.pose.position.x = controller.pose[0];
  message.pose.position.y = controller.pose[1];
  message.pose.position.z = controller.pose[2];
  message.pose.orientation.x = controller.pose[3];
  message.pose.orientation.y = controller.pose[4];
  message.pose.orientation.z = controller.pose[5];
  message.pose.orientation.w = controller.pose[6];
  return message;
}
}  // namespace

class XroboToolkitPxreaAdapter final : public rclcpp::Node
{
public:
  XroboToolkitPxreaAdapter()
  : Node("qiling_xrobotoolkit_pxrea_adapter")
  {
    declare_parameter("publish_rate_hz", 90.0);
    declare_parameter("left_pose_topic", std::string("/xr/left_controller_pose"));
    declare_parameter("right_pose_topic", std::string("/xr/right_controller_pose"));
    declare_parameter("joy_topic", std::string("/xr/controller_joy"));
    declare_parameter("input_frame", std::string("xr_origin"));
    declare_parameter("grip_button_threshold", 0.90);
    declare_parameter("frame_timeout_sec", 0.20);

    left_pose_pub_ = create_publisher<PoseMsg>(
      get_parameter("left_pose_topic").as_string(), rclcpp::QoS(1));
    right_pose_pub_ = create_publisher<PoseMsg>(
      get_parameter("right_pose_topic").as_string(), rclcpp::QoS(1));
    joy_pub_ = create_publisher<JoyMsg>(
      get_parameter("joy_topic").as_string(), rclcpp::QoS(1));

    const int result = PXREAInit(this, &XroboToolkitPxreaAdapter::sdkCallback, PXREAFullMask);
    if (result != 0) {
      throw std::runtime_error(
              "PXREAInit failed; start /opt/apps/roboticsservice/runService.sh and connect Quest first");
    }
    sdk_initialized_ = true;

    const double rate = std::max(get_parameter("publish_rate_hz").as_double(), 1.0);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / rate));
    publish_timer_ = create_wall_timer(period, std::bind(&XroboToolkitPxreaAdapter::publishTick, this));

    RCLCPP_INFO(
      get_logger(), "XRoboToolkit PXREA adapter ready at %.1f Hz; waiting for Quest controller frames",
      rate);
  }

  ~XroboToolkitPxreaAdapter() override
  {
    if (sdk_initialized_) {
      PXREADeinit();
    }
  }

private:
  static void sdkCallback(void * context, PXREAClientCallbackType type, int status, void * user_data)
  {
    auto * adapter = static_cast<XroboToolkitPxreaAdapter *>(context);
    if (adapter != nullptr) {
      adapter->handleSdkCallback(type, status, user_data);
    }
  }

  void handleSdkCallback(PXREAClientCallbackType type, int status, void * user_data)
  {
    if (type == PXREAServerConnect) {
      RCLCPP_INFO(get_logger(), "XRoboToolkit PC Service connected");
      return;
    }
    if (type == PXREAServerDisconnect) {
      RCLCPP_WARN(get_logger(), "XRoboToolkit PC Service disconnected");
      std::lock_guard<std::mutex> lock(mutex_);
      frame_.valid = false;
      return;
    }
    if (type != PXREADeviceStateJson || user_data == nullptr) {
      (void)status;
      return;
    }

    const auto * state = static_cast<const PXREADevStateJson *>(user_data);
    try {
      const json envelope = json::parse(state->stateJson);
      if (!envelope.contains("value")) {
        return;
      }
      const json value = envelope.at("value").is_string() ?
        json::parse(envelope.at("value").get<std::string>()) : envelope.at("value");
      if (!value.contains("Controller") || !value.at("Controller").is_object()) {
        return;
      }

      FrameState next;
      const auto & controllers = value.at("Controller");
      const bool left_valid = controllers.contains("left") &&
        parseController(controllers.at("left"), next.left);
      const bool right_valid = controllers.contains("right") &&
        parseController(controllers.at("right"), next.right);
      if (!left_valid && !right_valid) {
        return;
      }
      next.received_at = Clock::now();
      next.valid = true;

      std::lock_guard<std::mutex> lock(mutex_);
      if (left_valid) {
        frame_.left = next.left;
      }
      if (right_valid) {
        frame_.right = next.right;
      }
      frame_.received_at = next.received_at;
      frame_.valid = true;
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Invalid PXREA device state JSON: %s", error.what());
    }
  }

  void publishTick()
  {
    FrameState frame;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frame = frame_;
    }
    if (!frame.valid ||
      std::chrono::duration<double>(Clock::now() - frame.received_at).count() >
      get_parameter("frame_timeout_sec").as_double()) {
      return;
    }

    const rclcpp::Time stamp = now();
    const std::string frame_id = get_parameter("input_frame").as_string();
    if (frame.left.pose_valid) {
      left_pose_pub_->publish(makePoseMessage(frame.left, frame_id, stamp));
    }
    if (frame.right.pose_valid) {
      right_pose_pub_->publish(makePoseMessage(frame.right, frame_id, stamp));
    }

    JoyMsg joy;
    joy.header.stamp = stamp;
    joy.buttons.assign(6, 0);
    // axes[0:2] are triggers, axes[2:4] are analog grips for the later O6 adapter.
    joy.axes = {
      static_cast<float>(frame.left.trigger), static_cast<float>(frame.right.trigger),
      static_cast<float>(frame.left.grip), static_cast<float>(frame.right.grip)};
    const double threshold = get_parameter("grip_button_threshold").as_double();
    // Quest primary buttons: left X -> buttons[0], right A -> buttons[1].
    joy.buttons[0] = frame.left.primary_button ? 1 : 0;
    joy.buttons[1] = frame.right.primary_button ? 1 : 0;
    joy.buttons[4] = frame.left.grip >= threshold ? 1 : 0;
    joy.buttons[5] = frame.right.grip >= threshold ? 1 : 0;
    joy_pub_->publish(joy);
  }

  std::mutex mutex_;
  FrameState frame_;
  bool sdk_initialized_{false};

  rclcpp::Publisher<PoseMsg>::SharedPtr left_pose_pub_;
  rclcpp::Publisher<PoseMsg>::SharedPtr right_pose_pub_;
  rclcpp::Publisher<JoyMsg>::SharedPtr joy_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<XroboToolkitPxreaAdapter>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "qiling_xrobotoolkit_pxrea_adapter fatal: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
