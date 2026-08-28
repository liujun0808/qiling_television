#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
constexpr int kArmDof = 14;
using PoseMsg = geometry_msgs::msg::PoseStamped;

}  // namespace

class PoseTargetDemo final : public rclcpp::Node
{
public:
  PoseTargetDemo()
  : Node("qiling_pose_target_demo")
  {
    declare_parameter("publish_rate_hz", 50.0);
    declare_parameter("joint_state_topic", std::string("/joint_states"));
    declare_parameter("left_target_topic", std::string("/teleop/left_wrist_target"));
    declare_parameter("right_target_topic", std::string("/teleop/right_wrist_target"));
    declare_parameter("target_frame", std::string("base_link"));
    declare_parameter("left_ee_frame", std::string("LH_hand_base_link"));
    declare_parameter("right_ee_frame", std::string("RH_hand_base_link"));
    declare_parameter("translation_amplitude", 0.03);
    declare_parameter("vertical_amplitude", 0.02);
    declare_parameter("period_sec", 6.0);

    const std::string description_share =
      ament_index_cpp::get_package_share_directory("qi_robot_description");
    const auto urdf_path = std::filesystem::path(description_share) / "urdf" / "s4_dual_arm.urdf";
    pinocchio::urdf::buildModel(urdf_path.string(), model_);
    data_ = pinocchio::Data(model_);
    if (model_.nq != kArmDof || model_.nv != kArmDof) {
      throw std::runtime_error("pose target demo expected a 14 DoF arm model");
    }

    const std::array<std::string, kArmDof> names = {
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
      "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
      "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
      "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
    for (int i = 0; i < kArmDof; ++i) {
      const pinocchio::JointIndex joint_id = model_.getJointId(names[i]);
      if (joint_id == 0 || joint_id >= static_cast<pinocchio::JointIndex>(model_.njoints)) {
        throw std::runtime_error("missing joint in pose target demo: " + names[i]);
      }
      q_indices_[i] = model_.idx_qs[joint_id];
      joint_name_to_arm_index_[names[i]] = i;
    }

    left_frame_id_ = resolveFrame(get_parameter("left_ee_frame").as_string());
    right_frame_id_ = resolveFrame(get_parameter("right_ee_frame").as_string());
    q_.setZero(model_.nq);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&PoseTargetDemo::jointStateCallback, this, std::placeholders::_1));
    left_target_pub_ = create_publisher<PoseMsg>(
      get_parameter("left_target_topic").as_string(), rclcpp::QoS(1));
    right_target_pub_ = create_publisher<PoseMsg>(
      get_parameter("right_target_topic").as_string(), rclcpp::QoS(1));

    const double rate = get_parameter("publish_rate_hz").as_double();
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(rate, 1.0)));
    timer_ = create_wall_timer(period, std::bind(&PoseTargetDemo::publishTargets, this));
    RCLCPP_INFO(get_logger(), "Pose target demo ready at %.1f Hz", rate);
  }

private:
  pinocchio::FrameIndex resolveFrame(const std::string & name)
  {
    const auto id = model_.getFrameId(name);
    if (id >= static_cast<pinocchio::FrameIndex>(model_.nframes)) {
      throw std::runtime_error("missing end-effector frame: " + name);
    }
    return id;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.size() != msg->position.size()) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      const auto it = joint_name_to_arm_index_.find(msg->name[i]);
      if (it != joint_name_to_arm_index_.end() && std::isfinite(msg->position[i])) {
        q_[q_indices_[it->second]] = msg->position[i];
        state_received_ = true;
      }
    }
  }

  void publishTargets()
  {
    Eigen::VectorXd q_snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!state_received_) {
        return;
      }
      q_snapshot = q_;
    }

    pinocchio::forwardKinematics(model_, data_, q_snapshot);
    pinocchio::updateFramePlacements(model_, data_);
    if (!anchor_initialized_) {
      left_anchor_ = data_.oMf[left_frame_id_];
      right_anchor_ = data_.oMf[right_frame_id_];
      anchor_start_ = now();
      anchor_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Captured current dual-arm EE pose as demo anchor");
    }

    const double elapsed = (now() - anchor_start_).seconds();
    const double period = std::max(get_parameter("period_sec").as_double(), 0.1);
    const double phase = 2.0 * M_PI * elapsed / period;
    const double dx = get_parameter("translation_amplitude").as_double() * std::sin(phase);
    const double dz = get_parameter("vertical_amplitude").as_double() * (1.0 - std::cos(phase));

    pinocchio::SE3 left_target = left_anchor_;
    pinocchio::SE3 right_target = right_anchor_;
    left_target.translation().x() += dx;
    left_target.translation().z() += dz;
    right_target.translation().x() += dx;
    right_target.translation().z() += dz;

    const auto stamp = now();
    left_target_pub_->publish(toPoseMessage(left_target, stamp));
    right_target_pub_->publish(toPoseMessage(right_target, stamp));
  }

  PoseMsg toPoseMessage(const pinocchio::SE3 & pose, const rclcpp::Time & stamp) const
  {
    PoseMsg msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = get_parameter("target_frame").as_string();
    msg.pose.position.x = pose.translation().x();
    msg.pose.position.y = pose.translation().y();
    msg.pose.position.z = pose.translation().z();
    const Eigen::Quaterniond quat(pose.rotation());
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    return msg;
  }

  pinocchio::Model model_;
  pinocchio::Data data_{model_};
  Eigen::VectorXd q_;
  std::array<int, kArmDof> q_indices_{};
  std::unordered_map<std::string, int> joint_name_to_arm_index_;
  pinocchio::FrameIndex left_frame_id_{0};
  pinocchio::FrameIndex right_frame_id_{0};
  pinocchio::SE3 left_anchor_;
  pinocchio::SE3 right_anchor_;
  rclcpp::Time anchor_start_;
  bool state_received_{false};
  bool anchor_initialized_{false};
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<PoseMsg>::SharedPtr left_target_pub_;
  rclcpp::Publisher<PoseMsg>::SharedPtr right_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PoseTargetDemo>());
  } catch (const std::exception & error) {
    fprintf(stderr, "qiling_pose_target_demo fatal: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
