#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace
{
using PoseMsg = geometry_msgs::msg::PoseStamped;
using JoyMsg = sensor_msgs::msg::Joy;
using ModeMsg = std_msgs::msg::UInt8;
using Clock = std::chrono::steady_clock;

struct TimedPose
{
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Clock::time_point received_at{};
  std::string frame;
  bool valid{false};
};

struct TimedJoy
{
  JoyMsg message;
  Clock::time_point received_at{};
  bool valid{false};
};

struct SideState
{
  std::string name;
  TimedPose xr_pose;
  TimedPose robot_pose;
  Eigen::Isometry3d xr_anchor{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d robot_anchor{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d target{Eigen::Isometry3d::Identity()};
  bool active{false};
  bool target_valid{false};
};

bool finitePose(const PoseMsg & message, Eigen::Isometry3d & result)
{
  const auto & p = message.pose.position;
  const auto & o = message.pose.orientation;
  Eigen::Quaterniond quaternion(o.w, o.x, o.y, o.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-8) {
    return false;
  }
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
    return false;
  }
  quaternion.normalize();
  result = Eigen::Isometry3d::Identity();
  result.linear() = quaternion.toRotationMatrix();
  result.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  return result.matrix().allFinite();
}

PoseMsg toPoseMessage(const Eigen::Isometry3d & pose, const std::string & frame, rclcpp::Time stamp)
{
  PoseMsg message;
  message.header.stamp = stamp;
  message.header.frame_id = frame;
  const Eigen::Quaterniond quaternion(pose.rotation());
  message.pose.position.x = pose.translation().x();
  message.pose.position.y = pose.translation().y();
  message.pose.position.z = pose.translation().z();
  message.pose.orientation.x = quaternion.x();
  message.pose.orientation.y = quaternion.y();
  message.pose.orientation.z = quaternion.z();
  message.pose.orientation.w = quaternion.w();
  return message;
}

Eigen::Isometry3d boundedRelativeTransform(
  const Eigen::Isometry3d & xr_anchor,
  const Eigen::Isometry3d & xr_current,
  const Eigen::Isometry3d & robot_anchor,
  double translation_scale,
  const Eigen::Matrix3d & xr_to_robot_basis,
  const Eigen::Vector3d & robot_translation_axis_signs,
  double translation_deadband,
  bool invert_rotation,
  double rotation_scale,
  const Eigen::Vector3d & robot_rotation_axis_signs,
  double max_translation,
  double max_rotation)
{
  const Eigen::Isometry3d relative_xr = xr_anchor.inverse() * xr_current;
  Eigen::Isometry3d bounded = Eigen::Isometry3d::Identity();

  // XRoboToolkit reports a left-handed basis (X right, Y up, Z forward/in).
  // ROS robot frames are right-handed. The basis change therefore has
  // det=-1 and must not be represented by a quaternion. Translation uses the
  // basis change directly; rotation is converted by conjugation, which keeps
  // the result a valid proper rotation: R_robot = M * R_xr * M^-1.
  // Convert the XR basis to robot base_link first, then express the result
  // in the robot anchor frame because it is finally composed as
  // robot_anchor * relative_robot.
  const Eigen::Matrix3d anchor_basis_change =
    robot_anchor.rotation().transpose() * xr_to_robot_basis;
  Eigen::Vector3d translation_base = xr_to_robot_basis * relative_xr.translation();
  translation_base = robot_translation_axis_signs.cwiseProduct(translation_base);
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(translation_base[axis]) < translation_deadband) {
      translation_base[axis] = 0.0;
    }
  }
  Eigen::Vector3d translation = translation_scale *
    robot_anchor.rotation().transpose() * translation_base;
  const double translation_norm = translation.norm();
  if (translation_norm > max_translation && translation_norm > 1e-9) {
    translation *= max_translation / translation_norm;
  }
  bounded.translation() = translation;

  Eigen::Matrix3d xr_relative_rotation = relative_xr.rotation();
  if (invert_rotation) {
    xr_relative_rotation.transposeInPlace();
  }
  const Eigen::Matrix3d mapped_rotation =
    anchor_basis_change * xr_relative_rotation * anchor_basis_change.transpose();
  Eigen::AngleAxisd rotation(mapped_rotation);
  if (!std::isfinite(rotation.angle())) {
    rotation = Eigen::AngleAxisd::Identity();
  }
  // Apply direction corrections in the fixed robot base_link frame.  The
  // current Quest stream has the positive base_link-X wrist rotation
  // reversed; correcting the rotation vector rather than the quaternion
  // keeps the other two axes independent.
  Eigen::Vector3d rotation_vector = rotation.axis() * rotation.angle();
  Eigen::Vector3d rotation_vector_base = robot_anchor.rotation() * rotation_vector;
  rotation_vector_base = robot_rotation_axis_signs.cwiseProduct(rotation_vector_base);
  rotation_vector = robot_anchor.rotation().transpose() * rotation_vector_base;
  const double scaled_angle = std::min(
    rotation_vector.norm() * std::max(rotation_scale, 0.0), max_rotation);
  if (rotation_vector.allFinite() && rotation_vector.norm() > 1e-9) {
    bounded.linear() = Eigen::AngleAxisd(
      scaled_angle, rotation_vector.normalized()).toRotationMatrix();
  }
  return bounded;
}
}  // namespace

class XrPoseClutchBridge final : public rclcpp::Node
{
public:
  XrPoseClutchBridge()
  : Node("qiling_xr_pose_clutch_bridge")
  {
    declareParameters();

    left_pose_sub_ = create_subscription<PoseMsg>(
      get_parameter("left_pose_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const PoseMsg::SharedPtr message) { poseCallback(*message, left_.xr_pose); });
    right_pose_sub_ = create_subscription<PoseMsg>(
      get_parameter("right_pose_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const PoseMsg::SharedPtr message) { poseCallback(*message, right_.xr_pose); });
    left_robot_sub_ = create_subscription<PoseMsg>(
      get_parameter("left_robot_state_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const PoseMsg::SharedPtr message) { poseCallback(*message, left_.robot_pose); });
    right_robot_sub_ = create_subscription<PoseMsg>(
      get_parameter("right_robot_state_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const PoseMsg::SharedPtr message) { poseCallback(*message, right_.robot_pose); });
    joy_sub_ = create_subscription<JoyMsg>(
      get_parameter("joy_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const JoyMsg::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        joy_.message = *message;
        joy_.received_at = Clock::now();
        joy_.valid = true;
      });

    left_target_pub_ = create_publisher<PoseMsg>(
      get_parameter("left_target_topic").as_string(), rclcpp::QoS(1));
    right_target_pub_ = create_publisher<PoseMsg>(
      get_parameter("right_target_topic").as_string(), rclcpp::QoS(1));
    left_mode_pub_ = create_publisher<ModeMsg>(
      get_parameter("left_mode_topic").as_string(), rclcpp::QoS(1));
    right_mode_pub_ = create_publisher<ModeMsg>(
      get_parameter("right_mode_topic").as_string(), rclcpp::QoS(1));

    left_.name = "left";
    right_.name = "right";
    const double rate = std::max(get_parameter("publish_rate_hz").as_double(), 1.0);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / rate));
    timer_ = create_wall_timer(period, std::bind(&XrPoseClutchBridge::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "XR pose bridge ready at %.1f Hz; Grip is left button/axis %d/%d and right button/axis %d/%d",
      rate,
      static_cast<int>(get_parameter("left_grip_button").as_int()),
      static_cast<int>(get_parameter("left_grip_axis").as_int()),
      static_cast<int>(get_parameter("right_grip_button").as_int()),
      static_cast<int>(get_parameter("right_grip_axis").as_int()));
  }

private:
  void declareParameters()
  {
    declare_parameter("publish_rate_hz", 50.0);
    declare_parameter("left_pose_topic", std::string("/xr/left_controller_pose"));
    declare_parameter("right_pose_topic", std::string("/xr/right_controller_pose"));
    declare_parameter("joy_topic", std::string("/xr/controller_joy"));
    declare_parameter("left_robot_state_topic", std::string("/teleop/left_wrist_state"));
    declare_parameter("right_robot_state_topic", std::string("/teleop/right_wrist_state"));
    declare_parameter("left_target_topic", std::string("/teleop/left_wrist_target"));
    declare_parameter("right_target_topic", std::string("/teleop/right_wrist_target"));
    declare_parameter("left_mode_topic", std::string("/teleop/left_control_mode"));
    declare_parameter("right_mode_topic", std::string("/teleop/right_control_mode"));
    declare_parameter("target_frame", std::string("base_link"));
    declare_parameter("left_grip_button", 4);
    declare_parameter("right_grip_button", 5);
    declare_parameter("left_grip_axis", -1);
    declare_parameter("right_grip_axis", -1);
    declare_parameter("grip_axis_threshold", 0.5);
    declare_parameter("log_clutch_state", true);
    declare_parameter("input_timeout_sec", 0.20);
    declare_parameter("joy_timeout_sec", 0.20);
    declare_parameter("robot_state_timeout_sec", 0.20);
    declare_parameter("translation_scale", 1.0);
    // XRoboToolkit left-handed basis (X right, Y up, Z forward/in) to the
    // ROS robot base basis (X forward, Y left, Z up). This matrix has det=-1
    // because the source and destination bases have different handedness.
    declare_parameter("xr_to_robot_m00", 0.0);
    declare_parameter("xr_to_robot_m01", 0.0);
    declare_parameter("xr_to_robot_m02", 1.0);
    declare_parameter("xr_to_robot_m10", -1.0);
    declare_parameter("xr_to_robot_m11", 0.0);
    declare_parameter("xr_to_robot_m12", 0.0);
    declare_parameter("xr_to_robot_m20", 0.0);
    declare_parameter("xr_to_robot_m21", 1.0);
    declare_parameter("xr_to_robot_m22", 0.0);
    // Empirical correction from the current Quest pose stream: Y/Z already
    // agree with the robot base, while robot-base X is reversed.
    declare_parameter("robot_translation_axis_sign_x", -1.0);
    declare_parameter("robot_translation_axis_sign_y", 1.0);
    declare_parameter("robot_translation_axis_sign_z", 1.0);
    // Current calibrated Quest stream uses the same base_link-Y sign for both
    // controllers. Keep per-side parameters so later hardware calibration can
    // still change one side without altering the common basis mapping.
    declare_parameter("left_robot_translation_axis_sign_y", 1.0);
    declare_parameter("right_robot_translation_axis_sign_y", 1.0);
    declare_parameter("translation_deadband_m", 0.015);
    declare_parameter("invert_relative_rotation", true);
    declare_parameter("rotation_scale", 0.50);
    // Current Quest stream: positive rotation about robot base_link X is
    // observed with the opposite sign. Y/Z retain the existing convention.
    declare_parameter("robot_rotation_axis_sign_x", -1.0);
    declare_parameter("robot_rotation_axis_sign_y", 1.0);
    declare_parameter("robot_rotation_axis_sign_z", 1.0);
    declare_parameter("max_relative_translation", 0.60);
    declare_parameter("max_relative_rotation", 2.50);
  }

  void poseCallback(const PoseMsg & message, TimedPose & destination)
  {
    Eigen::Isometry3d pose;
    if (!finitePose(message, pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring non-finite XR or robot pose");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    destination.pose = pose;
    destination.received_at = Clock::now();
    destination.frame = message.header.frame_id;
    destination.valid = true;
  }

  bool fresh(const TimedPose & pose, double timeout) const
  {
    return pose.valid &&
           std::chrono::duration<double>(Clock::now() - pose.received_at).count() <= timeout;
  }

  bool gripPressed(int button_index, int axis_index) const
  {
    if (!joy_.valid ||
      std::chrono::duration<double>(Clock::now() - joy_.received_at).count() >
      get_parameter("joy_timeout_sec").as_double()) {
      return false;
    }
    bool pressed = false;
    if (button_index >= 0 && button_index < static_cast<int>(joy_.message.buttons.size())) {
      pressed = joy_.message.buttons[button_index] != 0;
    }
    if (axis_index >= 0 && axis_index < static_cast<int>(joy_.message.axes.size())) {
      pressed = pressed ||
        std::abs(joy_.message.axes[axis_index]) >= get_parameter("grip_axis_threshold").as_double();
    }
    return pressed;
  }

  void processSide(SideState & side, bool pressed)
  {
    const double input_timeout = get_parameter("input_timeout_sec").as_double();
    const double state_timeout = get_parameter("robot_state_timeout_sec").as_double();
    const bool input_fresh = fresh(side.xr_pose, input_timeout);
    const bool state_fresh = fresh(side.robot_pose, state_timeout);

    if (get_parameter("log_clutch_state").as_bool()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s clutch status: pressed=%s active=%s xr_fresh=%s robot_state_fresh=%s",
        side.name.c_str(), pressed ? "true" : "false", side.active ? "true" : "false",
        input_fresh ? "true" : "false", state_fresh ? "true" : "false");
    }

    if (!state_fresh) {
      side.active = false;
      side.target_valid = false;
      return;
    }

    if (!side.active) {
      side.target = side.robot_pose.pose;
      side.target_valid = true;
      if (pressed && input_fresh) {
        side.xr_anchor = side.xr_pose.pose;
        side.robot_anchor = side.robot_pose.pose;
        side.target = side.robot_anchor;
        side.active = true;
        RCLCPP_INFO(get_logger(), "%s arm clutch engaged", side.name.c_str());
      }
      return;
    }

    if (!pressed || !input_fresh) {
      side.active = false;
      side.target = side.robot_pose.pose;
      RCLCPP_INFO(
        get_logger(), "%s arm clutch released%s", side.name.c_str(),
        input_fresh ? "" : " (XR input timeout)");
      return;
    }

    Eigen::Matrix3d xr_to_robot_basis;
    xr_to_robot_basis <<
      get_parameter("xr_to_robot_m00").as_double(),
      get_parameter("xr_to_robot_m01").as_double(),
      get_parameter("xr_to_robot_m02").as_double(),
      get_parameter("xr_to_robot_m10").as_double(),
      get_parameter("xr_to_robot_m11").as_double(),
      get_parameter("xr_to_robot_m12").as_double(),
      get_parameter("xr_to_robot_m20").as_double(),
      get_parameter("xr_to_robot_m21").as_double(),
      get_parameter("xr_to_robot_m22").as_double();
    if (!xr_to_robot_basis.allFinite() ||
      std::abs(xr_to_robot_basis.determinant()) < 1e-8)
    {
      xr_to_robot_basis.setIdentity();
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Invalid XR-to-robot basis matrix; using identity mapping");
    }

    const Eigen::Isometry3d relative = boundedRelativeTransform(
      side.xr_anchor, side.xr_pose.pose, side.robot_anchor,
      get_parameter("translation_scale").as_double(),
      xr_to_robot_basis,
      Eigen::Vector3d(
        get_parameter("robot_translation_axis_sign_x").as_double(),
        side.name == "left" ?
          get_parameter("left_robot_translation_axis_sign_y").as_double() :
          get_parameter("right_robot_translation_axis_sign_y").as_double(),
        get_parameter("robot_translation_axis_sign_z").as_double()),
      get_parameter("translation_deadband_m").as_double(),
      get_parameter("invert_relative_rotation").as_bool(),
      get_parameter("rotation_scale").as_double(),
      Eigen::Vector3d(
        get_parameter("robot_rotation_axis_sign_x").as_double(),
        get_parameter("robot_rotation_axis_sign_y").as_double(),
        get_parameter("robot_rotation_axis_sign_z").as_double()),
      get_parameter("max_relative_translation").as_double(),
      get_parameter("max_relative_rotation").as_double());
    side.target = side.robot_anchor * relative;
    side.target_valid = true;
  }

  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    processSide(
      left_,
      gripPressed(
        get_parameter("left_grip_button").as_int(),
        get_parameter("left_grip_axis").as_int()));
    processSide(
      right_,
      gripPressed(
        get_parameter("right_grip_button").as_int(),
        get_parameter("right_grip_axis").as_int()));

    const rclcpp::Time stamp = now();
    const std::string frame = get_parameter("target_frame").as_string();
    if (left_.target_valid) {
      left_target_pub_->publish(toPoseMessage(left_.target, frame, stamp));
    }
    if (right_.target_valid) {
      right_target_pub_->publish(toPoseMessage(right_.target, frame, stamp));
    }
    // 0: clutch released/hold, 1: Grip active. Both translation and
    // orientation are always included in the published target pose.
    ModeMsg left_mode;
    left_mode.data = left_.active ? 1 : 0;
    ModeMsg right_mode;
    right_mode.data = right_.active ? 1 : 0;
    left_mode_pub_->publish(left_mode);
    right_mode_pub_->publish(right_mode);
  }

  SideState left_;
  SideState right_;
  TimedJoy joy_;
  std::mutex mutex_;

  rclcpp::Subscription<PoseMsg>::SharedPtr left_pose_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr right_pose_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr left_robot_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr right_robot_sub_;
  rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
  rclcpp::Publisher<PoseMsg>::SharedPtr left_target_pub_;
  rclcpp::Publisher<PoseMsg>::SharedPtr right_target_pub_;
  rclcpp::Publisher<ModeMsg>::SharedPtr left_mode_pub_;
  rclcpp::Publisher<ModeMsg>::SharedPtr right_mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<XrPoseClutchBridge>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "qiling_xr_pose_clutch_bridge fatal: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
