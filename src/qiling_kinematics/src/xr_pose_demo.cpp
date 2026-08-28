#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class XrPoseDemo final : public rclcpp::Node
{
public:
  XrPoseDemo()
  : Node("qiling_xr_pose_demo"), start_time_(std::chrono::steady_clock::now())
  {
    declare_parameter("publish_rate_hz", 50.0);
    declare_parameter("left_pose_topic", std::string("/xr/left_controller_pose"));
    declare_parameter("right_pose_topic", std::string("/xr/right_controller_pose"));
    declare_parameter("joy_topic", std::string("/xr/controller_joy"));
    declare_parameter("grip_start_sec", 2.0);
    declare_parameter("grip_end_sec", 12.0);
    declare_parameter("motion_amplitude_m", 0.06);
    declare_parameter("motion_amplitude_rad", 0.25);

    left_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("left_pose_topic").as_string(), rclcpp::QoS(1));
    right_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("right_pose_topic").as_string(), rclcpp::QoS(1));
    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>(
      get_parameter("joy_topic").as_string(), rclcpp::QoS(1));

    const double rate = std::max(get_parameter("publish_rate_hz").as_double(), 1.0);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / rate));
    timer_ = create_wall_timer(period, std::bind(&XrPoseDemo::tick, this));
    RCLCPP_INFO(
      get_logger(), "XR demo publishes static pose, then moves both controllers after %.1f s",
      get_parameter("grip_start_sec").as_double());
  }

private:
  geometry_msgs::msg::PoseStamped makePose(double side_sign, double elapsed) const
  {
    const double grip_start = get_parameter("grip_start_sec").as_double();
    const double t = std::max(0.0, elapsed - grip_start);
    const double amplitude = get_parameter("motion_amplitude_m").as_double();
    const double angle_amplitude = get_parameter("motion_amplitude_rad").as_double();
    const double x = amplitude * std::sin(0.7 * t);
    const double z = 0.5 * amplitude * (1.0 - std::cos(0.7 * t));
    const double yaw = side_sign * angle_amplitude * std::sin(0.5 * t);

    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = "xr_origin";
    message.pose.position.x = x;
    message.pose.position.y = side_sign * 0.25;
    message.pose.position.z = z;
    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = std::sin(yaw * 0.5);
    message.pose.orientation.w = std::cos(yaw * 0.5);
    return message;
  }

  void tick()
  {
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
    const double grip_start = get_parameter("grip_start_sec").as_double();
    const double grip_end = get_parameter("grip_end_sec").as_double();
    const bool grip = elapsed >= grip_start && elapsed < grip_end;

    left_pose_pub_->publish(makePose(1.0, elapsed));
    right_pose_pub_->publish(makePose(-1.0, elapsed));

    sensor_msgs::msg::Joy joy;
    joy.header.stamp = now();
    joy.buttons.assign(6, 0);
    joy.axes.assign(0, 0.0F);
    joy.buttons[4] = grip ? 1 : 0;
    joy.buttons[5] = grip ? 1 : 0;
    joy_pub_->publish(joy);
  }

  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XrPoseDemo>());
  rclcpp::shutdown();
  return 0;
}
