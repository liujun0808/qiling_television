#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "qi/msg/hands_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace
{

constexpr std::size_t kFingerCount = 6;
constexpr std::uint8_t kLeftMask = 0x01;
constexpr std::uint8_t kRightMask = 0x02;

using Position = std::array<std::uint16_t, kFingerCount>;

bool readPositionParameter(
  rclcpp::Node & node, const char * name, const Position & defaults, Position & values)
{
  std::vector<int64_t> default_values(defaults.begin(), defaults.end());
  const auto parameter = node.declare_parameter<std::vector<int64_t>>(name, default_values);
  if (parameter.size() != kFingerCount) {
    RCLCPP_ERROR(
      node.get_logger(), "%s must contain exactly %zu values", name, kFingerCount);
    return false;
  }

  for (std::size_t i = 0; i < kFingerCount; ++i) {
    if (parameter[i] < 0 || parameter[i] > std::numeric_limits<std::uint8_t>::max()) {
      RCLCPP_ERROR(
        node.get_logger(), "%s[%zu] must be in [0, 255], got %ld",
        name, i, parameter[i]);
      return false;
    }
    values[i] = static_cast<std::uint16_t>(parameter[i]);
  }
  return true;
}

std::string formatPosition(const Position & values)
{
  std::ostringstream stream;
  stream << '[';
  for (std::size_t i = 0; i < kFingerCount; ++i) {
    if (i != 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << ']';
  return stream.str();
}

}  // namespace

class O6CommandAdapterNode final : public rclcpp::Node
{
public:
  O6CommandAdapterNode()
  : Node("qiling_o6_command_adapter")
  {
    declare_parameter("state_topic", std::string("/teleop/o6_trigger_state"));
    declare_parameter("command_topic", std::string("/handscmd"));
    declare_parameter(
      "home_complete_topic", std::string("/teleop/startup_home_complete"));
    declare_parameter("require_home_complete", true);

    const Position default_open = {255, 104, 255, 255, 255, 255};
    const Position default_close = {101, 60, 0, 0, 0, 0};
    if (!readPositionParameter(*this, "open_position", default_open, open_position_) ||
      !readPositionParameter(*this, "close_position", default_close, close_position_))
    {
      throw std::runtime_error("invalid O6 position parameter");
    }

    speed_ = declare_parameter<int>("speed", 200);
    if (speed_ < 0 || speed_ > std::numeric_limits<std::uint8_t>::max()) {
      throw std::runtime_error("speed must be in [0, 255]");
    }

    const auto state_topic = get_parameter("state_topic").as_string();
    const auto command_topic = get_parameter("command_topic").as_string();
    const auto home_topic = get_parameter("home_complete_topic").as_string();

    command_pub_ = create_publisher<qi::msg::HandsCmd>(command_topic, rclcpp::QoS(10));
    state_sub_ = create_subscription<std_msgs::msg::UInt8>(
      state_topic, rclcpp::QoS(1),
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        onTriggerState(message->data);
      });
    home_sub_ = create_subscription<std_msgs::msg::Bool>(
      home_topic, rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        home_complete_ = message->data;
        RCLCPP_INFO(
          get_logger(), "startup home complete=%s", home_complete_ ? "true" : "false");
      });

    RCLCPP_INFO(
      get_logger(),
      "O6 command adapter ready: state=%s, command=%s, speed=%d, "
      "hold-last-target-on-input-loss=true",
      state_topic.c_str(), command_topic.c_str(), speed_);
    RCLCPP_INFO(
      get_logger(), "open=%s, close=%s",
      formatPosition(open_position_).c_str(), formatPosition(close_position_).c_str());
  }

private:
  void onTriggerState(std::uint8_t raw_state)
  {
    const std::uint8_t state = raw_state & (kLeftMask | kRightMask);
    if (get_parameter("require_home_complete").as_bool() && !home_complete_) {
      return;
    }

    if (!have_state_) {
      have_state_ = true;
      last_state_ = state;
      publishCommand(state, "initial O6 state");
      return;
    }

    const std::uint8_t changed = state ^ last_state_;
    if (changed == 0) {
      return;
    }

    last_state_ = state;
    publishCommand(state, "O6 trigger state changed");
  }

  void publishCommand(std::uint8_t state, const char * reason)
  {
    const Position & left_position = (state & kLeftMask) != 0 ? close_position_ : open_position_;
    const Position & right_position = (state & kRightMask) != 0 ? close_position_ : open_position_;

    qi::msg::HandsCmd command;
    command.mode = 1;
    command.mode_ctrl = 1;
    command.timestamp = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    for (std::size_t i = 0; i < kFingerCount; ++i) {
      command.hands[0].positions[i] = left_position[i];
      command.hands[0].durations[i] = static_cast<std::uint16_t>(speed_);
      command.hands[1].positions[i] = right_position[i];
      command.hands[1].durations[i] = static_cast<std::uint16_t>(speed_);
    }
    command.hands[0].mode = 1;
    command.hands[0].hand_id = 0;
    command.hands[1].mode = 1;
    command.hands[1].hand_id = 1;

    command_pub_->publish(command);
    RCLCPP_INFO(
      get_logger(),
      "%s: left=%s, right=%s, speed=%d",
      reason, formatPosition(left_position).c_str(), formatPosition(right_position).c_str(), speed_);
  }

  Position open_position_{};
  Position close_position_{};
  int speed_{200};
  bool home_complete_{false};
  bool have_state_{false};
  std::uint8_t last_state_{0};
  rclcpp::Publisher<qi::msg::HandsCmd>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<O6CommandAdapterNode>());
  } catch (const std::exception & error) {
    std::cerr << "O6 command adapter failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
