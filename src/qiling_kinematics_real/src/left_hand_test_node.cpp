#include <cerrno>
#include <chrono>
#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <poll.h>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include "qi/msg/hands_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr std::size_t kFingerCount = 6;

bool parseBinaryInput(const std::string & line, int & value)
{
  std::istringstream stream(line);
  char extra = '\0';
  if (!(stream >> value) || (stream >> extra)) {
    return false;
  }
  return value == 0 || value == 1;
}

bool readPositionParameter(
  rclcpp::Node & node, const char * name,
  const std::array<std::uint16_t, kFingerCount> & defaults,
  std::array<std::uint16_t, kFingerCount> & values)
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

std::string formatPosition(
  const std::array<std::uint16_t, kFingerCount> & values)
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("left_hand_test_node");

  // These are the O6 example poses published by LinkerHand. They avoid
  // driving every joint to the same endpoint and are kept configurable for
  // the real-robot bench test.
  const std::array<std::uint16_t, kFingerCount> default_open_position =
    {255, 104, 255, 255, 255, 255};
  const std::array<std::uint16_t, kFingerCount> default_close_position =
    {101, 60, 0, 0, 0, 0};
  auto open_position = default_open_position;
  auto close_position = default_close_position;
  int speed = 200;
  if (!readPositionParameter(*node, "open_position", default_open_position, open_position) ||
    !readPositionParameter(*node, "close_position", default_close_position, close_position))
  {
    rclcpp::shutdown();
    return 2;
  }
  speed = node->declare_parameter<int>("speed", speed);
  if (speed < 0 || speed > std::numeric_limits<std::uint8_t>::max()) {
    RCLCPP_ERROR(node->get_logger(), "speed must be in [0, 255], got %d", speed);
    rclcpp::shutdown();
    return 2;
  }

  auto publisher = node->create_publisher<qi::msg::HandsCmd>("/handscmd", 10);

  RCLCPP_WARN(
    node->get_logger(),
    "Manual left-hand test only: input 0=open, 1=close, q=quit. "
    "No command is sent until 0 or 1 is entered.");
  RCLCPP_WARN(
    node->get_logger(),
    "open_position=%s, close_position=%s, speed=%d; right hand is sent "
    "the open pose because the SDK command contains both hands.",
    formatPosition(open_position).c_str(), formatPosition(close_position).c_str(), speed);

  std::string line;
  std::cout << "left hand [0=open, 1=close, q=quit] > " << std::flush;
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    pollfd input_fd{};
    input_fd.fd = STDIN_FILENO;
    input_fd.events = POLLIN;
    const int poll_result = ::poll(&input_fd, 1, 100);
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(node->get_logger(), "poll(stdin) failed: errno=%d", errno);
      break;
    }
    if (poll_result == 0 || !(input_fd.revents & (POLLIN | POLLHUP))) {
      continue;
    }

    if (!std::getline(std::cin, line)) {
      break;
    }

    if (line == "q" || line == "Q") {
      break;
    }

    int input = -1;
    if (!parseBinaryInput(line, input)) {
      RCLCPP_WARN(node->get_logger(), "Only 0, 1, or q is accepted; no command sent");
      std::cout << "left hand [0=open, 1=close, q=quit] > " << std::flush;
      continue;
    }

    const auto & target_position = input == 1 ? close_position : open_position;
    // The legacy qi message calls this field "durations", but the current
    // SDK forwards it as O6 command 0x05, which is the speed field.
    const auto command_speed = static_cast<std::uint16_t>(speed);

    qi::msg::HandsCmd command;
    command.mode = 1;
    command.mode_ctrl = 1;
    command.timestamp = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    // The SDK uses a fixed two-hand command and currently ignores the
    // per-hand mode in syncHandToContext(). Keep the right hand at its
    // known startup-open target while testing the left hand.
    for (std::size_t i = 0; i < kFingerCount; ++i) {
      command.hands[0].positions[i] = target_position[i];
      command.hands[0].durations[i] = command_speed;
      command.hands[1].positions[i] = open_position[i];
      command.hands[1].durations[i] = command_speed;
    }
    command.hands[0].mode = 1;
    command.hands[0].hand_id = 0;
    command.hands[1].mode = 1;
    command.hands[1].hand_id = 1;

    if (publisher->get_subscription_count() == 0) {
      RCLCPP_WARN(
        node->get_logger(),
        "No /handscmd subscriber discovered; command will still be published once");
    }
    publisher->publish(command);
    const auto target_text = formatPosition(target_position);
    const auto open_text = formatPosition(open_position);
    RCLCPP_INFO(
      node->get_logger(),
      "sent left hand %s: positions=%s, right hand held at open_position=%s",
      input == 1 ? "CLOSE" : "OPEN",
      target_text.c_str(), open_text.c_str());
    std::cout << "left hand [0=open, 1=close, q=quit] > " << std::flush;
  }

  rclcpp::shutdown();
  return 0;
}
