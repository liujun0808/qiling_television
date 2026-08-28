#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace
{

using Clock = std::chrono::steady_clock;
using JoyMsg = sensor_msgs::msg::Joy;
using HomeCompleteMsg = std_msgs::msg::Bool;
using StateMsg = std_msgs::msg::UInt8;

struct TriggerState
{
  double value{0.0};
  bool valid{false};
};

struct DebouncedState
{
  bool stable_closed{false};
  bool candidate_closed{false};
  Clock::time_point candidate_since{};
};

}  // namespace

class O6TriggerStateNode final : public rclcpp::Node
{
public:
  O6TriggerStateNode()
  : Node("qiling_o6_trigger_state")
  {
    declare_parameter("joy_topic", std::string("/xr/controller_joy"));
    declare_parameter("state_topic", std::string("/teleop/o6_trigger_state"));
    declare_parameter(
      "home_complete_topic", std::string("/teleop/startup_home_complete"));
    declare_parameter("require_home_complete", true);
    declare_parameter("left_trigger_axis", 0);
    declare_parameter("right_trigger_axis", 1);
    declare_parameter("close_threshold", 0.75);
    declare_parameter("release_threshold", 0.25);
    declare_parameter("debounce_sec", 0.08);
    declare_parameter("joy_timeout_sec", 0.20);
    declare_parameter("publish_rate_hz", 90.0);

    joy_sub_ = create_subscription<JoyMsg>(
      get_parameter("joy_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const JoyMsg::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (message->axes.size() <= static_cast<std::size_t>(
            std::max(get_parameter("left_trigger_axis").as_int(),
            get_parameter("right_trigger_axis").as_int()))) {
          return;
        }
        left_.value = finiteAxis(
          message->axes[static_cast<std::size_t>(get_parameter("left_trigger_axis").as_int())]);
        right_.value = finiteAxis(
          message->axes[static_cast<std::size_t>(get_parameter("right_trigger_axis").as_int())]);
        left_.valid = true;
        right_.valid = true;
        joy_received_at_ = Clock::now();
      });

    home_complete_sub_ = create_subscription<HomeCompleteMsg>(
      get_parameter("home_complete_topic").as_string(), rclcpp::QoS(1),
      [this](const HomeCompleteMsg::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        home_complete_ = message->data;
      });

    state_pub_ = create_publisher<StateMsg>(
      get_parameter("state_topic").as_string(), rclcpp::QoS(1));

    const double rate = std::max(get_parameter("publish_rate_hz").as_double(), 1.0);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / rate));
    timer_ = create_wall_timer(period, std::bind(&O6TriggerStateNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "O6 trigger state ready: left axis=%d, right axis=%d, close=%.2f, release=%.2f, debounce=%.3f s",
      static_cast<int>(get_parameter("left_trigger_axis").as_int()),
      static_cast<int>(get_parameter("right_trigger_axis").as_int()),
      get_parameter("close_threshold").as_double(),
      get_parameter("release_threshold").as_double(),
      get_parameter("debounce_sec").as_double());
  }

private:
  static double finiteAxis(float value)
  {
    return std::isfinite(value) ? std::clamp(static_cast<double>(value), 0.0, 1.0) : 0.0;
  }

  bool updateDebounced(
    TriggerState input, DebouncedState & state, Clock::time_point now, const char * side)
  {
    const double close_threshold = get_parameter("close_threshold").as_double();
    const double release_threshold = get_parameter("release_threshold").as_double();
    const double debounce_sec = std::max(get_parameter("debounce_sec").as_double(), 0.0);

    bool requested = state.stable_closed;
    if (!input.valid) {
      requested = false;
    } else if (state.stable_closed) {
      requested = input.value > release_threshold;
    } else {
      requested = input.value >= close_threshold;
    }

    if (requested != state.candidate_closed) {
      state.candidate_closed = requested;
      state.candidate_since = now;
    }

    if (state.candidate_closed != state.stable_closed &&
      std::chrono::duration<double>(now - state.candidate_since).count() >= debounce_sec)
    {
      state.stable_closed = state.candidate_closed;
      RCLCPP_INFO(
        get_logger(), "%s O6 trigger %s", side,
        state.stable_closed ? "closed" : "open");
      return true;
    }
    return false;
  }

  void tick()
  {
    TriggerState left;
    TriggerState right;
    Clock::time_point joy_received_at;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      left = left_;
      right = right_;
      joy_received_at = joy_received_at_;
    }

    const double timeout = std::max(get_parameter("joy_timeout_sec").as_double(), 0.0);
    const bool fresh = left.valid && right.valid &&
      std::chrono::duration<double>(Clock::now() - joy_received_at).count() <= timeout;
    if (!fresh) {
      left.valid = false;
      right.valid = false;
    }

    const auto now = Clock::now();
    bool home_complete;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      home_complete = home_complete_;
    }
    if (get_parameter("require_home_complete").as_bool() && !home_complete) {
      left_state_ = DebouncedState{};
      right_state_ = DebouncedState{};
      StateMsg message;
      message.data = 0;
      state_pub_->publish(message);
      return;
    }
    updateDebounced(left, left_state_, now, "left");
    updateDebounced(right, right_state_, now, "right");

    StateMsg message;
    // Bit 0: left O6 closed. Bit 1: right O6 closed.
    message.data = static_cast<std::uint8_t>(
      (left_state_.stable_closed ? 0x01 : 0x00) |
      (right_state_.stable_closed ? 0x02 : 0x00));
    state_pub_->publish(message);
  }

  std::mutex mutex_;
  TriggerState left_;
  TriggerState right_;
  Clock::time_point joy_received_at_{};
  DebouncedState left_state_;
  DebouncedState right_state_;
  rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
  rclcpp::Subscription<HomeCompleteMsg>::SharedPtr home_complete_sub_;
  rclcpp::Publisher<StateMsg>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool home_complete_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<O6TriggerStateNode>());
  rclcpp::shutdown();
  return 0;
}
