#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mit_msgs/msg/mit_joint_commands.hpp>
#include <mit_msgs/msg/mit_low_state.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "qiling_kinematics/arm_control_state.hpp"
#include "qiling_kinematics/anthropomorphic_elbow.hpp"
#include "qiling_kinematics/dual_arm_kinematics.hpp"
#include "qiling_kinematics/hierarchical_dik_solver.hpp"
#include "qiling_kinematics/ik_diagnostics.hpp"
#include "qiling_kinematics/ik_types.hpp"

namespace
{

constexpr int kArmDof = 14;
constexpr int kSideCount = 2;
constexpr int kLeftSide = 0;
constexpr int kRightSide = 1;
constexpr int kLegMotorCount = 12;
constexpr int kBodyMotorCount = 26;
constexpr int kCommandSize = kBodyMotorCount;
constexpr int kLeftArmCommandOffset = 12;
constexpr int kRightArmCommandOffset = 19;

using Clock = std::chrono::steady_clock;
using PoseMsg = geometry_msgs::msg::PoseStamped;
using ModeMsg = std_msgs::msg::UInt8;
using Trigger = std_srvs::srv::Trigger;
using qiling_kinematics::ArmControlState;
using qiling_kinematics::ArmKinematics;
using qiling_kinematics::ArmScalarJacobian;
using qiling_kinematics::ArmRunState;
using qiling_kinematics::ArmSide;
using qiling_kinematics::ArmVector;
using qiling_kinematics::AnthropomorphicElbow;
using qiling_kinematics::AnthropomorphicElbowConfig;
using qiling_kinematics::AnthropomorphicElbowOutput;
using qiling_kinematics::computeElbowSwivelJacobian;
using qiling_kinematics::DualArmKinematics;
using qiling_kinematics::HierarchicalDIKSolver;
using qiling_kinematics::IkDiagnostics;
using qiling_kinematics::SolverStatus;
using qiling_kinematics::kSingleArmDof;

struct TimedTarget
{
  pinocchio::SE3 pose{pinocchio::SE3::Identity()};
  Clock::time_point received_at{};
  bool received{false};
};

struct WorkspaceGuardState
{
  double bad_duration_sec{0.0};
};

enum class StartupHomePhase
{
  WaitingForState,
  MoveToTransition,
  SettleAtTransition,
  MoveToHome,
  SettleAtHome,
  TransitionToTeleop,
  HoldForRecording,
  MoveDirectToHome,
  Complete,
  Fault,
};

const char * toString(StartupHomePhase phase)
{
  switch (phase) {
    case StartupHomePhase::WaitingForState:
      return "WAITING_FOR_STATE";
    case StartupHomePhase::MoveToTransition:
      return "MOVE_TO_TRANSITION";
    case StartupHomePhase::SettleAtTransition:
      return "SETTLE_AT_TRANSITION";
    case StartupHomePhase::MoveToHome:
      return "MOVE_TO_HOME";
    case StartupHomePhase::SettleAtHome:
      return "SETTLE_AT_HOME";
    case StartupHomePhase::TransitionToTeleop:
      return "TRANSITION_TO_TELEOP";
    case StartupHomePhase::HoldForRecording:
      return "HOLD_FOR_RECORDING";
    case StartupHomePhase::MoveDirectToHome:
      return "MOVE_DIRECT_TO_HOME";
    case StartupHomePhase::Complete:
      return "COMPLETE";
    case StartupHomePhase::Fault:
      return "FAULT";
  }
  return "UNKNOWN";
}

double ageSeconds(Clock::time_point now, Clock::time_point stamp, bool valid)
{
  if (!valid) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(now - stamp).count();
}

bool computeHomeElbowDirection(
  const ArmKinematics & arm, double minimum_radius, Eigen::Vector3d & direction)
{
  const Eigen::Vector3d shoulder_to_wrist =
    arm.wrist_pose.translation() - arm.shoulder_pose.translation();
  const double distance = shoulder_to_wrist.norm();
  if (!std::isfinite(distance) || distance <= 1.0e-12) {
    return false;
  }
  const Eigen::Vector3d axis = shoulder_to_wrist / distance;
  direction =
    (Eigen::Matrix3d::Identity() - axis * axis.transpose()) *
    (arm.elbow_pose.translation() - arm.shoulder_pose.translation());
  const double radius = direction.norm();
  if (!std::isfinite(radius) || radius <= minimum_radius) {
    return false;
  }
  direction /= radius;
  return direction.allFinite();
}

bool validJointRange(const ArmVector & q_min, const ArmVector & q_max)
{
  return q_min.allFinite() && q_max.allFinite() &&
         (q_min.array() < q_max.array()).all();
}

bool outsideJointRange(
  const ArmVector & measured, const ArmVector & q_min, const ArmVector & q_max)
{
  if (!measured.allFinite() || !validJointRange(q_min, q_max)) {
    return true;
  }
  return (measured.array() < q_min.array()).any() ||
         (measured.array() > q_max.array()).any();
}

bool insideJointSafetyRange(
  const ArmVector & measured, const ArmVector & q_min, const ArmVector & q_max,
  double margin)
{
  if (!measured.allFinite() || !validJointRange(q_min, q_max) ||
    !std::isfinite(margin) || margin < 0.0)
  {
    return false;
  }
  return (measured.array() >= (q_min.array() + margin)).all() &&
         (measured.array() <= (q_max.array() - margin)).all();
}

ArmVector clampToJointRange(
  const ArmVector & measured, const ArmVector & q_min, const ArmVector & q_max)
{
  ArmVector clamped = measured;
  for (int i = 0; i < kSingleArmDof; ++i) {
    clamped[i] = std::clamp(measured[i], q_min[i], q_max[i]);
  }
  return clamped;
}

ArmVector limitRecoveryTarget(
  const ArmVector & reference, const ArmVector & q_min, const ArmVector & q_max,
  double margin)
{
  ArmVector target = reference;
  for (int i = 0; i < kSingleArmDof; ++i) {
    const double safe_margin = std::min(
      std::max(margin, 0.0), 0.5 * (q_max[i] - q_min[i]));
    if (reference[i] < q_min[i] + safe_margin) {
      target[i] = q_min[i] + safe_margin;
    } else if (reference[i] > q_max[i] - safe_margin) {
      target[i] = q_max[i] - safe_margin;
    }
  }
  return target;
}

ArmVector moveReferenceTowards(
  const ArmVector & current, const ArmVector & target, double max_step)
{
  ArmVector next = current;
  const double step = std::max(max_step, 0.0);
  for (int i = 0; i < kSingleArmDof; ++i) {
    next[i] = std::clamp(target[i], current[i] - step, current[i] + step);
  }
  return next;
}

double minJointLimitDistance(
  const ArmVector & measured, const ArmVector & q_min, const ArmVector & q_max)
{
  double minimum = std::numeric_limits<double>::infinity();
  for (int i = 0; i < kSingleArmDof; ++i) {
    minimum = std::min(minimum, std::min(measured[i] - q_min[i], q_max[i] - measured[i]));
  }
  return minimum;
}

}  // namespace

class DifferentialIkNode final : public rclcpp::Node
{
public:
  DifferentialIkNode()
  : Node("qiling_differential_ik"),
    left_control_("left"),
    right_control_("right")
  {
    declareParameters();
    loadModel();
    validateLimitRecoveryParameters();
    configureSolvers();

    lower_state_sub_ = create_subscription<mit_msgs::msg::MITLowState>(
      get_parameter("lower_state_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&DifferentialIkNode::lowerStateCallback, this, std::placeholders::_1));
    left_target_sub_ = create_subscription<PoseMsg>(
      get_parameter("left_target_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const PoseMsg::SharedPtr message) { updateTarget(*message, kLeftSide); });
    right_target_sub_ = create_subscription<PoseMsg>(
      get_parameter("right_target_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const PoseMsg::SharedPtr message) { updateTarget(*message, kRightSide); });
    left_mode_sub_ = create_subscription<ModeMsg>(
      get_parameter("left_mode_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const ModeMsg::SharedPtr message) { updateMode(*message, kLeftSide); });
    right_mode_sub_ = create_subscription<ModeMsg>(
      get_parameter("right_mode_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const ModeMsg::SharedPtr message) { updateMode(*message, kRightSide); });

    command_pub_ = create_publisher<mit_msgs::msg::MITJointCommands>(
      get_parameter("command_topic").as_string(), rclcpp::QoS(1));
    left_state_pub_ = create_publisher<PoseMsg>(
      get_parameter("left_state_topic").as_string(), rclcpp::QoS(1));
    right_state_pub_ = create_publisher<PoseMsg>(
      get_parameter("right_state_topic").as_string(), rclcpp::QoS(1));
    home_complete_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("startup_home_complete_topic").as_string(),
      rclcpp::QoS(1).transient_local());
    home_state_pub_ = create_publisher<std_msgs::msg::UInt8>(
      get_parameter("home_state_topic").as_string(),
      rclcpp::QoS(1).transient_local());
    recording_hold_service_ = create_service<Trigger>(
      get_parameter("recording_hold_service").as_string(),
      std::bind(
        &DifferentialIkNode::recordingHoldService, this,
        std::placeholders::_1, std::placeholders::_2));
    home_request_service_ = create_service<Trigger>(
      get_parameter("home_request_service").as_string(),
      std::bind(
        &DifferentialIkNode::homeRequestService, this,
        std::placeholders::_1, std::placeholders::_2));

    const double rate = get_parameter("control_rate_hz").as_double();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(rate, 1.0)));
    last_control_tick_ = Clock::now();
    control_timer_ = create_wall_timer(period, std::bind(&DifferentialIkNode::controlTick, this));

    RCLCPP_INFO(
      get_logger(),
      "Differential IK Phase 6 ready: nq=%d nv=%d, strict position -> orientation "
      "null-space QP, target/singularity/workspace/slew protection, gravity feedforward, "
      "elbow geometry=DIAGNOSTIC, elbow control=%s, %.1f Hz",
      static_cast<int>(kinematics_->model().nq),
      static_cast<int>(kinematics_->model().nv),
      get_parameter("elbow_task_enabled").as_bool() ? "ON" : "OFF", rate);
  }

private:
  void declareParameters()
  {
    declare_parameter("control_rate_hz", 50.0);
    declare_parameter("min_control_dt_sec", 0.01);
    declare_parameter("max_control_dt_sec", 0.04);
    declare_parameter("control_stall_fault_sec", 0.10);

    declare_parameter("lower_state_topic", std::string("/human_lower_state"));
    declare_parameter("command_topic", std::string("/human_lower_command"));
    declare_parameter("left_target_topic", std::string("/teleop/left_wrist_target"));
    declare_parameter("right_target_topic", std::string("/teleop/right_wrist_target"));
    declare_parameter("left_mode_topic", std::string("/teleop/left_control_mode"));
    declare_parameter("right_mode_topic", std::string("/teleop/right_control_mode"));
    declare_parameter("left_state_topic", std::string("/teleop/left_wrist_state"));
    declare_parameter("right_state_topic", std::string("/teleop/right_wrist_state"));
    declare_parameter(
      "startup_home_complete_topic", std::string("/teleop/startup_home_complete"));
    declare_parameter("home_state_topic", std::string("/teleop/home_state"));
    declare_parameter(
      "recording_hold_service", std::string("/teleop/request_recording_hold"));
    declare_parameter("home_request_service", std::string("/teleop/request_home"));

    declare_parameter("target_timeout_sec", 0.25);
    declare_parameter("joint_state_timeout_sec", 0.20);
    declare_parameter("target_frame", std::string("base_link"));
    declare_parameter("target_filter_enabled", true);
    declare_parameter("target_filter_time_constant_sec", 0.04);
    declare_parameter("workspace_guard_enabled", true);
    declare_parameter("workspace_guard_error_m", 0.03);
    declare_parameter("workspace_guard_timeout_sec", 0.25);
    declare_parameter("workspace_guard_limit_distance_rad", 0.003);
    declare_parameter("workspace_guard_singularity_sigma", 0.003);
    declare_parameter("orientation_task_enabled", true);
    declare_parameter("position_gain", 3.0);
    declare_parameter("max_linear_velocity_mps", 0.20);
    declare_parameter("position_error_deadband_m", 0.003);
    declare_parameter("position_sigma_slowdown_start", 0.05);
    declare_parameter("position_sigma_stop", 0.003);
    declare_parameter("position_singularity_speed_scale_min", 0.0);
    declare_parameter("position_regularization", 1.0e-4);
    declare_parameter("position_smoothness_weight", 1.0e-3);
    declare_parameter("rotation_gain", 3.0);
    declare_parameter("max_angular_velocity_rps", 0.80);
    declare_parameter("orientation_error_deadband_rad", 0.01);
    declare_parameter("wrist_sigma_slowdown_start", 0.08);
    declare_parameter("wrist_sigma_stop", 0.005);
    declare_parameter("orientation_singularity_speed_scale_min", 0.0);
    declare_parameter("orientation_regularization", 1.0e-4);
    declare_parameter("orientation_smoothness_weight", 1.0e-3);
    declare_parameter("rank_threshold", 1.0e-4);
    declare_parameter("characteristic_length_m", 0.25);
    declare_parameter("position_sigma_warn", 0.05);
    declare_parameter("wrist_sigma_warn", 0.08);
    declare_parameter("max_orientation_position_degradation_mps", 5.0e-4);
    declare_parameter("elbow_task_enabled", true);
    declare_parameter("elbow_swivel_gain", 2.0);
    declare_parameter("max_elbow_swivel_velocity_rps", 0.80);
    declare_parameter("elbow_posture_weight", 0.08);
    declare_parameter("elbow_joint_centering_weight", 0.03);
    declare_parameter("elbow_smoothness_weight", 0.05);
    declare_parameter("elbow_regularization", 1.0e-4);
    declare_parameter("elbow_sigma_fade_start", 0.08);
    declare_parameter("elbow_sigma_disable", 0.04);
    declare_parameter("max_elbow_position_degradation_mps", 5.0e-4);
    declare_parameter("max_elbow_orientation_degradation_rps", 2.0e-3);
    declare_parameter("log_pose_error", false);
    declare_parameter("joint_limit_margin_rad", 0.08);
    declare_parameter("joint_limit_damper_gain", 1.0);
    declare_parameter("hard_limit_tolerance_rad", 0.005);
    declare_parameter("active_entry_margin_rad", 0.05);
    declare_parameter("limit_recovery_margin_rad", 0.08);
    declare_parameter("limit_recovery_max_velocity_rps", 0.25);
    declare_parameter("joint_limit_prediction_enabled", true);
    declare_parameter("joint_limit_prediction_delay_sec", 0.04);
    declare_parameter("joint_limit_prediction_margin_rad", 0.02);
    declare_parameter("measured_velocity_filter_alpha", 0.25);
    declare_parameter("measured_velocity_clamp_rps", 3.0);
    declare_parameter("left_q_rest", std::vector<double>{
      0.0104905777, 0.6509880424, -0.1741435826, -1.7084382772,
      0.0738155171, -0.0963225737, -0.0936522484});
    declare_parameter("right_q_rest", std::vector<double>{
      -0.0749599487, -0.6475547552, 0.1073853672, -1.2964446545,
      -0.0009536889, -0.4251545072, 0.4945830405});
    declare_parameter("left_home_transition_q", std::vector<double>{
      1.00, 1.20, 0.0, -1.20, 0.0, 0.0, 0.0});
    declare_parameter("right_home_transition_q", std::vector<double>{
      1.00, -1.20, 0.0, -1.20, 0.0, 0.0, 0.0});
    declare_parameter("elbow_geometry_diagnostic_enabled", true);
    declare_parameter("elbow_outward_weight", 1.0);
    declare_parameter("elbow_downward_weight", 0.30);
    declare_parameter("elbow_home_weight", 0.80);
    declare_parameter("min_shoulder_wrist_distance_m", 0.05);
    declare_parameter("min_elbow_projection_radius_m", 0.02);
    declare_parameter<std::vector<double>>(
      "max_joint_velocity_rps", {1.0, 1.0, 1.0, 1.2, 1.2, 1.2, 1.2});
    declare_parameter<std::vector<double>>(
      "max_joint_acceleration_rps2", {3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0});

    declare_parameter("gravity_compensation_enabled", true);
    declare_parameter("gravity_torque_scale", 1.0);
    declare_parameter("gravity_torque_limit_scale", 1.0);
    declare_parameter("gravity_transition_duration_sec", 0.50);

    declare_parameter("startup_home_enabled", true);
    declare_parameter("home_transition_duration_sec", 2.5);
    declare_parameter("home_move_duration_sec", 3.0);
    declare_parameter("home_settle_duration_sec", 0.30);
    declare_parameter("home_transition_tolerance_rad", 0.15);
    declare_parameter("home_tolerance_rad", 0.15);
    declare_parameter("home_settle_timeout_sec", 4.0);
    declare_parameter("home_hold_transition_duration_sec", 0.50);
    declare_parameter("home_to_teleop_transition_duration_sec", 0.50);
    declare_parameter("home_log_period_ms", 1000);

    declare_parameter("consecutive_primary_qp_failures_to_fault", 3);
    declare_parameter("log_state_transitions", true);
    declare_parameter("diagnostics_log_period_ms", 2000);

    declare_parameter("command_kp", 40.0);
    declare_parameter("command_kd", 2.0);
    declare_parameter("max_reference_tracking_error_rad", 0.35);
    declare_parameter("qp_max_iter", 80);
    declare_parameter("qp_eps_abs", 1e-5);
    declare_parameter("qp_eps_rel", 1e-5);
  }

  void loadModel()
  {
    const std::string description_share =
      ament_index_cpp::get_package_share_directory("qi_robot_description");
    const auto urdf_path = std::filesystem::path(description_share) / "urdf" / "s4_dual_arm.urdf";
    kinematics_ = std::make_unique<DualArmKinematics>(urdf_path);
    const auto readVector = [this](const char * parameter_name) {
        const auto values = get_parameter(parameter_name).as_double_array();
        if (values.size() != kSingleArmDof) {
          throw std::runtime_error(std::string(parameter_name) + " must contain exactly 7 values");
        }
        ArmVector result;
        for (int i = 0; i < kSingleArmDof; ++i) {
          result[i] = values[i];
        }
        if (!result.allFinite()) {
          throw std::runtime_error(std::string(parameter_name) + " contains non-finite values");
        }
        return result;
      };
    home_q_[kLeftSide] = readVector("left_q_rest");
    home_q_[kRightSide] = readVector("right_q_rest");
    home_transition_q_[kLeftSide] = readVector("left_home_transition_q");
    home_transition_q_[kRightSide] = readVector("right_home_transition_q");
    for (int side = 0; side < kSideCount; ++side) {
      const ArmSide arm_side = side == kLeftSide ? ArmSide::Left : ArmSide::Right;
      q_state_[side].setZero();
      q_min_[side] = kinematics_->lowerPositionLimits(arm_side);
      q_max_[side] = kinematics_->upperPositionLimits(arm_side);
    }
    const auto home_kinematics = kinematics_->evaluate(
      home_q_[kLeftSide], home_q_[kRightSide]);
    effort_limits_ = kinematics_->effortLimits();
    const double minimum_radius = get_parameter("min_elbow_projection_radius_m").as_double();
    for (int side = 0; side < kSideCount; ++side) {
      if (!computeHomeElbowDirection(home_kinematics[side], minimum_radius, home_elbow_direction_[side])) {
        throw std::runtime_error("unable to compute home elbow swivel direction");
      }
    }
  }

  void validateLimitRecoveryParameters() const
  {
    const double active_margin = get_parameter("active_entry_margin_rad").as_double();
    const double recovery_margin = get_parameter("limit_recovery_margin_rad").as_double();
    const double recovery_velocity =
      get_parameter("limit_recovery_max_velocity_rps").as_double();
    if (!std::isfinite(active_margin) || active_margin < 0.0) {
      throw std::runtime_error("active_entry_margin_rad must be finite and non-negative");
    }
    if (!std::isfinite(recovery_margin) || recovery_margin < active_margin) {
      throw std::runtime_error(
              "limit_recovery_margin_rad must be finite and no smaller than "
              "active_entry_margin_rad");
    }
    if (!std::isfinite(recovery_velocity) || recovery_velocity <= 0.0) {
      throw std::runtime_error("limit_recovery_max_velocity_rps must be finite and positive");
    }
    for (int side = 0; side < kSideCount; ++side) {
      for (int joint = 0; joint < kSingleArmDof; ++joint) {
        if (2.0 * recovery_margin >= q_max_[side][joint] - q_min_[side][joint]) {
          throw std::runtime_error(
                  "limit_recovery_margin_rad leaves no interior range for every arm joint");
        }
      }
    }
  }

  HierarchicalDIKSolver::Config solverConfig() const
  {
    HierarchicalDIKSolver::Config config;
    config.position_gain = get_parameter("position_gain").as_double();
    config.max_linear_velocity = get_parameter("max_linear_velocity_mps").as_double();
    config.position_error_deadband =
      get_parameter("position_error_deadband_m").as_double();
    config.position_sigma_slowdown_start =
      get_parameter("position_sigma_slowdown_start").as_double();
    config.position_sigma_stop = get_parameter("position_sigma_stop").as_double();
    config.position_singularity_speed_scale_min =
      get_parameter("position_singularity_speed_scale_min").as_double();
    config.position_regularization = get_parameter("position_regularization").as_double();
    config.position_smoothness_weight =
      get_parameter("position_smoothness_weight").as_double();
    config.rotation_gain = get_parameter("rotation_gain").as_double();
    config.max_angular_velocity = get_parameter("max_angular_velocity_rps").as_double();
    config.orientation_error_deadband =
      get_parameter("orientation_error_deadband_rad").as_double();
    config.wrist_sigma_slowdown_start =
      get_parameter("wrist_sigma_slowdown_start").as_double();
    config.wrist_sigma_stop = get_parameter("wrist_sigma_stop").as_double();
    config.orientation_singularity_speed_scale_min =
      get_parameter("orientation_singularity_speed_scale_min").as_double();
    config.orientation_regularization =
      get_parameter("orientation_regularization").as_double();
    config.orientation_smoothness_weight =
      get_parameter("orientation_smoothness_weight").as_double();
    config.rank_threshold = get_parameter("rank_threshold").as_double();
    config.characteristic_length = get_parameter("characteristic_length_m").as_double();
    config.max_orientation_position_degradation =
      get_parameter("max_orientation_position_degradation_mps").as_double();
    config.elbow_swivel_gain = get_parameter("elbow_swivel_gain").as_double();
    config.max_elbow_swivel_velocity =
      get_parameter("max_elbow_swivel_velocity_rps").as_double();
    config.elbow_posture_weight = get_parameter("elbow_posture_weight").as_double();
    config.elbow_joint_centering_weight =
      get_parameter("elbow_joint_centering_weight").as_double();
    config.elbow_smoothness_weight = get_parameter("elbow_smoothness_weight").as_double();
    config.elbow_regularization = get_parameter("elbow_regularization").as_double();
    config.elbow_sigma_fade_start = get_parameter("elbow_sigma_fade_start").as_double();
    config.elbow_sigma_disable = get_parameter("elbow_sigma_disable").as_double();
    config.max_elbow_position_degradation =
      get_parameter("max_elbow_position_degradation_mps").as_double();
    config.max_elbow_orientation_degradation =
      get_parameter("max_elbow_orientation_degradation_rps").as_double();
    config.joint_limit_margin = get_parameter("joint_limit_margin_rad").as_double();
    config.joint_limit_damper_gain = get_parameter("joint_limit_damper_gain").as_double();
    config.hard_limit_tolerance = get_parameter("hard_limit_tolerance_rad").as_double();
    config.joint_limit_prediction_enabled =
      get_parameter("joint_limit_prediction_enabled").as_bool();
    config.joint_limit_prediction_delay_sec =
      get_parameter("joint_limit_prediction_delay_sec").as_double();
    config.joint_limit_prediction_margin_rad =
      get_parameter("joint_limit_prediction_margin_rad").as_double();
    const auto max_joint_velocity = get_parameter("max_joint_velocity_rps").as_double_array();
    if (max_joint_velocity.size() != kSingleArmDof) {
      throw std::runtime_error("max_joint_velocity_rps must contain exactly 7 values");
    }
    for (int i = 0; i < kSingleArmDof; ++i) {
      config.max_joint_velocity_rps[i] = max_joint_velocity[i];
    }
    const auto max_joint_acceleration =
      get_parameter("max_joint_acceleration_rps2").as_double_array();
    if (max_joint_acceleration.size() != kSingleArmDof) {
      throw std::runtime_error("max_joint_acceleration_rps2 must contain exactly 7 values");
    }
    for (int i = 0; i < kSingleArmDof; ++i) {
      config.max_joint_acceleration_rps2[i] = max_joint_acceleration[i];
    }
    config.qp_max_iter = get_parameter("qp_max_iter").as_int();
    config.qp_eps_abs = get_parameter("qp_eps_abs").as_double();
    config.qp_eps_rel = get_parameter("qp_eps_rel").as_double();
    return config;
  }

  void configureSolvers()
  {
    const auto config = solverConfig();
    left_solver_ = std::make_unique<HierarchicalDIKSolver>(config);
    right_solver_ = std::make_unique<HierarchicalDIKSolver>(config);
    elbow_config_.outward_weight = get_parameter("elbow_outward_weight").as_double();
    elbow_config_.downward_weight = get_parameter("elbow_downward_weight").as_double();
    elbow_config_.home_weight = get_parameter("elbow_home_weight").as_double();
    elbow_config_.min_shoulder_wrist_distance =
      get_parameter("min_shoulder_wrist_distance_m").as_double();
    elbow_config_.min_elbow_projection_radius =
      get_parameter("min_elbow_projection_radius_m").as_double();
  }

  TimedTarget filteredTarget(int side, const TimedTarget & raw, double dt)
  {
    if (!get_parameter("target_filter_enabled").as_bool()) {
      return raw;
    }
    const double time_constant =
      get_parameter("target_filter_time_constant_sec").as_double();
    if (!std::isfinite(time_constant) || time_constant <= 0.0) {
      return raw;
    }
    if (!filtered_target_valid_[side]) {
      filtered_targets_[side] = raw;
      filtered_target_valid_[side] = true;
      return filtered_targets_[side];
    }

    const double alpha = std::clamp(dt / (time_constant + dt), 0.0, 1.0);
    const Eigen::Vector3d position =
      (1.0 - alpha) * filtered_targets_[side].pose.translation() +
      alpha * raw.pose.translation();
    Eigen::Quaterniond current(filtered_targets_[side].pose.rotation());
    Eigen::Quaterniond desired(raw.pose.rotation());
    if (current.dot(desired) < 0.0) {
      desired.coeffs() *= -1.0;
    }
    Eigen::Quaterniond blended = current.slerp(alpha, desired);
    if (!blended.coeffs().allFinite() || blended.norm() < 1.0e-8 ||
      !position.allFinite()) {
      filtered_targets_[side] = raw;
      return filtered_targets_[side];
    }
    blended.normalize();
    filtered_targets_[side].pose = pinocchio::SE3(blended.toRotationMatrix(), position);
    filtered_targets_[side].received_at = raw.received_at;
    filtered_targets_[side].received = raw.received;
    return filtered_targets_[side];
  }

  void resetTargetAndGuard(int side)
  {
    filtered_target_valid_[side] = false;
    workspace_guard_[side].bad_duration_sec = 0.0;
  }

  void updateMeasuredVelocity(
    int side, const ArmVector & measured, bool fresh, double dt, bool control_stalled)
  {
    if (!fresh || control_stalled || !measured.allFinite() ||
      !std::isfinite(dt) || dt <= 0.0)
    {
      measured_velocity_initialized_[side] = false;
      measured_velocity_[side].setZero();
      return;
    }

    if (!measured_velocity_initialized_[side]) {
      previous_measured_q_[side] = measured;
      measured_velocity_[side].setZero();
      measured_velocity_initialized_[side] = true;
      return;
    }

    const double velocity_clamp = get_parameter("measured_velocity_clamp_rps").as_double();
    const double alpha = std::clamp(
      get_parameter("measured_velocity_filter_alpha").as_double(), 0.0, 1.0);
    if (!std::isfinite(velocity_clamp) || velocity_clamp <= 0.0 ||
      !std::isfinite(alpha))
    {
      measured_velocity_[side].setZero();
      previous_measured_q_[side] = measured;
      return;
    }

    ArmVector raw_velocity = (measured - previous_measured_q_[side]) / dt;
    previous_measured_q_[side] = measured;
    for (int i = 0; i < kSingleArmDof; ++i) {
      raw_velocity[i] = std::clamp(raw_velocity[i], -velocity_clamp, velocity_clamp);
    }
    measured_velocity_[side] =
      (1.0 - alpha) * measured_velocity_[side] + alpha * raw_velocity;
    if (!measured_velocity_[side].allFinite()) {
      measured_velocity_[side].setZero();
    }
  }

  std::string formatArmVector(const ArmVector & values) const
  {
    std::ostringstream stream;
    stream << "[";
    for (int i = 0; i < kSingleArmDof; ++i) {
      if (i != 0) {
        stream << ", ";
      }
      stream << values[i];
    }
    stream << "]";
    return stream.str();
  }

  void logInvalidBounds(
    int side,
    const char * side_name,
    const ArmVector & measured,
    const ArmVector & q_min,
    const ArmVector & q_max,
    const HierarchicalDIKSolver::Result & result,
    double dt)
  {
    const ArmSide arm_side = side == kLeftSide ? ArmSide::Left : ArmSide::Right;
    const auto & joint_names = kinematics_->jointNames(arm_side);
    int bad_joint = -1;
    const char * reason = "no direct measured-position violation";

    for (int i = 0; i < kSingleArmDof; ++i) {
      if (!std::isfinite(q_min[i]) || !std::isfinite(q_max[i]) || q_min[i] > q_max[i]) {
        bad_joint = i;
        reason = "invalid configured/URDF position bounds";
        break;
      }
      if (!std::isfinite(measured[i]) || measured[i] < q_min[i] || measured[i] > q_max[i]) {
        bad_joint = i;
        reason = "measured position outside URDF position bounds";
        break;
      }
    }

    if (bad_joint >= 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s position-primary INVALID_BOUNDS: %s; joint=%s[%d] "
        "q_measured=%.9f urdf_range=[%.9f, %.9f] dt=%.6f "
        "min_hard_limit_distance=%.9f qdot=%s",
        side_name, reason, joint_names[bad_joint].c_str(), bad_joint,
        measured[bad_joint], q_min[bad_joint], q_max[bad_joint], dt,
        result.min_hard_limit_distance_rad, formatArmVector(result.qdot).c_str());
      return;
    }

    // INVALID_BOUNDS can also be caused by an infeasible velocity-bound
    // intersection even when the measured position itself is inside the
    // configured hard limits. Print all vectors so that case is distinguishable
    // from an actual URDF/SDK joint mapping problem.
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "%s position-primary INVALID_BOUNDS: %s; dt=%.6f "
      "min_hard_limit_distance=%.9f q_measured=%s q_min=%s q_max=%s qdot=%s",
      side_name, reason, dt, result.min_hard_limit_distance_rad,
      formatArmVector(measured).c_str(), formatArmVector(q_min).c_str(),
      formatArmVector(q_max).c_str(), formatArmVector(result.qdot).c_str());
  }

  void logReferenceIntegrationFailure(
    int side,
    const char * side_name,
    const ArmControlState & control,
    const ArmVector & qdot,
    double dt,
    const ArmVector & measured,
    const ArmVector & q_min,
    const ArmVector & q_max,
    double max_tracking_error)
  {
    const ArmSide arm_side = side == kLeftSide ? ArmSide::Left : ArmSide::Right;
    const auto & joint_names = kinematics_->jointNames(arm_side);
    const ArmVector & reference = control.reference();

    if (!control.initialized() || control.state() != ArmRunState::Active) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s reference integration rejected: invalid control state initialized=%s "
        "state=%s dt=%.6f q_measured=%s q_ref=%s qdot=%s",
        side_name, control.initialized() ? "true" : "false",
        qiling_kinematics::toString(control.state()), dt,
        formatArmVector(measured).c_str(), formatArmVector(reference).c_str(),
        formatArmVector(qdot).c_str());
      return;
    }

    const char * scalar_reason = nullptr;
    if (!qdot.allFinite()) {
      scalar_reason = "qdot contains non-finite values";
    } else if (!measured.allFinite()) {
      scalar_reason = "measured position contains non-finite values";
    } else if (!q_min.allFinite() || !q_max.allFinite()) {
      scalar_reason = "configured/URDF bounds contain non-finite values";
    } else if (!std::isfinite(dt) || dt <= 0.0) {
      scalar_reason = "invalid control dt";
    } else if (!std::isfinite(max_tracking_error) || max_tracking_error < 0.0) {
      scalar_reason = "invalid max tracking error";
    }

    if (scalar_reason != nullptr) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s reference integration rejected: reason=%s dt=%.9f "
        "max_tracking_error=%.9f q_measured=%s q_ref=%s qdot=%s q_min=%s q_max=%s",
        side_name, scalar_reason, dt, max_tracking_error,
        formatArmVector(measured).c_str(), formatArmVector(reference).c_str(),
        formatArmVector(qdot).c_str(), formatArmVector(q_min).c_str(),
        formatArmVector(q_max).c_str());
      return;
    }

    for (int i = 0; i < kSingleArmDof; ++i) {
      const double integrated = reference[i] + dt * qdot[i];
      const double tracking_min = std::max(q_min[i], measured[i] - max_tracking_error);
      const double tracking_max = std::min(q_max[i], measured[i] + max_tracking_error);
      const char * reason = nullptr;
      if (q_min[i] > q_max[i]) {
        reason = "lower bound is greater than upper bound";
      } else if (measured[i] < q_min[i] || measured[i] > q_max[i]) {
        reason = "measured position outside URDF position bounds";
      } else if (!std::isfinite(reference[i])) {
        reason = "previous reference is non-finite";
      } else if (!std::isfinite(integrated)) {
        reason = "q_measured + qdot * dt is non-finite";
      } else if (tracking_min > tracking_max) {
        reason = "measured tracking interval has no intersection with URDF bounds";
      }

      if (reason != nullptr) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "%s reference integration rejected: reason=%s; joint=%s[%d] "
          "q_measured=%.9f urdf_range=[%.9f, %.9f] q_ref=%.9f "
          "qdot=%.9f q_integrated=%.9f tracking_range=[%.9f, %.9f] "
          "dt=%.9f max_tracking_error=%.9f",
          side_name, reason, joint_names[i].c_str(), i, measured[i],
          q_min[i], q_max[i], reference[i], qdot[i], integrated,
          tracking_min, tracking_max, dt, max_tracking_error);
        return;
      }
    }

    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "%s reference integration rejected: no single-joint cause identified; "
      "q_measured=%s q_ref=%s qdot=%s q_min=%s q_max=%s dt=%.9f "
      "max_tracking_error=%.9f",
      side_name, formatArmVector(measured).c_str(), formatArmVector(reference).c_str(),
      formatArmVector(qdot).c_str(), formatArmVector(q_min).c_str(),
      formatArmVector(q_max).c_str(), dt, max_tracking_error);
  }

  void lowerStateCallback(const mit_msgs::msg::MITLowState::SharedPtr message)
  {
    const auto & positions = message->joint_states.position;
    if (positions.size() != kBodyMotorCount) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring /human_lower_state with %zu positions; expected 26 body motors",
        positions.size());
      return;
    }

    for (int i = 0; i < kBodyMotorCount; ++i) {
      if (!std::isfinite(positions[i])) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring /human_lower_state with non-finite position at motor %d", i);
        return;
      }
    }

    const auto received_at = Clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < kLegMotorCount; ++i) {
      leg_q_state_[i] = positions[i];
    }
    for (int i = 0; i < kSingleArmDof; ++i) {
      q_state_[kLeftSide][i] = positions[kLeftArmCommandOffset + i];
      q_state_[kRightSide][i] = positions[kRightArmCommandOffset + i];
    }
    arm_state_received_[kLeftSide] = true;
    arm_state_received_[kRightSide] = true;
    last_arm_state_time_[kLeftSide] = received_at;
    last_arm_state_time_[kRightSide] = received_at;
  }

  void updateTarget(const PoseMsg & message, int side)
  {
    const std::string required_frame = get_parameter("target_frame").as_string();
    if (message.header.frame_id != required_frame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring target in frame '%s'; expected exact frame '%s'",
        message.header.frame_id.c_str(), required_frame.c_str());
      return;
    }

    Eigen::Quaterniond quaternion(
      message.pose.orientation.w, message.pose.orientation.x,
      message.pose.orientation.y, message.pose.orientation.z);
    const double quaternion_norm = quaternion.norm();
    if (!std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-8) {
      return;
    }
    quaternion.normalize();

    const Eigen::Vector3d position(
      message.pose.position.x, message.pose.position.y, message.pose.position.z);
    if (!position.allFinite()) {
      return;
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
    targets_[side].pose = pinocchio::SE3(quaternion.toRotationMatrix(), position);
    targets_[side].received_at = Clock::now();
    targets_[side].received = true;
  }

  void updateMode(const ModeMsg & message, int side)
  {
    if (message.data > 2) {
      return;
    }
    std::lock_guard<std::mutex> lock(mode_mutex_);
    requested_modes_[side] = message.data;
  }

  bool getFreshMeasuredArms(std::array<ArmVector, kSideCount> & measured)
  {
    const auto now_steady = Clock::now();
    std::array<Clock::time_point, kSideCount> state_times;
    std::array<bool, kSideCount> state_received;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      measured = q_state_;
      state_times = last_arm_state_time_;
      state_received = arm_state_received_;
    }

    const double timeout = get_parameter("joint_state_timeout_sec").as_double();
    if (!std::isfinite(timeout) || timeout <= 0.0) {
      return false;
    }
    for (int side = 0; side < kSideCount; ++side) {
      const double age = ageSeconds(now_steady, state_times[side], state_received[side]);
      if (!state_received[side] || !measured[side].allFinite() ||
        !std::isfinite(age) || age > timeout)
      {
        return false;
      }
    }
    return true;
  }

  void recordingHoldService(
    const Trigger::Request::SharedPtr /*request*/, Trigger::Response::SharedPtr response)
  {
    std::array<ArmVector, kSideCount> measured;
    if (!home_complete_ || home_phase_ != StartupHomePhase::Complete ||
      !getFreshMeasuredArms(measured))
    {
      response->success = false;
      response->message =
        "recording hold rejected: robot is not in READY home state or feedback is stale";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    // X (success) and A (failure) both arrive here. A bumpless MIT-control
    // handoff must preserve the command sent on the previous cycle; preserving
    // only the measured pose is insufficient because Kp * (pos - measured) is
    // part of the supporting torque. Hold the exact last published position.
    home_hold_start_q_ = last_published_command_initialized_ ?
      last_published_positions_ :
      std::array<ArmVector, kSideCount>{
      left_control_.reference(), right_control_.reference()};
    if (!home_hold_start_q_[kLeftSide].allFinite() ||
      !home_hold_start_q_[kRightSide].allFinite())
    {
      response->success = false;
      response->message =
        "recording hold rejected: no finite previously published arm command";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    home_hold_target_q_ = home_hold_start_q_;
    home_start_q_ = home_hold_target_q_;
    home_command_q_ = home_hold_start_q_;
    for (auto & velocity : home_command_qdot_) {
      velocity.setZero();
    }
    home_complete_ = false;
    teleop_side_ready_.fill(false);
    left_control_.enterHold(measured[kLeftSide], true, "recording episode boundary");
    right_control_.enterHold(measured[kRightSide], true, "recording episode boundary");
    // enterHold() establishes the release guard and may temporarily latch the
    // measurement. Restore the exact command before the next publish; clearing
    // the release guard later also preserves this reference.
    left_control_.synchronizeReference(
      home_hold_target_q_[kLeftSide], "recording command latched");
    right_control_.synchronizeReference(
      home_hold_target_q_[kRightSide], "recording command latched");
    // The robot must remain supported while waiting for the B (return-home)
    // request.  X/A only changes the recording state; it must not remove the
    // gravity feedforward and let the arms sag during the recording hold.
    beginGravityBlend(
      1.0, std::max(get_parameter("gravity_transition_duration_sec").as_double(), 0.0));
    resetTargetAndGuard(kLeftSide);
    resetTargetAndGuard(kRightSide);
    elbow_geometry_[kLeftSide].reset();
    elbow_geometry_[kRightSide].reset();
    setHomePhase(
      StartupHomePhase::HoldForRecording,
      "episode ended; holding last published command");

    response->success = true;
    response->message = "recording hold accepted; press home request after marking episode";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void homeRequestService(
    const Trigger::Request::SharedPtr /*request*/, Trigger::Response::SharedPtr response)
  {
    std::array<ArmVector, kSideCount> measured;
    if (home_complete_ || home_phase_ != StartupHomePhase::HoldForRecording ||
      !getFreshMeasuredArms(measured))
    {
      response->success = false;
      response->message =
        "home request rejected: robot is not holding a completed episode or feedback is stale";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    // Start from the command that was actually sent on the previous cycle.
    // Starting from measured feedback would remove the existing Kp position
    // error in one frame and create an actuator-torque step.
    home_start_q_ = last_published_command_initialized_ ?
      last_published_positions_ : home_command_q_;
    if (!home_start_q_[kLeftSide].allFinite() ||
      !home_start_q_[kRightSide].allFinite())
    {
      home_start_q_ = home_hold_target_q_;
    }
    if (!home_start_q_[kLeftSide].allFinite() ||
      !home_start_q_[kRightSide].allFinite())
    {
      response->success = false;
      response->message = "home request rejected: no finite held arm command";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    home_command_q_ = home_start_q_;
    for (auto & velocity : home_command_qdot_) {
      velocity.setZero();
    }
    // Fade gravity feedforward over the complete direct-home trajectory. The
    // position trajectory and effort transition therefore share the same
    // quintic duration instead of removing several N.m in the first 0.5 s.
    beginGravityBlend(
      0.0, std::max(get_parameter("home_move_duration_sec").as_double(), 0.1));
    teleop_side_ready_.fill(false);
    setHomePhase(StartupHomePhase::MoveDirectToHome, "direct home requested");

    response->success = true;
    response->message = "direct held-command to home interpolation started";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  static double maxJointError(
    const std::array<ArmVector, kSideCount> & measured,
    const std::array<ArmVector, kSideCount> & target)
  {
    double result = 0.0;
    for (int side = 0; side < kSideCount; ++side) {
      result = std::max(result, (measured[side] - target[side]).cwiseAbs().maxCoeff());
    }
    return result;
  }

  static void quinticSegment(
    const ArmVector & start, const ArmVector & end, double elapsed, double duration,
    ArmVector & position, ArmVector & velocity)
  {
    if (!std::isfinite(duration) || duration <= 0.0) {
      position = end;
      velocity.setZero();
      return;
    }
    const double u = std::clamp(elapsed / duration, 0.0, 1.0);
    const double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    const double dsdu =
      30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u;
    position = start + s * (end - start);
    velocity = (dsdu / duration) * (end - start);
  }

  void publishHomeStatus()
  {
    std_msgs::msg::Bool message;
    message.data = home_complete_;
    home_complete_pub_->publish(message);

    std_msgs::msg::UInt8 state;
    switch (home_phase_) {
      case StartupHomePhase::WaitingForState:
        state.data = 0;
        break;
      case StartupHomePhase::MoveToTransition:
      case StartupHomePhase::MoveToHome:
      case StartupHomePhase::MoveDirectToHome:
      case StartupHomePhase::TransitionToTeleop:
        state.data = 1;
        break;
      case StartupHomePhase::SettleAtTransition:
      case StartupHomePhase::SettleAtHome:
        state.data = 2;
        break;
      case StartupHomePhase::Complete:
        state.data = 3;
        break;
      case StartupHomePhase::Fault:
        state.data = 4;
        break;
      case StartupHomePhase::HoldForRecording:
        state.data = 5;
        break;
    }
    home_state_pub_->publish(state);
  }

  void setHomePhase(StartupHomePhase phase, const char * reason)
  {
    home_phase_ = phase;
    home_phase_time_sec_ = 0.0;
    home_settle_time_sec_ = 0.0;
    if (phase != StartupHomePhase::Complete && phase != StartupHomePhase::HoldForRecording &&
      phase != StartupHomePhase::MoveDirectToHome)
    {
      resetGravityBlend();
    }
    RCLCPP_INFO(
      get_logger(), "home phase -> %s (%s)", toString(home_phase_), reason);
  }

  void resetGravityBlend()
  {
    gravity_blend_scale_ = 0.0;
    gravity_blend_start_scale_ = 0.0;
    gravity_blend_target_scale_ = 0.0;
    gravity_blend_elapsed_sec_ = 0.0;
    gravity_blend_duration_sec_ = 0.0;
    gravity_blend_active_ = false;
  }

  void beginGravityBlend(double target_scale, double duration_sec)
  {
    const double target = std::clamp(
      std::isfinite(target_scale) ? target_scale : 0.0, 0.0, 1.0);
    const double duration =
      std::isfinite(duration_sec) ? std::max(duration_sec, 0.0) : 0.0;
    gravity_blend_start_scale_ = std::clamp(gravity_blend_scale_, 0.0, 1.0);
    gravity_blend_target_scale_ = target;
    gravity_blend_elapsed_sec_ = 0.0;
    gravity_blend_duration_sec_ = duration;
    if (duration <= 0.0 || std::abs(gravity_blend_start_scale_ - target) <= 1.0e-9) {
      gravity_blend_scale_ = target;
      gravity_blend_active_ = false;
    } else {
      gravity_blend_active_ = true;
    }
  }

  void updateGravityBlend(double dt)
  {
    if (!gravity_blend_active_) {
      return;
    }
    const double safe_dt = std::isfinite(dt) && dt > 0.0 ? dt : 0.0;
    gravity_blend_elapsed_sec_ += safe_dt;
    const double u = std::clamp(
      gravity_blend_elapsed_sec_ / gravity_blend_duration_sec_, 0.0, 1.0);
    const double smooth = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    gravity_blend_scale_ = gravity_blend_start_scale_ +
      smooth * (gravity_blend_target_scale_ - gravity_blend_start_scale_);
    if (u >= 1.0) {
      gravity_blend_scale_ = gravity_blend_target_scale_;
      gravity_blend_active_ = false;
    }
  }

  void beginStartupHome(const std::array<ArmVector, kSideCount> & measured)
  {
    // The real robot may not actually be at mathematical zero. Start from the
    // first complete measured arm state and make the transition trajectory
    // continuous from that physical pose.
    home_start_q_ = measured;
    home_command_q_ = measured;
    home_hold_start_q_ = measured;
    home_hold_target_q_ = measured;
    home_handoff_start_q_ = measured;
    home_handoff_target_q_ = measured;
    resetGravityBlend();
    for (auto & velocity : home_command_qdot_) {
      velocity.setZero();
    }
    home_started_ = true;
    teleop_side_ready_.fill(false);
    if (get_parameter("startup_home_enabled").as_bool()) {
      setHomePhase(StartupHomePhase::MoveToTransition, "complete real state received");
    } else {
      home_command_q_ = home_q_;
      if (!left_control_.initialized()) {
        left_control_.initialize(home_q_[kLeftSide]);
      }
      if (!right_control_.initialized()) {
        right_control_.initialize(home_q_[kRightSide]);
      }
      beginGravityBlend(
        1.0, std::max(get_parameter("gravity_transition_duration_sec").as_double(), 0.0));
      setHomePhase(StartupHomePhase::Complete, "startup home disabled");
      home_complete_ = true;
    }
  }

  void updateStartupHome(
    const std::array<ArmVector, kSideCount> & measured, double dt)
  {
    if (!home_started_ || home_phase_ == StartupHomePhase::Fault) {
      for (int side = 0; side < kSideCount; ++side) {
        home_command_q_[side] =
          home_phase_ == StartupHomePhase::Fault ? measured[side] :
          home_phase_ == StartupHomePhase::HoldForRecording ?
          home_start_q_[side] : home_q_[side];
        home_command_qdot_[side].setZero();
      }
      return;
    }

    const double safe_dt = std::isfinite(dt) && dt > 0.0 ? dt : 0.02;
    home_phase_time_sec_ += safe_dt;
    const double transition_duration =
      std::max(get_parameter("home_transition_duration_sec").as_double(), 0.1);
    const double home_duration =
      std::max(get_parameter("home_move_duration_sec").as_double(), 0.1);
    const double hold_transition_duration =
      std::max(get_parameter("home_hold_transition_duration_sec").as_double(), 0.0);
    const double home_to_teleop_duration =
      std::max(get_parameter("home_to_teleop_transition_duration_sec").as_double(), 0.0);
    const double settle_duration =
      std::max(get_parameter("home_settle_duration_sec").as_double(), 0.0);
    const double transition_tolerance =
      std::max(get_parameter("home_transition_tolerance_rad").as_double(), 0.0);
    const double home_tolerance =
      std::max(get_parameter("home_tolerance_rad").as_double(), 0.0);
    const double settle_timeout =
      std::max(get_parameter("home_settle_timeout_sec").as_double(), 0.1);

    switch (home_phase_) {
      case StartupHomePhase::MoveToTransition:
        for (int side = 0; side < kSideCount; ++side) {
          quinticSegment(
            home_start_q_[side], home_transition_q_[side], home_phase_time_sec_,
            transition_duration, home_command_q_[side], home_command_qdot_[side]);
        }
        if (home_phase_time_sec_ >= transition_duration) {
          home_command_q_ = home_transition_q_;
          for (auto & velocity : home_command_qdot_) {
            velocity.setZero();
          }
          setHomePhase(StartupHomePhase::SettleAtTransition, "trajectory finished");
        }
        break;

      case StartupHomePhase::SettleAtTransition:
        home_command_q_ = home_transition_q_;
        for (auto & velocity : home_command_qdot_) {
          velocity.setZero();
        }
        if (maxJointError(measured, home_transition_q_) <= transition_tolerance) {
          home_settle_time_sec_ += safe_dt;
          if (home_settle_time_sec_ >= settle_duration) {
            setHomePhase(StartupHomePhase::MoveToHome, "transition reached");
          }
        } else {
          home_settle_time_sec_ = 0.0;
        }
        if (home_phase_time_sec_ >= settle_timeout) {
          setHomePhase(StartupHomePhase::Fault, "transition pose was not reached");
        }
        break;

      case StartupHomePhase::MoveToHome:
        for (int side = 0; side < kSideCount; ++side) {
          quinticSegment(
            home_transition_q_[side], home_q_[side], home_phase_time_sec_, home_duration,
            home_command_q_[side], home_command_qdot_[side]);
        }
        if (home_phase_time_sec_ >= home_duration) {
          home_command_q_ = home_q_;
          for (auto & velocity : home_command_qdot_) {
            velocity.setZero();
          }
          setHomePhase(StartupHomePhase::SettleAtHome, "trajectory finished");
        }
        break;

      case StartupHomePhase::HoldForRecording:
        for (int side = 0; side < kSideCount; ++side) {
          quinticSegment(
            home_hold_start_q_[side], home_hold_target_q_[side], home_phase_time_sec_,
            hold_transition_duration, home_command_q_[side], home_command_qdot_[side]);
        }
        if (home_phase_time_sec_ >= hold_transition_duration) {
          home_command_q_ = home_hold_target_q_;
          for (auto & velocity : home_command_qdot_) {
            velocity.setZero();
          }
        }
        break;

      case StartupHomePhase::TransitionToTeleop:
        for (int side = 0; side < kSideCount; ++side) {
          quinticSegment(
            home_handoff_start_q_[side], home_handoff_target_q_[side],
            home_phase_time_sec_, home_to_teleop_duration,
            home_command_q_[side], home_command_qdot_[side]);
        }
        if (home_phase_time_sec_ >= home_to_teleop_duration) {
          home_command_q_ = home_handoff_target_q_;
          for (auto & velocity : home_command_qdot_) {
            velocity.setZero();
          }
          // At initial startup the arm controls have not been initialized
          // yet. Seed them with the handoff endpoint, not the raw measured
          // state on the next tick, so normal Hold cannot create a step.
          if (!left_control_.initialized()) {
            left_control_.initialize(home_handoff_target_q_[kLeftSide]);
          } else {
            left_control_.synchronizeReference(
              home_handoff_target_q_[kLeftSide], "home handoff reference synchronized");
          }
          if (!right_control_.initialized()) {
            right_control_.initialize(home_handoff_target_q_[kRightSide]);
          } else {
            right_control_.synchronizeReference(
              home_handoff_target_q_[kRightSide], "home handoff reference synchronized");
          }
          beginGravityBlend(
            1.0, std::max(get_parameter("gravity_transition_duration_sec").as_double(), 0.0));
          home_complete_ = true;
          setHomePhase(StartupHomePhase::Complete, "home-to-teleop handoff finished");
          RCLCPP_INFO(
            get_logger(),
            "home complete; release each Grip once before teleoperation");
        }
        break;

      case StartupHomePhase::MoveDirectToHome:
        for (int side = 0; side < kSideCount; ++side) {
          quinticSegment(
            home_start_q_[side], home_q_[side], home_phase_time_sec_, home_duration,
            home_command_q_[side], home_command_qdot_[side]);
        }
        if (home_phase_time_sec_ >= home_duration) {
          home_command_q_ = home_q_;
          for (auto & velocity : home_command_qdot_) {
            velocity.setZero();
          }
          setHomePhase(StartupHomePhase::SettleAtHome, "direct home trajectory finished");
        }
        break;

      case StartupHomePhase::SettleAtHome:
        home_command_q_ = home_q_;
        for (auto & velocity : home_command_qdot_) {
          velocity.setZero();
        }
        if (maxJointError(measured, home_q_) <= home_tolerance) {
          home_settle_time_sec_ += safe_dt;
          if (home_settle_time_sec_ >= settle_duration) {
            // home_q_ is still the command being published. Smoothly hand
            // that command to the measured pose before opening the normal
            // teleoperation path; otherwise the first processArm() tick can
            // replace home_q_ with q_measured in one cycle.
            home_handoff_start_q_ = home_q_;
            home_handoff_target_q_ = measured;
            setHomePhase(
              StartupHomePhase::TransitionToTeleop,
              "home reached; starting smooth handoff to teleoperation");
          }
        } else {
          home_settle_time_sec_ = 0.0;
        }
        if (home_phase_time_sec_ >= settle_timeout) {
          setHomePhase(StartupHomePhase::Fault, "home pose was not reached");
        }
        break;

      case StartupHomePhase::WaitingForState:
      case StartupHomePhase::Complete:
      case StartupHomePhase::Fault:
        home_command_q_ =
          home_phase_ == StartupHomePhase::Fault ? measured : home_q_;
        for (auto & velocity : home_command_qdot_) {
          velocity.setZero();
        }
        break;
    }
  }

  void processArm(
    int side,
    const char * side_name,
    const ArmKinematics & arm_kinematics,
    const ArmVector & measured,
    const TimedTarget & target,
    bool requested_active,
    bool state_received,
    double state_age,
    double target_age,
    bool control_stalled,
    double dt,
    ArmControlState & control,
    HierarchicalDIKSolver & solver,
    IkDiagnostics & diagnostics)
  {
    const ArmRunState previous_state = control.state();

    if (state_received && !control.initialized()) {
      control.initialize(measured);
    }

    diagnostics = IkDiagnostics{};
    diagnostics.target_age_sec = target_age;
    diagnostics.joint_state_age_sec = state_age;
    diagnostics.qdot_measured = measured_velocity_[side];

    if (!control.initialized()) {
      diagnostics.control_state = ArmRunState::Hold;
      diagnostics.command_held = true;
      return;
    }

    const double state_timeout = get_parameter("joint_state_timeout_sec").as_double();
    if (!state_received || !std::isfinite(state_age) || state_age > state_timeout) {
      // Keep a limit-recovery reference if the feedback stream temporarily
      // becomes stale. Replacing it with the last measured value could put an
      // already-out-of-range position straight back into the command.
      if (!control.recovering()) {
        control.enterHold(measured, requested_active, "JointState timeout");
      }
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    const ArmVector & q_min = q_min_[side];
    const ArmVector & q_max = q_max_[side];
    const bool measured_outside = outsideJointRange(measured, q_min, q_max);
    const double active_entry_margin =
      get_parameter("active_entry_margin_rad").as_double();
    const double recovery_margin =
      get_parameter("limit_recovery_margin_rad").as_double();
    const double recovery_velocity =
      get_parameter("limit_recovery_max_velocity_rps").as_double();

    if (control.recovering()) {
      control.updateGripRequest(requested_active, measured);
      if (control_stalled) {
        diagnostics.command_held = true;
        finishArmCycle(side_name, previous_state, control, diagnostics);
        return;
      }

      if (!measured_outside && insideJointSafetyRange(
          measured, q_min, q_max, active_entry_margin))
      {
        control.finishLimitRecovery(requested_active, "joint limit recovery complete");
        diagnostics.command_held = true;
        finishArmCycle(side_name, previous_state, control, diagnostics);
        return;
      }

      // Keep unrelated joints at their held reference. Only references that
      // are themselves near/outside a limit are moved into the recovery zone.
      // If an unsafe measured joint already has a safe held reference, waiting
      // for feedback to follow that reference avoids an unnecessary pose step.
      const ArmVector target_reference = limitRecoveryTarget(
        control.reference(), q_min, q_max, recovery_margin);
      const ArmVector next_reference = moveReferenceTowards(
        control.reference(), target_reference, recovery_velocity * dt);
      control.updateRecoveryReference(next_reference);
      resetTargetAndGuard(side);
      elbow_geometry_[side].reset();
      diagnostics.solver_status = SolverStatus::InvalidBounds;
      diagnostics.min_hard_limit_distance_rad =
        minJointLimitDistance(measured, q_min, q_max);
      diagnostics.qdot.setZero();
      diagnostics.command_held = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s LIMIT_RECOVERY: measured=%s reference=%s inward_target=%s "
        "urdf_range=[%s, %s] grip_pressed=%s",
        side_name, formatArmVector(measured).c_str(),
        formatArmVector(control.reference()).c_str(),
        formatArmVector(target_reference).c_str(), formatArmVector(q_min).c_str(),
        formatArmVector(q_max).c_str(), requested_active ? "true" : "false");
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    if (measured_outside) {
      // Never use the out-of-range feedback value as a position command. The
      // first recovery command is clamped to the exact URDF boundary and all
      // subsequent references move toward the interior recovery target.
      control.enterLimitRecovery(
        clampToJointRange(measured, q_min, q_max), requested_active,
        "measured position outside URDF range; limit recovery");
      resetTargetAndGuard(side);
      elbow_geometry_[side].reset();
      diagnostics.solver_status = SolverStatus::InvalidBounds;
      diagnostics.min_hard_limit_distance_rad =
        minJointLimitDistance(measured, q_min, q_max);
      diagnostics.qdot.setZero();
      diagnostics.command_held = true;
      RCLCPP_ERROR(
        get_logger(),
        "%s entered LIMIT_RECOVERY: measured=%s was outside URDF range; "
        "initial_reference=%s grip_pressed=%s",
        side_name, formatArmVector(measured).c_str(),
        formatArmVector(control.reference()).c_str(), requested_active ? "true" : "false");
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    // Check the measured state before updateGripRequest can create ACTIVE.
    // If feedback is still inside the URDF hard range but outside the ACTIVE
    // safety margin, recover the held reference into the interior at the same
    // bounded speed used for a hard-limit recovery. Recovery starts only from
    // an explicit Grip request, so an idle Hold never moves autonomously.
    if (requested_active && control.state() == ArmRunState::Hold &&
      !control.releaseRequired() && !insideJointSafetyRange(
        measured, q_min, q_max, active_entry_margin))
    {
      // Preserve the last published Hold command as the first recovery command
      // to keep this boundary bumpless. It is clamped only to the URDF hard
      // range; subsequent cycles move it toward the configured interior zone.
      const ArmVector initial_reference = clampToJointRange(
        control.reference(), q_min, q_max);
      control.enterLimitRecovery(
        initial_reference, true,
        "measured position outside ACTIVE safety range; limit recovery");
      resetTargetAndGuard(side);
      elbow_geometry_[side].reset();
      diagnostics.solver_status = SolverStatus::InvalidBounds;
      diagnostics.min_hard_limit_distance_rad =
        minJointLimitDistance(measured, q_min, q_max);
      diagnostics.qdot.setZero();
      diagnostics.command_held = true;
      const ArmVector target_reference = limitRecoveryTarget(
        initial_reference, q_min, q_max, recovery_margin);
      RCLCPP_WARN(
        get_logger(),
        "%s entered LIMIT_RECOVERY before ACTIVE: measured=%s reference=%s "
        "inward_target=%s safety_margin=%.4f recovery_margin=%.4f "
        "max_velocity=%.4f urdf_range=[%s, %s]; teleoperation remains disabled",
        side_name, formatArmVector(measured).c_str(),
        formatArmVector(initial_reference).c_str(),
        formatArmVector(target_reference).c_str(), active_entry_margin,
        recovery_margin, recovery_velocity, formatArmVector(q_min).c_str(),
        formatArmVector(q_max).c_str());
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    control.updateGripRequest(requested_active, measured);

    if (control_stalled) {
      control.enterHold(measured, requested_active, "control loop stall");
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    if (!requested_active) {
      resetTargetAndGuard(side);
      elbow_geometry_[side].reset();
      control.enterHold(measured, false, "Grip released");
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    const double target_timeout = get_parameter("target_timeout_sec").as_double();
    if (!target.received || !std::isfinite(target_age) || target_age > target_timeout) {
      resetTargetAndGuard(side);
      control.enterHold(measured, true, "target timeout");
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    if (!control.active()) {
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    const TimedTarget control_target = filteredTarget(side, target, dt);

    const bool orientation_enabled = get_parameter("orientation_task_enabled").as_bool();
    const bool elbow_task_enabled =
      get_parameter("elbow_task_enabled").as_bool() && orientation_enabled;
    const bool elbow_geometry_enabled =
      get_parameter("elbow_geometry_diagnostic_enabled").as_bool() || elbow_task_enabled;
    AnthropomorphicElbowOutput elbow_output;
    if (elbow_geometry_enabled) {
      elbow_output = elbow_geometry_[side].compute(
        side == kLeftSide ? ArmSide::Left : ArmSide::Right,
        arm_kinematics.shoulder_pose.translation(),
        arm_kinematics.elbow_pose.translation(),
        arm_kinematics.wrist_pose.translation(),
        home_elbow_direction_[side], elbow_config_);
      diagnostics.elbow_geometry_valid = elbow_output.valid;
      diagnostics.elbow_used_previous_direction = elbow_output.used_previous_direction;
      diagnostics.elbow_used_home_direction = elbow_output.used_home_direction;
      diagnostics.elbow_swivel_error_rad = elbow_output.signed_swivel_error;
      diagnostics.elbow_projection_radius_m = elbow_output.current_projection_radius;
      diagnostics.elbow_shoulder_wrist_distance_m = elbow_output.shoulder_wrist_distance;
      diagnostics.elbow_preferred_direction = elbow_output.preferred_direction;
    }

    HierarchicalDIKSolver::PositionInput position_input;
    position_input.jacobian = arm_kinematics.wrist_jacobian.topRows<3>();
    position_input.position_error =
      control_target.pose.translation() - arm_kinematics.wrist_pose.translation();
    position_input.q_measured = measured;
    position_input.q_min = q_min_[side];
    position_input.q_max = q_max_[side];
    position_input.qdot_previous = control.previousVelocity();
    position_input.qdot_measured = measured_velocity_[side];
    position_input.dt = dt;

    HierarchicalDIKSolver::Result result;
    if (orientation_enabled) {
      HierarchicalDIKSolver::PoseHierarchyInput pose_input;
      pose_input.position = position_input;
      pose_input.angular_jacobian = arm_kinematics.wrist_jacobian.bottomRows<3>();
      pose_input.orientation_error = pinocchio::log3(
        control_target.pose.rotation() * arm_kinematics.wrist_pose.rotation().transpose());
      if (elbow_task_enabled && elbow_output.valid) {
        pose_input.elbow_geometry_valid = true;
        pose_input.elbow_swivel_error = elbow_output.signed_swivel_error;
        pose_input.q_rest = home_q_[side];
        pose_input.elbow_swivel_jacobian = computeElbowSwivelJacobian(
          arm_kinematics.shoulder_pose.translation(),
          arm_kinematics.elbow_pose.translation(),
          arm_kinematics.wrist_pose.translation(),
          arm_kinematics.elbow_linear_jacobian,
          arm_kinematics.wrist_jacobian.topRows<3>(),
          elbow_output.shoulder_wrist_axis,
          elbow_output.current_direction,
          elbow_output.current_projection_radius);
      }
      result = solver.solvePoseHierarchy(pose_input);
    } else {
      result = solver.solvePositionPrimary(position_input);
    }
    diagnostics.solver_status = result.status;
    diagnostics.position_error_norm = result.position_error_norm;
    diagnostics.rotation_error_norm = result.rotation_error_norm;
    diagnostics.solve_time_us = result.solve_time_us;
    diagnostics.joint_limit_damper_active = result.joint_limit_damper_active;
    diagnostics.min_hard_limit_distance_rad = result.min_hard_limit_distance_rad;
    diagnostics.joint_limit_prediction_active = result.joint_limit_prediction_active;
    diagnostics.min_predicted_limit_distance_rad = result.min_predicted_limit_distance_rad;
    diagnostics.position_rank = result.position_rank;
    diagnostics.position_sigma_min = result.position_sigma_min;
    diagnostics.position_condition_number = result.position_condition_number;
    diagnostics.position_speed_scale = result.position_speed_scale;
    diagnostics.wrist_rank = result.wrist_rank;
    diagnostics.wrist_sigma_min = result.wrist_sigma_min;
    diagnostics.wrist_condition_number = result.wrist_condition_number;
    diagnostics.orientation_speed_scale = result.orientation_speed_scale;
    diagnostics.elbow_status = result.elbow_status;
    diagnostics.elbow_applied = result.elbow_applied;
    diagnostics.elbow_scale = result.elbow_scale;
    diagnostics.elbow_swivel_velocity_desired = result.elbow_swivel_velocity_desired;
    diagnostics.elbow_swivel_velocity_achieved = result.elbow_swivel_velocity_achieved;
    diagnostics.elbow_position_degradation_mps = result.elbow_position_degradation_mps;
    diagnostics.elbow_orientation_degradation_rps =
      result.elbow_orientation_degradation_rps;
    diagnostics.qdot = result.qdot;

    if (!result.success) {
      if (result.status == SolverStatus::InvalidBounds) {
        logInvalidBounds(
          side, side_name, measured, position_input.q_min, position_input.q_max, result, dt);
      }
      control.recordSolveFailure(
        measured,
        get_parameter("consecutive_primary_qp_failures_to_fault").as_int(),
        std::string("position-primary QP ") + qiling_kinematics::toString(result.status));
      diagnostics.command_held = true;
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    const bool guard_enabled = get_parameter("workspace_guard_enabled").as_bool();
    const double guard_error = get_parameter("workspace_guard_error_m").as_double();
    const double guard_timeout = get_parameter("workspace_guard_timeout_sec").as_double();
    const double guard_limit_distance =
      get_parameter("workspace_guard_limit_distance_rad").as_double();
    const double guard_sigma =
      get_parameter("workspace_guard_singularity_sigma").as_double();
    const bool guard_parameters_valid =
      std::isfinite(guard_error) && guard_error >= 0.0 &&
      std::isfinite(guard_timeout) && guard_timeout > 0.0 &&
      std::isfinite(guard_limit_distance) && guard_limit_distance >= 0.0 &&
      std::isfinite(guard_sigma) && guard_sigma >= 0.0;
    const bool persistent_boundary_request = guard_parameters_valid &&
      result.position_error_norm > guard_error &&
      (result.min_hard_limit_distance_rad <= guard_limit_distance ||
      result.position_sigma_min <= guard_sigma);
    if (guard_enabled && persistent_boundary_request) {
      workspace_guard_[side].bad_duration_sec += dt;
    } else {
      workspace_guard_[side].bad_duration_sec = 0.0;
    }
    if (guard_enabled && guard_parameters_valid &&
      workspace_guard_[side].bad_duration_sec >= guard_timeout)
    {
      control.enterHold(measured, true, "unreachable target or workspace boundary");
      diagnostics.qdot.setZero();
      diagnostics.command_held = true;
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    control.recordSolveSuccess();
    const double position_sigma_warn = get_parameter("position_sigma_warn").as_double();
    const double wrist_sigma_warn = get_parameter("wrist_sigma_warn").as_double();
    if (result.position_sigma_min < position_sigma_warn ||
      (get_parameter("orientation_task_enabled").as_bool() &&
      result.wrist_sigma_min < wrist_sigma_warn))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(),
        get_parameter("diagnostics_log_period_ms").as_int(),
        "%s near singularity: position rank=%d sigma_min=%.5f cond=%.1f; "
        "wrist rank=%d sigma_min=%.5f cond=%.1f",
        side_name, result.position_rank, result.position_sigma_min,
        result.position_condition_number, result.wrist_rank,
        result.wrist_sigma_min, result.wrist_condition_number);
    }
    const double max_tracking_error =
      get_parameter("max_reference_tracking_error_rad").as_double();
    if (!control.integrateReference(
        result.qdot, dt, measured, position_input.q_min, position_input.q_max,
        max_tracking_error))
    {
      logReferenceIntegrationFailure(
        side, side_name, control, result.qdot, dt, measured,
        position_input.q_min, position_input.q_max, max_tracking_error);
      control.enterFault(measured, "reference integration failed");
      diagnostics.command_held = true;
      finishArmCycle(side_name, previous_state, control, diagnostics);
      return;
    }

    diagnostics.command_held = false;
    finishArmCycle(side_name, previous_state, control, diagnostics);

    if (get_parameter("log_pose_error").as_bool()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(),
        get_parameter("diagnostics_log_period_ms").as_int(),
        "%s Phase 6 geometry valid=%s swivel_error=%.4f rad preferred=(%.2f,%.2f,%.2f); "
        "position=%.4f m orientation=%.4f rad orientation_status=%s "
        "orientation_applied=%s elbow_status=%s elbow_applied=%s elbow_scale=%.2f "
        "elbow_degradation=(%.6f m/s,%.6f rad/s) damper=%s limit_distance=%.4f rad "
        "sigma_position=%.5f sigma_wrist=%.5f speed=(%.2f,%.2f) solve=%.1f us",
        side_name, diagnostics.elbow_geometry_valid ? "true" : "false",
        diagnostics.elbow_swivel_error_rad,
        diagnostics.elbow_preferred_direction.x(),
        diagnostics.elbow_preferred_direction.y(),
        diagnostics.elbow_preferred_direction.z(),
        result.position_error_norm,
        result.rotation_error_norm,
        qiling_kinematics::toString(result.orientation_status),
        result.orientation_applied ? "true" : "false",
        qiling_kinematics::toString(result.elbow_status),
        result.elbow_applied ? "true" : "false",
        result.elbow_scale,
        result.elbow_position_degradation_mps,
        result.elbow_orientation_degradation_rps,
        result.joint_limit_damper_active ? "true" : "false",
        result.min_hard_limit_distance_rad,
        result.position_sigma_min,
        result.wrist_sigma_min,
        result.position_speed_scale,
        result.orientation_speed_scale,
        result.solve_time_us);
    }
  }

  void finishArmCycle(
    const char * side_name,
    ArmRunState previous_state,
    const ArmControlState & control,
    IkDiagnostics & diagnostics)
  {
    diagnostics.control_state = control.state();
    if (previous_state != control.state() &&
      get_parameter("log_state_transitions").as_bool())
    {
      RCLCPP_INFO(
        get_logger(), "%s arm %s -> %s: %s",
        side_name,
        qiling_kinematics::toString(previous_state),
        qiling_kinematics::toString(control.state()),
        control.reason().c_str());
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(),
      get_parameter("diagnostics_log_period_ms").as_int(),
      "%s state=%s held=%s solver=%s target_age=%.3f state_age=%.3f qdot_max=%.3f "
      "qdot_measured_max=%.3f prediction=%s predicted_limit_distance=%.4f "
      "damper=%s limit_distance=%.4f sigma_position=%.5f sigma_wrist=%.5f "
      "speed=(%.2f,%.2f) "
      "elbow_valid=%s swivel=%.4f elbow_status=%s elbow_applied=%s elbow_scale=%.2f",
      side_name,
      qiling_kinematics::toString(control.state()),
      diagnostics.command_held ? "true" : "false",
      qiling_kinematics::toString(diagnostics.solver_status),
      diagnostics.target_age_sec,
      diagnostics.joint_state_age_sec,
      diagnostics.qdot.cwiseAbs().maxCoeff(),
      diagnostics.qdot_measured.cwiseAbs().maxCoeff(),
      diagnostics.joint_limit_prediction_active ? "true" : "false",
      diagnostics.min_predicted_limit_distance_rad,
      diagnostics.joint_limit_damper_active ? "true" : "false",
      diagnostics.min_hard_limit_distance_rad,
      diagnostics.position_sigma_min,
      diagnostics.wrist_sigma_min,
      diagnostics.position_speed_scale,
      diagnostics.orientation_speed_scale,
      diagnostics.elbow_geometry_valid ? "true" : "false",
      diagnostics.elbow_swivel_error_rad,
      qiling_kinematics::toString(diagnostics.elbow_status),
      diagnostics.elbow_applied ? "true" : "false",
      diagnostics.elbow_scale);
  }

  void controlTick()
  {
    const auto now_steady = Clock::now();
    const double raw_dt =
      std::chrono::duration<double>(now_steady - last_control_tick_).count();
    last_control_tick_ = now_steady;

    const double nominal_dt =
      1.0 / std::max(get_parameter("control_rate_hz").as_double(), 1.0);
    const double min_dt = std::max(
      get_parameter("min_control_dt_sec").as_double(), 1.0e-6);
    const double max_dt = std::max(
      get_parameter("max_control_dt_sec").as_double(), min_dt);
    const double dt = std::clamp(
      std::isfinite(raw_dt) && raw_dt > 0.0 ? raw_dt : nominal_dt,
      min_dt, max_dt);
    const bool control_stalled =
      !std::isfinite(raw_dt) ||
      raw_dt > get_parameter("control_stall_fault_sec").as_double();
    updateGravityBlend(dt);

    std::array<ArmVector, kSideCount> q_snapshot;
    std::array<double, kLegMotorCount> leg_q_snapshot;
    std::array<bool, kSideCount> state_received{};
    std::array<Clock::time_point, kSideCount> state_times{};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      q_snapshot = q_state_;
      leg_q_snapshot = leg_q_state_;
      state_received = arm_state_received_;
      state_times = last_arm_state_time_;
    }

    std::array<TimedTarget, kSideCount> targets;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      targets = targets_;
    }

    std::array<uint8_t, kSideCount> modes{};
    {
      std::lock_guard<std::mutex> lock(mode_mutex_);
      modes = requested_modes_;
    }

    if (!state_received[kLeftSide] && !state_received[kRightSide]) {
      return;
    }

    const auto arm_kinematics = kinematics_->evaluate(
      q_snapshot[kLeftSide], q_snapshot[kRightSide]);
    gravity_torques_ = kinematics_->gravityTorques(
      q_snapshot[kLeftSide], q_snapshot[kRightSide]);

    const std::array<double, kSideCount> state_age = {
      ageSeconds(now_steady, state_times[kLeftSide], state_received[kLeftSide]),
      ageSeconds(now_steady, state_times[kRightSide], state_received[kRightSide])};
    const std::array<double, kSideCount> target_age = {
      ageSeconds(now_steady, targets[kLeftSide].received_at, targets[kLeftSide].received),
      ageSeconds(now_steady, targets[kRightSide].received_at, targets[kRightSide].received)};

    const double state_timeout = get_parameter("joint_state_timeout_sec").as_double();
    const bool left_state_fresh =
      state_received[kLeftSide] && state_age[kLeftSide] <= state_timeout;
    const bool right_state_fresh =
      state_received[kRightSide] && state_age[kRightSide] <= state_timeout;
    updateMeasuredVelocity(
      kLeftSide, q_snapshot[kLeftSide], left_state_fresh, dt, control_stalled);
    updateMeasuredVelocity(
      kRightSide, q_snapshot[kRightSide], right_state_fresh, dt, control_stalled);
    publishCurrentState(arm_kinematics, left_state_fresh, right_state_fresh);

    // Home/re-home owns the arm command path until both measured arms have
    // reached the configured home pose. XR targets and Grip modes are ignored
    // during this phase so a held controller cannot create a pose jump.
    if (!home_complete_) {
      publishHomeStatus();
      if (!left_state_fresh || !right_state_fresh) {
        if (home_started_) {
          if (home_phase_ != StartupHomePhase::Fault) {
            setHomePhase(
              StartupHomePhase::Fault, "joint state became stale during home motion");
          }
          home_command_q_ = q_snapshot;
          for (auto & velocity : home_command_qdot_) {
            velocity.setZero();
          }
          publishCommand(
            leg_q_snapshot, q_snapshot, home_command_q_, home_command_qdot_);
        }
        return;
      }
      if (!home_started_) {
        beginStartupHome(q_snapshot);
      }
      updateStartupHome(q_snapshot, dt);
      publishCommand(
        leg_q_snapshot, q_snapshot, home_command_q_, home_command_qdot_);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), get_parameter("home_log_period_ms").as_int(),
        "home phase=%s error=%.4f rad", toString(home_phase_),
        maxJointError(q_snapshot, home_phase_ == StartupHomePhase::MoveToTransition ||
          home_phase_ == StartupHomePhase::SettleAtTransition ?
          home_transition_q_ : home_q_));
      return;
    }

    publishHomeStatus();

    // A Grip held while home was running must not cause a target jump. Each
    // side becomes eligible only after its own Grip has been released once.
    for (int side = 0; side < kSideCount; ++side) {
      if (modes[side] == 0) {
        teleop_side_ready_[side] = true;
      }
    }

    processArm(
      kLeftSide, "left", arm_kinematics[kLeftSide], q_snapshot[kLeftSide],
      targets[kLeftSide], teleop_side_ready_[kLeftSide] && modes[kLeftSide] != 0,
      state_received[kLeftSide], state_age[kLeftSide], target_age[kLeftSide],
      control_stalled, dt, left_control_, *left_solver_, left_diagnostics_);
    processArm(
      kRightSide, "right", arm_kinematics[kRightSide], q_snapshot[kRightSide],
      targets[kRightSide], teleop_side_ready_[kRightSide] && modes[kRightSide] != 0,
      state_received[kRightSide], state_age[kRightSide], target_age[kRightSide],
      control_stalled, dt, right_control_, *right_solver_, right_diagnostics_);

    if (left_control_.initialized() && right_control_.initialized()) {
      publishCommand(leg_q_snapshot, q_snapshot);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for one complete 26-motor /human_lower_state before first command");
    }
  }

  void publishCommand(
    const std::array<double, kLegMotorCount> & leg_positions,
    const std::array<ArmVector, kSideCount> & measured)
  {
    const std::array<ArmVector, kSideCount> references = {
      left_control_.reference(), right_control_.reference()};
    const std::array<ArmVector, kSideCount> velocities = {
      ArmVector::Zero(), ArmVector::Zero()};
    publishCommand(leg_positions, measured, references, velocities);
  }

  void publishCommand(
    const std::array<double, kLegMotorCount> & leg_positions,
    const std::array<ArmVector, kSideCount> & measured,
    const std::array<ArmVector, kSideCount> & references,
    const std::array<ArmVector, kSideCount> & velocities)
  {
    mit_msgs::msg::MITJointCommands command;
    command.commands.resize(kCommandSize);
    command.stamp = now();

    const float kp = static_cast<float>(get_parameter("command_kp").as_double());
    const float kd = static_cast<float>(get_parameter("command_kd").as_double());
    for (auto & joint : command.commands) {
      joint.kp = 0.0F;
      joint.kd = 0.0F;
      joint.pos = 0.0F;
      joint.vel = 0.0F;
      joint.eff = 0.0F;
    }

    // Keep the legs at their measured positions. The arm-only controller must
    // not turn the unused 0..11 command slots into a request for joint zero.
    for (int i = 0; i < kLegMotorCount; ++i) {
      command.commands[i].pos = static_cast<float>(leg_positions[i]);
    }

    const std::array<ArmVector, kSideCount> control_references = {
      left_control_.reference(), right_control_.reference()};
    std::array<ArmVector, kSideCount> efforts;
    for (auto & effort : efforts) {
      effort.setZero();
    }

    for (int i = 0; i < kSingleArmDof; ++i) {
      command.commands[kLeftArmCommandOffset + i].kp = kp;
      command.commands[kLeftArmCommandOffset + i].kd = kd;
      command.commands[kLeftArmCommandOffset + i].pos =
        static_cast<float>(references[kLeftSide][i]);
      command.commands[kLeftArmCommandOffset + i].vel =
        static_cast<float>(velocities[kLeftSide][i]);
      efforts[kLeftSide][i] = gravityFeedforward(kLeftSide, i);
      command.commands[kLeftArmCommandOffset + i].eff = static_cast<float>(
        efforts[kLeftSide][i]);

      command.commands[kRightArmCommandOffset + i].kp = kp;
      command.commands[kRightArmCommandOffset + i].kd = kd;
      command.commands[kRightArmCommandOffset + i].pos =
        static_cast<float>(references[kRightSide][i]);
      command.commands[kRightArmCommandOffset + i].vel =
        static_cast<float>(velocities[kRightSide][i]);
      efforts[kRightSide][i] = gravityFeedforward(kRightSide, i);
      command.commands[kRightArmCommandOffset + i].eff = static_cast<float>(
        efforts[kRightSide][i]);
    }

    logCommandBoundary(measured, control_references, references, velocities, efforts);
    command_pub_->publish(command);
    last_published_positions_ = references;
    last_published_velocities_ = velocities;
    last_published_efforts_ = efforts;
    last_published_command_initialized_ = true;
  }

  void logCommandBoundary(
    const std::array<ArmVector, kSideCount> & measured,
    const std::array<ArmVector, kSideCount> & control_references,
    const std::array<ArmVector, kSideCount> & positions,
    const std::array<ArmVector, kSideCount> & velocities,
    const std::array<ArmVector, kSideCount> & efforts)
  {
    const std::array<ArmRunState, kSideCount> states = {
      left_control_.state(), right_control_.state()};
    const bool phase_changed = !command_log_initialized_ ||
      home_phase_ != last_command_phase_;
    const bool state_changed = !command_log_initialized_ || states != last_command_states_;
    const bool blend_changed = !command_log_initialized_ ||
      gravity_blend_active_ != last_command_gravity_blend_active_;
    if (phase_changed || state_changed || blend_changed) {
      const double kp = get_parameter("command_kp").as_double();
      const double kd = get_parameter("command_kd").as_double();
      for (int side = 0; side < kSideCount; ++side) {
        ArmVector delta_position = ArmVector::Zero();
        ArmVector delta_velocity = ArmVector::Zero();
        ArmVector delta_effort = ArmVector::Zero();
        if (last_published_command_initialized_) {
          delta_position = positions[side] - last_published_positions_[side];
          delta_velocity = velocities[side] - last_published_velocities_[side];
          delta_effort = efforts[side] - last_published_efforts_[side];
        }
        const ArmVector delta_command_torque =
          kp * delta_position + kd * delta_velocity + delta_effort;
        RCLCPP_INFO(
          get_logger(),
          "command boundary side=%s home_phase=%s home_complete=%s control_state=%s "
          "gravity_blend=%.3f blend_active=%s previous=%s q_measured=%s q_ref=%s "
          "pos=%s vel=%s eff=%s dpos=%s dvel=%s deff=%s dtau_cmd=%s",
          side == kLeftSide ? "left" : "right", toString(home_phase_),
          home_complete_ ? "true" : "false", qiling_kinematics::toString(states[side]),
          gravity_blend_scale_, gravity_blend_active_ ? "true" : "false",
          last_published_command_initialized_ ? "true" : "false",
          formatArmVector(measured[side]).c_str(),
          formatArmVector(control_references[side]).c_str(),
          formatArmVector(positions[side]).c_str(),
          formatArmVector(velocities[side]).c_str(),
          formatArmVector(efforts[side]).c_str(),
          formatArmVector(delta_position).c_str(),
          formatArmVector(delta_velocity).c_str(),
          formatArmVector(delta_effort).c_str(),
          formatArmVector(delta_command_torque).c_str());
      }
    }
    command_log_initialized_ = true;
    last_command_phase_ = home_phase_;
    last_command_states_ = states;
    last_command_gravity_blend_active_ = gravity_blend_active_;
  }

  double gravityFeedforward(int side, int joint) const
  {
    const bool gravity_phase = home_phase_ == StartupHomePhase::Complete ||
      home_phase_ == StartupHomePhase::HoldForRecording ||
      home_phase_ == StartupHomePhase::MoveDirectToHome;
    if (!get_parameter("gravity_compensation_enabled").as_bool() || !gravity_phase ||
      gravity_blend_scale_ <= 0.0 || side < 0 || side >= kSideCount ||
      joint < 0 || joint >= kSingleArmDof)
    {
      return 0.0;
    }

    // Gravity feedforward is used during normal teleoperation and recording
    // hold. At B it is faded over the complete direct-home trajectory;
    // startup home, settling, the smooth handoff, and FAULT remain torque-free.
    const ArmControlState & control = side == kLeftSide ? left_control_ : right_control_;
    if (control.state() == ArmRunState::Fault) {
      return 0.0;
    }

    const double scale = get_parameter("gravity_torque_scale").as_double();
    const double limit_scale = get_parameter("gravity_torque_limit_scale").as_double();
    const double effort_limit = effort_limits_[side][joint];
    const double torque = gravity_blend_scale_ * scale * gravity_torques_[side][joint];
    if (!std::isfinite(scale) || !std::isfinite(limit_scale) ||
      scale < 0.0 || limit_scale < 0.0 || !std::isfinite(torque) ||
      !std::isfinite(effort_limit) || effort_limit <= 0.0)
    {
      return 0.0;
    }
    return std::clamp(torque, -limit_scale * effort_limit, limit_scale * effort_limit);
  }

  PoseMsg makePoseMessage(const pinocchio::SE3 & pose) const
  {
    PoseMsg message;
    message.header.stamp = now();
    message.header.frame_id = get_parameter("target_frame").as_string();

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

  void publishCurrentState(
    const std::array<ArmKinematics, kSideCount> & arm_kinematics,
    bool left_fresh, bool right_fresh)
  {
    if (left_fresh) {
      left_state_pub_->publish(makePoseMessage(arm_kinematics[kLeftSide].wrist_pose));
    }
    if (right_fresh) {
      right_state_pub_->publish(makePoseMessage(arm_kinematics[kRightSide].wrist_pose));
    }
  }

  std::unique_ptr<DualArmKinematics> kinematics_;
  std::array<ArmVector, kSideCount> q_state_{};
  std::array<double, kLegMotorCount> leg_q_state_{};
  std::array<ArmVector, kSideCount> q_min_{};
  std::array<ArmVector, kSideCount> q_max_{};
  std::array<ArmVector, kSideCount> home_q_{};
  std::array<ArmVector, kSideCount> home_transition_q_{};
  std::array<ArmVector, kSideCount> home_start_q_{};
  std::array<ArmVector, kSideCount> home_hold_start_q_{};
  std::array<ArmVector, kSideCount> home_hold_target_q_{};
  std::array<ArmVector, kSideCount> home_handoff_start_q_{};
  std::array<ArmVector, kSideCount> home_handoff_target_q_{};
  std::array<ArmVector, kSideCount> home_command_q_{};
  std::array<ArmVector, kSideCount> home_command_qdot_{};
  std::array<ArmVector, kSideCount> gravity_torques_{};
  std::array<ArmVector, kSideCount> effort_limits_{};
  std::array<ArmVector, kSideCount> last_published_positions_{};
  std::array<ArmVector, kSideCount> last_published_velocities_{};
  std::array<ArmVector, kSideCount> last_published_efforts_{};
  std::array<Eigen::Vector3d, kSideCount> home_elbow_direction_{};
  std::array<TimedTarget, kSideCount> filtered_targets_{};
  std::array<bool, kSideCount> filtered_target_valid_{};
  std::array<ArmVector, kSideCount> previous_measured_q_{};
  std::array<ArmVector, kSideCount> measured_velocity_{};
  std::array<bool, kSideCount> measured_velocity_initialized_{};
  std::array<WorkspaceGuardState, kSideCount> workspace_guard_{};
  std::unique_ptr<HierarchicalDIKSolver> left_solver_;
  std::unique_ptr<HierarchicalDIKSolver> right_solver_;
  ArmControlState left_control_;
  ArmControlState right_control_;
  std::array<AnthropomorphicElbow, kSideCount> elbow_geometry_{};
  AnthropomorphicElbowConfig elbow_config_{};
  IkDiagnostics left_diagnostics_;
  IkDiagnostics right_diagnostics_;

  std::array<TimedTarget, kSideCount> targets_{};
  std::array<uint8_t, kSideCount> requested_modes_{};
  std::array<bool, kSideCount> teleop_side_ready_{};
  std::array<bool, kSideCount> arm_state_received_{};
  std::array<Clock::time_point, kSideCount> last_arm_state_time_{};
  std::mutex state_mutex_;
  std::mutex target_mutex_;
  std::mutex mode_mutex_;

  StartupHomePhase home_phase_{StartupHomePhase::WaitingForState};
  bool home_started_{false};
  bool home_complete_{false};
  double home_phase_time_sec_{0.0};
  double home_settle_time_sec_{0.0};
  double gravity_blend_scale_{0.0};
  double gravity_blend_start_scale_{0.0};
  double gravity_blend_target_scale_{0.0};
  double gravity_blend_elapsed_sec_{0.0};
  double gravity_blend_duration_sec_{0.0};
  bool gravity_blend_active_{false};
  bool command_log_initialized_{false};
  bool last_published_command_initialized_{false};
  StartupHomePhase last_command_phase_{StartupHomePhase::WaitingForState};
  std::array<ArmRunState, kSideCount> last_command_states_{
    ArmRunState::Hold, ArmRunState::Hold};
  bool last_command_gravity_blend_active_{false};

  rclcpp::Subscription<mit_msgs::msg::MITLowState>::SharedPtr lower_state_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr left_target_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr right_target_sub_;
  rclcpp::Subscription<ModeMsg>::SharedPtr left_mode_sub_;
  rclcpp::Subscription<ModeMsg>::SharedPtr right_mode_sub_;
  rclcpp::Publisher<mit_msgs::msg::MITJointCommands>::SharedPtr command_pub_;
  rclcpp::Publisher<PoseMsg>::SharedPtr left_state_pub_;
  rclcpp::Publisher<PoseMsg>::SharedPtr right_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr home_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr home_state_pub_;
  rclcpp::Service<Trigger>::SharedPtr recording_hold_service_;
  rclcpp::Service<Trigger>::SharedPtr home_request_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  Clock::time_point last_control_tick_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DifferentialIkNode>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "qiling_differential_ik fatal: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
