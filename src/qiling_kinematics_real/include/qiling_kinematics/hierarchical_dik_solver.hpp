#pragma once

#include <memory>

#include "qiling_kinematics/ik_types.hpp"

namespace qiling_kinematics
{

// The solver is being migrated one strict hierarchy level at a time. The
// Phase 1 legacy method remains only for regression tests; runtime uses the
// Phase 4 pose-hierarchy method.
class HierarchicalDIKSolver
{
public:
  struct Config
  {
    double position_gain{3.0};
    double rotation_gain{3.0};
    double position_weight{1.0};
    double rotation_weight{0.8};
    double damping{0.02};
    double posture_weight{0.0};
    double joint_limit_margin{0.08};
    double max_joint_velocity{1.5};
    double max_position_error{0.25};
    double max_rotation_error{1.2};
    double max_linear_velocity{0.20};
    // Library defaults stay neutral; teleoperation enables the protective
    // deadbands explicitly in differential_ik.yaml.
    double position_error_deadband{0.0};
    double position_sigma_slowdown_start{0.05};
    double position_sigma_stop{0.0};
    double position_singularity_speed_scale_min{0.0};
    double position_regularization{1.0e-4};
    double position_smoothness_weight{1.0e-3};
    double max_angular_velocity{0.80};
    double orientation_error_deadband{0.0};
    double wrist_sigma_slowdown_start{0.08};
    double wrist_sigma_stop{0.0};
    double orientation_singularity_speed_scale_min{0.0};
    double orientation_regularization{1.0e-4};
    double orientation_smoothness_weight{1.0e-3};
    double rank_threshold{1.0e-4};
    double max_orientation_position_degradation{5.0e-4};
    // Phase 5.2: the elbow is a one-dimensional tertiary task in the null
    // space of the complete 6D wrist task.
    double elbow_swivel_gain{2.0};
    double max_elbow_swivel_velocity{0.80};
    double elbow_posture_weight{0.08};
    double elbow_joint_centering_weight{0.03};
    double elbow_smoothness_weight{0.05};
    double elbow_regularization{1.0e-4};
    double elbow_sigma_fade_start{0.08};
    double elbow_sigma_disable{0.04};
    double max_elbow_position_degradation{5.0e-4};
    double max_elbow_orientation_degradation{2.0e-3};
    double characteristic_length{0.25};
    double joint_limit_damper_gain{1.0};
    double hard_limit_tolerance{0.005};
    // Predictive joint-limit protection. The measured velocity is used only
    // to reserve the distance required to brake before the URDF hard limit.
    bool joint_limit_prediction_enabled{false};
    double joint_limit_prediction_delay_sec{0.0};
    double joint_limit_prediction_margin_rad{0.0};
    ArmVector max_joint_velocity_rps{ArmVector::Constant(1.5)};
    // Keep the library default backwards-compatible; the runtime YAML sets
    // the finite safety values used by teleoperation.
    ArmVector max_joint_acceleration_rps2{ArmVector::Constant(1.0e6)};
    int qp_max_iter{80};
    double qp_eps_abs{1.0e-5};
    double qp_eps_rel{1.0e-5};
  };

  struct Input
  {
    ArmJacobian jacobian{ArmJacobian::Zero()};
    CartesianVector pose_error{CartesianVector::Zero()};
    ArmVector q_measured{ArmVector::Zero()};
    ArmVector q_reference{ArmVector::Zero()};
    ArmVector q_nominal{ArmVector::Zero()};
    ArmVector q_min{ArmVector::Zero()};
    ArmVector q_max{ArmVector::Zero()};
    double dt{0.02};
  };

  struct PositionInput
  {
    ArmLinearJacobian jacobian{ArmLinearJacobian::Zero()};
    Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
    ArmVector q_measured{ArmVector::Zero()};
    ArmVector q_min{ArmVector::Zero()};
    ArmVector q_max{ArmVector::Zero()};
    ArmVector qdot_previous{ArmVector::Zero()};
    ArmVector qdot_measured{ArmVector::Zero()};
    double dt{0.02};
  };

  struct PoseHierarchyInput
  {
    PositionInput position;
    ArmLinearJacobian angular_jacobian{ArmLinearJacobian::Zero()};
    Eigen::Vector3d orientation_error{Eigen::Vector3d::Zero()};
    bool elbow_geometry_valid{false};
    ArmScalarJacobian elbow_swivel_jacobian{ArmScalarJacobian::Zero()};
    double elbow_swivel_error{0.0};
    ArmVector q_rest{ArmVector::Zero()};
  };

  struct Result
  {
    bool success{false};
    SolverStatus status{SolverStatus::InvalidInput};
    ArmVector qdot{ArmVector::Zero()};
    ArmVector qdot_pose{ArmVector::Zero()};
    double position_error_norm{0.0};
    double rotation_error_norm{0.0};
    double solve_time_us{0.0};
    int proxqp_status{-1};
    SolverStatus orientation_status{SolverStatus::InvalidInput};
    bool orientation_applied{false};
    int position_rank{0};
    ArmVector qdot_position{ArmVector::Zero()};
    ArmVector qdot_lower{ArmVector::Zero()};
    ArmVector qdot_upper{ArmVector::Zero()};
    Eigen::Vector3d desired_linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d achieved_linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d achieved_angular_velocity{Eigen::Vector3d::Zero()};
    double position_degradation_mps{0.0};
    double orientation_solve_time_us{0.0};
    int orientation_proxqp_status{-1};
    bool joint_limit_damper_active{false};
    double min_hard_limit_distance_rad{0.0};
    bool joint_limit_prediction_active{false};
    double min_predicted_limit_distance_rad{0.0};
    double position_sigma_min{0.0};
    double position_sigma_max{0.0};
    double position_condition_number{0.0};
    int wrist_rank{0};
    double wrist_sigma_min{0.0};
    double wrist_sigma_max{0.0};
    double wrist_condition_number{0.0};
    double position_speed_scale{1.0};
    double orientation_speed_scale{1.0};
    SolverStatus elbow_status{SolverStatus::InvalidInput};
    bool elbow_applied{false};
    double elbow_scale{0.0};
    double elbow_swivel_velocity_desired{0.0};
    double elbow_swivel_velocity_achieved{0.0};
    double elbow_position_degradation_mps{0.0};
    double elbow_orientation_degradation_rps{0.0};
    double elbow_solve_time_us{0.0};
    int elbow_proxqp_status{-1};
  };

  explicit HierarchicalDIKSolver(const Config & config);
  ~HierarchicalDIKSolver();

  HierarchicalDIKSolver(const HierarchicalDIKSolver &) = delete;
  HierarchicalDIKSolver & operator=(const HierarchicalDIKSolver &) = delete;
  HierarchicalDIKSolver(HierarchicalDIKSolver &&) noexcept;
  HierarchicalDIKSolver & operator=(HierarchicalDIKSolver &&) noexcept;

  Result solveLegacyFullPose(const Input & input);
  Result solvePositionPrimary(const PositionInput & input);
  Result solvePoseHierarchy(const PoseHierarchyInput & input);
  const Config & config() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace qiling_kinematics
