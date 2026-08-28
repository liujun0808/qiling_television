#pragma once

#include <limits>

#include "qiling_kinematics/ik_types.hpp"

namespace qiling_kinematics
{

struct IkDiagnostics
{
  ArmRunState control_state{ArmRunState::Hold};
  SolverStatus solver_status{SolverStatus::InvalidInput};
  bool command_held{true};

  double target_age_sec{std::numeric_limits<double>::infinity()};
  double joint_state_age_sec{std::numeric_limits<double>::infinity()};
  double position_error_norm{0.0};
  double rotation_error_norm{0.0};
  double solve_time_us{0.0};
  bool joint_limit_damper_active{false};
  double min_hard_limit_distance_rad{0.0};
  int position_rank{0};
  double position_sigma_min{0.0};
  double position_condition_number{0.0};
  double position_speed_scale{1.0};
  int wrist_rank{0};
  double wrist_sigma_min{0.0};
  double wrist_condition_number{0.0};
  double orientation_speed_scale{1.0};
  bool elbow_geometry_valid{false};
  bool elbow_used_previous_direction{false};
  bool elbow_used_home_direction{false};
  double elbow_swivel_error_rad{0.0};
  double elbow_projection_radius_m{0.0};
  double elbow_shoulder_wrist_distance_m{0.0};
  Eigen::Vector3d elbow_preferred_direction{Eigen::Vector3d::Zero()};
  SolverStatus elbow_status{SolverStatus::InvalidInput};
  bool elbow_applied{false};
  double elbow_scale{0.0};
  double elbow_swivel_velocity_desired{0.0};
  double elbow_swivel_velocity_achieved{0.0};
  double elbow_position_degradation_mps{0.0};
  double elbow_orientation_degradation_rps{0.0};

  ArmVector qdot{ArmVector::Zero()};
};

}  // namespace qiling_kinematics
