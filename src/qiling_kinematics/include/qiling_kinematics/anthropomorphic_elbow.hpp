#pragma once

#include <array>

#include <Eigen/Core>

#include "qiling_kinematics/ik_types.hpp"

namespace qiling_kinematics
{

struct AnthropomorphicElbowConfig
{
  double outward_weight{1.0};
  double downward_weight{0.30};
  double home_weight{0.80};
  double min_shoulder_wrist_distance{0.05};
  double min_elbow_projection_radius{0.02};
};

struct AnthropomorphicElbowOutput
{
  bool valid{false};
  bool used_previous_direction{false};
  bool used_home_direction{false};
  bool current_direction_valid{false};
  Eigen::Vector3d shoulder_wrist_axis{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_direction{Eigen::Vector3d::Zero()};
  Eigen::Vector3d preferred_direction{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
  double shoulder_wrist_distance{0.0};
  double current_projection_radius{0.0};
  double signed_swivel_error{0.0};
};

/**
 * Local scalar Jacobian of the signed elbow swivel angle. The shoulder is
 * treated as fixed in base_link, while both the elbow and wrist linear
 * Jacobians contribute to the instantaneous shoulder-wrist frame motion.
 */
ArmScalarJacobian computeElbowSwivelJacobian(
  const Eigen::Vector3d & shoulder,
  const Eigen::Vector3d & elbow,
  const Eigen::Vector3d & wrist,
  const ArmLinearJacobian & elbow_jacobian,
  const ArmLinearJacobian & wrist_jacobian,
  const Eigen::Vector3d & shoulder_wrist_axis,
  const Eigen::Vector3d & current_direction,
  double current_projection_radius);

/**
 * Read-only anthropomorphic elbow target generator.
 *
 * It computes a stable preferred shoulder-wrist swivel direction in the
 * base_link axes. It deliberately does not produce qdot or modify an IK
 * result; Phase 5 initially uses it only for geometry and continuity tests.
 */
class AnthropomorphicElbow
{
public:
  void reset();

  AnthropomorphicElbowOutput compute(
    ArmSide side,
    const Eigen::Vector3d & shoulder,
    const Eigen::Vector3d & elbow,
    const Eigen::Vector3d & wrist,
    const Eigen::Vector3d & home_direction,
    const AnthropomorphicElbowConfig & config);

private:
  std::array<Eigen::Vector3d, 2> previous_direction_{};
  std::array<bool, 2> previous_valid_{};
};

}  // namespace qiling_kinematics
