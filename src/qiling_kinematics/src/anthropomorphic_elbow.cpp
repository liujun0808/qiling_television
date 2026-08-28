#include "qiling_kinematics/anthropomorphic_elbow.hpp"

#include <cmath>

#include <Eigen/Geometry>

namespace qiling_kinematics
{
namespace
{

constexpr double kEpsilon = 1.0e-12;

bool normalizeIfLarge(Eigen::Vector3d & value, double minimum_norm)
{
  const double norm = value.norm();
  if (!value.allFinite() || !std::isfinite(norm) || norm <= minimum_norm) {
    return false;
  }
  value /= norm;
  return value.allFinite();
}

int sideIndex(ArmSide side)
{
  return side == ArmSide::Left ? 0 : 1;
}

Eigen::Vector3d outwardDirection(ArmSide side)
{
  return Eigen::Vector3d(0.0, side == ArmSide::Left ? 1.0 : -1.0, 0.0);
}

}  // namespace

ArmScalarJacobian computeElbowSwivelJacobian(
  const Eigen::Vector3d & shoulder,
  const Eigen::Vector3d & elbow,
  const Eigen::Vector3d & wrist,
  const ArmLinearJacobian & elbow_jacobian,
  const ArmLinearJacobian & wrist_jacobian,
  const Eigen::Vector3d & shoulder_wrist_axis,
  const Eigen::Vector3d & current_direction,
  double current_projection_radius)
{
  ArmScalarJacobian result = ArmScalarJacobian::Zero();
  if (!shoulder.allFinite() || !elbow.allFinite() || !wrist.allFinite() ||
    !elbow_jacobian.allFinite() || !wrist_jacobian.allFinite() ||
    !shoulder_wrist_axis.allFinite() || !current_direction.allFinite() ||
    !std::isfinite(current_projection_radius) ||
    current_projection_radius <= kEpsilon)
  {
    return result;
  }

  const Eigen::Vector3d shoulder_to_wrist = wrist - shoulder;
  const double shoulder_wrist_distance = shoulder_to_wrist.norm();
  const double axis_norm = shoulder_wrist_axis.norm();
  const double direction_norm = current_direction.norm();
  if (!std::isfinite(shoulder_wrist_distance) ||
    shoulder_wrist_distance <= kEpsilon ||
    !std::isfinite(axis_norm) || axis_norm <= kEpsilon ||
    !std::isfinite(direction_norm) || direction_norm <= kEpsilon)
  {
    return result;
  }

  const Eigen::Vector3d axis = shoulder_wrist_axis / axis_norm;
  const Eigen::Vector3d direction = current_direction / direction_norm;
  const Eigen::Matrix3d axis_projection =
    Eigen::Matrix3d::Identity() - axis * axis.transpose();
  const Eigen::Matrix3d direction_projection =
    Eigen::Matrix3d::Identity() - direction * direction.transpose();
  const Eigen::Vector3d shoulder_to_elbow = elbow - shoulder;
  const double elbow_axis_component = axis.dot(shoulder_to_elbow);

  // phi = atan2(axis dot (u x u_preferred), u dot u_preferred).
  // For this local Jacobian the preferred direction is held fixed during the
  // current control tick. Its dependence on the updated wrist frame is
  // accounted for on the next tick by AnthropomorphicElbow::compute().
  for (int column = 0; column < kSingleArmDof; ++column) {
    const Eigen::Vector3d wrist_velocity = wrist_jacobian.col(column);
    const Eigen::Vector3d elbow_velocity = elbow_jacobian.col(column);
    const Eigen::Vector3d axis_velocity =
      axis_projection * wrist_velocity / shoulder_wrist_distance;
    const Eigen::Vector3d radial_velocity =
      elbow_velocity - axis_velocity * elbow_axis_component -
      axis * axis_velocity.dot(shoulder_to_elbow);
    const Eigen::Vector3d direction_velocity =
      direction_projection * radial_velocity / current_projection_radius;
    result[column] = axis.dot(direction.cross(direction_velocity));
  }
  if (!result.allFinite()) {
    result.setZero();
  }
  return result;
}

void AnthropomorphicElbow::reset()
{
  previous_direction_ = {};
  previous_valid_ = {};
}

AnthropomorphicElbowOutput AnthropomorphicElbow::compute(
  ArmSide side,
  const Eigen::Vector3d & shoulder,
  const Eigen::Vector3d & elbow,
  const Eigen::Vector3d & wrist,
  const Eigen::Vector3d & home_direction,
  const AnthropomorphicElbowConfig & config)
{
  AnthropomorphicElbowOutput output;
  if (!shoulder.allFinite() || !elbow.allFinite() || !wrist.allFinite() ||
    !home_direction.allFinite() ||
    !std::isfinite(config.outward_weight) ||
    !std::isfinite(config.downward_weight) ||
    !std::isfinite(config.home_weight) ||
    !std::isfinite(config.min_shoulder_wrist_distance) ||
    !std::isfinite(config.min_elbow_projection_radius) ||
    config.min_shoulder_wrist_distance <= 0.0 ||
    config.min_elbow_projection_radius <= 0.0)
  {
    return output;
  }

  const int index = sideIndex(side);
  const Eigen::Vector3d shoulder_to_wrist = wrist - shoulder;
  output.shoulder_wrist_distance = shoulder_to_wrist.norm();
  if (!std::isfinite(output.shoulder_wrist_distance) ||
    output.shoulder_wrist_distance <= config.min_shoulder_wrist_distance)
  {
    return output;
  }

  output.shoulder_wrist_axis = shoulder_to_wrist / output.shoulder_wrist_distance;
  const Eigen::Matrix3d projection =
    Eigen::Matrix3d::Identity() -
    output.shoulder_wrist_axis * output.shoulder_wrist_axis.transpose();
  const Eigen::Vector3d shoulder_to_elbow = elbow - shoulder;
  Eigen::Vector3d current_projected = projection * shoulder_to_elbow;
  output.current_projection_radius = current_projected.norm();
  output.current_direction = current_projected;
  output.current_direction_valid = normalizeIfLarge(
    output.current_direction, config.min_elbow_projection_radius);

  Eigen::Vector3d preferred =
    config.outward_weight * outwardDirection(side) +
    config.downward_weight * Eigen::Vector3d(0.0, 0.0, -1.0) +
    config.home_weight * home_direction;
  preferred = projection * preferred;
  bool preferred_valid = normalizeIfLarge(preferred, kEpsilon);

  if (!preferred_valid && previous_valid_[index]) {
    preferred = projection * previous_direction_[index];
    preferred_valid = normalizeIfLarge(preferred, kEpsilon);
    output.used_previous_direction = preferred_valid;
  }
  if (!preferred_valid) {
    preferred = projection * home_direction;
    preferred_valid = normalizeIfLarge(preferred, kEpsilon);
    output.used_home_direction = preferred_valid;
  }
  if (!preferred_valid) {
    return output;
  }

  // A zero-radius elbow has no defined swivel angle. Keep the preferred
  // direction available for diagnostics, but do not expose a usable elbow
  // target to the future tertiary controller.
  if (!output.current_direction_valid) {
    output.preferred_direction = preferred;
    return output;
  }

  // Keep the selected branch continuous when a projected vector changes sign
  // numerically near a degenerate shoulder-wrist plane.
  if (previous_valid_[index] && preferred.dot(previous_direction_[index]) < 0.0) {
    preferred = -preferred;
  }
  output.preferred_direction = preferred;
  output.target_position =
    shoulder + output.shoulder_wrist_distance * output.shoulder_wrist_axis +
    output.current_projection_radius * output.preferred_direction;
  output.valid = output.target_position.allFinite();

  previous_direction_[index] = output.preferred_direction;
  previous_valid_[index] = output.valid;

  if (output.valid && output.current_direction_valid) {
    output.signed_swivel_error = std::atan2(
      output.shoulder_wrist_axis.dot(
        output.current_direction.cross(output.preferred_direction)),
      output.current_direction.dot(output.preferred_direction));
  }
  return output;
}

}  // namespace qiling_kinematics
