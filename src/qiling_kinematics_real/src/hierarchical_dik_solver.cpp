#include "qiling_kinematics/hierarchical_dik_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/SVD>

#include <proxsuite/proxqp/dense/dense.hpp>

namespace qiling_kinematics
{
namespace
{

using Qp = proxsuite::proxqp::dense::QP<double>;
constexpr int kPositionNullity = kSingleArmDof - 3;
using PositionNullBasis = Eigen::Matrix<double, kSingleArmDof, kPositionNullity>;
using ReducedOrientationJacobian = Eigen::Matrix<double, 3, kPositionNullity>;
using WristNullBasis = Eigen::Matrix<double, kSingleArmDof, 1>;

Eigen::Vector3d clampVectorNorm(const Eigen::Vector3d & value, double limit)
{
  if (!value.allFinite() || !std::isfinite(limit) || limit < 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const double norm = value.norm();
  if (norm > limit && norm > 1.0e-12) {
    return value * (limit / norm);
  }
  return value;
}

Eigen::Vector3d applyDeadband(const Eigen::Vector3d & value, double deadband)
{
  if (!value.allFinite() || !std::isfinite(deadband) || deadband < 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const double norm = value.norm();
  if (norm <= deadband || norm <= 1.0e-12) {
    return Eigen::Vector3d::Zero();
  }
  return value * ((norm - deadband) / norm);
}

double singularitySpeedScale(
  double sigma, double slowdown_start, double stop, double minimum_scale)
{
  if (!std::isfinite(sigma) || !std::isfinite(slowdown_start) ||
    !std::isfinite(stop) || !std::isfinite(minimum_scale) ||
    slowdown_start <= stop || stop < 0.0 || minimum_scale < 0.0 ||
    minimum_scale > 1.0)
  {
    return 0.0;
  }
  if (sigma <= stop) {
    return minimum_scale;
  }
  if (sigma >= slowdown_start) {
    return 1.0;
  }
  const double alpha = (sigma - stop) / (slowdown_start - stop);
  return minimum_scale + alpha * (1.0 - minimum_scale);
}

bool finiteInput(const HierarchicalDIKSolver::Input & input)
{
  return input.jacobian.allFinite() &&
         input.pose_error.allFinite() &&
         input.q_measured.allFinite() &&
         input.q_reference.allFinite() &&
         input.q_nominal.allFinite() &&
         input.q_min.allFinite() &&
         input.q_max.allFinite() &&
         std::isfinite(input.dt) && input.dt > 0.0;
}

bool finitePositionInput(const HierarchicalDIKSolver::PositionInput & input)
{
  return input.jacobian.allFinite() &&
         input.position_error.allFinite() &&
         input.q_measured.allFinite() &&
         input.q_min.allFinite() &&
         input.q_max.allFinite() &&
         input.qdot_previous.allFinite() &&
         input.qdot_measured.allFinite() &&
         std::isfinite(input.dt) && input.dt > 0.0;
}

}  // namespace

struct HierarchicalDIKSolver::Impl
{
  explicit Impl(const Config & solver_config)
  : config(solver_config),
    qp(
      kSingleArmDof, 0, 0, true,
      proxsuite::proxqp::HessianType::Dense,
      proxsuite::proxqp::DenseBackend::PrimalDualLDLT),
    orientation_qp(
      kPositionNullity, 0, kSingleArmDof, false,
      proxsuite::proxqp::HessianType::Dense,
      proxsuite::proxqp::DenseBackend::PrimalDualLDLT),
    elbow_qp(
      1, 0, kSingleArmDof, false,
      proxsuite::proxqp::HessianType::Dense,
      proxsuite::proxqp::DenseBackend::PrimalDualLDLT)
  {
    qp.settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    qp.settings.max_iter = config.qp_max_iter;
    qp.settings.eps_abs = config.qp_eps_abs;
    qp.settings.eps_rel = config.qp_eps_rel;
    qp.settings.verbose = false;

    qp.model.H.setIdentity();
    qp.model.g.setZero();
    qp.model.A.resize(0, kSingleArmDof);
    qp.model.b.resize(0);
    qp.model.l_box.setConstant(-config.max_joint_velocity);
    qp.model.u_box.setConstant(config.max_joint_velocity);
    qp.init(
      qp.model.H, qp.model.g, qp.model.A, qp.model.b,
      proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,
      qp.model.l_box, qp.model.u_box, false);

    orientation_qp.settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
    orientation_qp.settings.max_iter = config.qp_max_iter;
    orientation_qp.settings.eps_abs = config.qp_eps_abs;
    orientation_qp.settings.eps_rel = config.qp_eps_rel;
    orientation_qp.settings.verbose = false;
    orientation_qp.model.H.setIdentity();
    orientation_qp.model.g.setZero();
    orientation_qp.model.A.resize(0, kPositionNullity);
    orientation_qp.model.b.resize(0);
    orientation_qp.model.C.setZero();
    orientation_qp.model.C.topRows<kPositionNullity>().setIdentity();
    orientation_qp.model.l.setConstant(-config.max_joint_velocity);
    orientation_qp.model.u.setConstant(config.max_joint_velocity);
    orientation_qp.init(
      orientation_qp.model.H, orientation_qp.model.g,
      orientation_qp.model.A, orientation_qp.model.b,
      orientation_qp.model.C, orientation_qp.model.l, orientation_qp.model.u,
      false);

    elbow_qp.settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
    elbow_qp.settings.max_iter = config.qp_max_iter;
    elbow_qp.settings.eps_abs = config.qp_eps_abs;
    elbow_qp.settings.eps_rel = config.qp_eps_rel;
    elbow_qp.settings.verbose = false;
    elbow_qp.model.H.setIdentity();
    elbow_qp.model.g.setZero();
    elbow_qp.model.A.resize(0, 1);
    elbow_qp.model.b.resize(0);
    // ProxQP rejects an all-zero inequality matrix during initialization.
    // This placeholder is replaced with the actual wrist-null vector before
    // every solve.
    elbow_qp.model.C.setZero();
    elbow_qp.model.C(0, 0) = 1.0;
    elbow_qp.model.l.setConstant(-config.max_joint_velocity);
    elbow_qp.model.u.setConstant(config.max_joint_velocity);
    elbow_qp.init(
      elbow_qp.model.H, elbow_qp.model.g,
      elbow_qp.model.A, elbow_qp.model.b,
      elbow_qp.model.C, elbow_qp.model.l, elbow_qp.model.u,
      false);
  }

  Config config;
  Qp qp;
  Qp orientation_qp;
  Qp elbow_qp;
};

HierarchicalDIKSolver::HierarchicalDIKSolver(const Config & config)
: impl_(std::make_unique<Impl>(config))
{
}

HierarchicalDIKSolver::~HierarchicalDIKSolver() = default;
HierarchicalDIKSolver::HierarchicalDIKSolver(HierarchicalDIKSolver &&) noexcept = default;
HierarchicalDIKSolver & HierarchicalDIKSolver::operator=(
  HierarchicalDIKSolver &&) noexcept = default;

HierarchicalDIKSolver::Result HierarchicalDIKSolver::solveLegacyFullPose(
  const Input & input)
{
  Result result;
  result.position_error_norm = input.pose_error.head<3>().norm();
  result.rotation_error_norm = input.pose_error.tail<3>().norm();

  if (!finiteInput(input)) {
    return result;
  }

  const Config & config = impl_->config;
  if (!std::isfinite(config.position_gain) ||
    !std::isfinite(config.rotation_gain) ||
    !std::isfinite(config.position_weight) ||
    !std::isfinite(config.rotation_weight) ||
    !std::isfinite(config.damping) ||
    !std::isfinite(config.posture_weight) ||
    !std::isfinite(config.joint_limit_margin) ||
    !std::isfinite(config.max_joint_velocity) ||
    config.max_joint_velocity <= 0.0)
  {
    return result;
  }

  Eigen::Matrix<double, kCartesianDof, kCartesianDof> task_weight =
    Eigen::Matrix<double, kCartesianDof, kCartesianDof>::Zero();
  task_weight.diagonal().head<3>().setConstant(config.position_weight);
  task_weight.diagonal().tail<3>().setConstant(config.rotation_weight);

  CartesianVector desired_twist = CartesianVector::Zero();
  desired_twist.head<3>() =
    config.position_gain *
    clampVectorNorm(input.pose_error.head<3>(), config.max_position_error);
  desired_twist.tail<3>() =
    config.rotation_gain *
    clampVectorNorm(input.pose_error.tail<3>(), config.max_rotation_error);

  const ArmJacobian weighted_jacobian = task_weight * input.jacobian;
  const CartesianVector weighted_twist = task_weight * desired_twist;
  const ArmVector posture_velocity = input.q_nominal - input.q_reference;
  const Eigen::Matrix<double, kSingleArmDof, kSingleArmDof> identity =
    Eigen::Matrix<double, kSingleArmDof, kSingleArmDof>::Identity();

  impl_->qp.model.H =
    weighted_jacobian.transpose() * weighted_jacobian +
    (config.damping + config.posture_weight) * identity;
  impl_->qp.model.H =
    0.5 * (impl_->qp.model.H + impl_->qp.model.H.transpose());
  impl_->qp.model.g =
    -weighted_jacobian.transpose() * weighted_twist -
    config.posture_weight * posture_velocity;

  for (int i = 0; i < kSingleArmDof; ++i) {
    const double lower_q = input.q_min[i] + config.joint_limit_margin;
    const double upper_q = input.q_max[i] - config.joint_limit_margin;
    impl_->qp.model.l_box[i] = std::max(
      -config.max_joint_velocity,
      (lower_q - input.q_measured[i]) / input.dt);
    impl_->qp.model.u_box[i] = std::min(
      config.max_joint_velocity,
      (upper_q - input.q_measured[i]) / input.dt);
    if (!std::isfinite(impl_->qp.model.l_box[i]) ||
      !std::isfinite(impl_->qp.model.u_box[i]) ||
      impl_->qp.model.l_box[i] > impl_->qp.model.u_box[i])
    {
      result.status = SolverStatus::InvalidBounds;
      return result;
    }
  }

  const auto solve_start = std::chrono::steady_clock::now();
  impl_->qp.update(
    impl_->qp.model.H, impl_->qp.model.g,
    impl_->qp.model.A, impl_->qp.model.b,
    proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,
    impl_->qp.model.l_box, impl_->qp.model.u_box, false);
  impl_->qp.solve();
  const auto solve_end = std::chrono::steady_clock::now();
  result.solve_time_us =
    std::chrono::duration<double, std::micro>(solve_end - solve_start).count();

  const auto proxqp_status = impl_->qp.results.info.status;
  result.proxqp_status = static_cast<int>(proxqp_status);
  if (proxqp_status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED ||
    impl_->qp.results.x.size() != kSingleArmDof)
  {
    result.status = SolverStatus::QpFailure;
    return result;
  }

  result.qdot = impl_->qp.results.x;
  if (!result.qdot.allFinite()) {
    result.qdot.setZero();
    result.status = SolverStatus::QpFailure;
    return result;
  }
  // Numerical QP tolerances can return a value a few ulps outside a box
  // constraint. Enforce the safety envelope explicitly before the result can
  // reach reference integration or a robot command.
  for (int i = 0; i < kSingleArmDof; ++i) {
    result.qdot[i] = std::clamp(
      result.qdot[i], impl_->qp.model.l_box[i], impl_->qp.model.u_box[i]);
  }

  result.success = true;
  result.status = SolverStatus::Solved;
  return result;
}

HierarchicalDIKSolver::Result HierarchicalDIKSolver::solvePositionPrimary(
  const PositionInput & input)
{
  Result result;

  if (!finitePositionInput(input)) {
    return result;
  }
  result.position_error_norm = input.position_error.norm();

  const Config & config = impl_->config;
  if (!std::isfinite(config.position_gain) || config.position_gain < 0.0 ||
    !std::isfinite(config.max_linear_velocity) || config.max_linear_velocity <= 0.0 ||
    !std::isfinite(config.position_error_deadband) || config.position_error_deadband < 0.0 ||
    !std::isfinite(config.position_sigma_slowdown_start) ||
    !std::isfinite(config.position_sigma_stop) ||
    config.position_sigma_slowdown_start <= config.position_sigma_stop ||
    config.position_sigma_stop < 0.0 ||
    !std::isfinite(config.position_singularity_speed_scale_min) ||
    config.position_singularity_speed_scale_min < 0.0 ||
    config.position_singularity_speed_scale_min > 1.0 ||
    !std::isfinite(config.position_regularization) || config.position_regularization < 0.0 ||
    !std::isfinite(config.position_smoothness_weight) ||
    config.position_smoothness_weight < 0.0 ||
    !std::isfinite(config.joint_limit_margin) || config.joint_limit_margin < 0.0 ||
    !std::isfinite(config.joint_limit_damper_gain) ||
    config.joint_limit_damper_gain <= 0.0 ||
    !std::isfinite(config.hard_limit_tolerance) || config.hard_limit_tolerance < 0.0 ||
    !std::isfinite(config.joint_limit_prediction_delay_sec) ||
    config.joint_limit_prediction_delay_sec < 0.0 ||
    !std::isfinite(config.joint_limit_prediction_margin_rad) ||
    config.joint_limit_prediction_margin_rad < 0.0 ||
    !std::isfinite(config.rank_threshold) || config.rank_threshold <= 0.0 ||
    !config.max_joint_velocity_rps.allFinite() ||
    (config.max_joint_velocity_rps.array() <= 0.0).any() ||
    !config.max_joint_acceleration_rps2.allFinite() ||
    (config.max_joint_acceleration_rps2.array() <= 0.0).any())
  {
    return result;
  }

  const Eigen::JacobiSVD<ArmLinearJacobian> position_svd(input.jacobian);
  const auto & position_singular_values = position_svd.singularValues();
  result.position_sigma_max = position_singular_values[0];
  result.position_sigma_min = position_singular_values[2];
  result.position_condition_number = result.position_sigma_min > 0.0 ?
    result.position_sigma_max / result.position_sigma_min :
    std::numeric_limits<double>::infinity();
  for (const double singular_value : position_singular_values) {
    if (singular_value > config.rank_threshold) {
      ++result.position_rank;
    }
  }
  result.position_speed_scale = singularitySpeedScale(
    result.position_sigma_min, config.position_sigma_slowdown_start,
    config.position_sigma_stop, config.position_singularity_speed_scale_min);
  result.desired_linear_velocity = result.position_speed_scale * clampVectorNorm(
    config.position_gain * applyDeadband(
      input.position_error, config.position_error_deadband),
    config.max_linear_velocity);

  const Eigen::Matrix<double, kSingleArmDof, kSingleArmDof> identity =
    Eigen::Matrix<double, kSingleArmDof, kSingleArmDof>::Identity();
  impl_->qp.model.H = input.jacobian.transpose() * input.jacobian +
    (config.position_regularization + config.position_smoothness_weight) * identity;
  impl_->qp.model.H = 0.5 * (impl_->qp.model.H + impl_->qp.model.H.transpose());
  impl_->qp.model.g =
    -input.jacobian.transpose() * result.desired_linear_velocity -
    config.position_smoothness_weight * input.qdot_previous;

  for (int i = 0; i < kSingleArmDof; ++i) {
    const double q_min = input.q_min[i];
    const double q_max = input.q_max[i];
    const double q = input.q_measured[i];
    const double velocity_limit = config.max_joint_velocity_rps[i];
    if (q_min >= q_max) {
      result.status = SolverStatus::InvalidBounds;
      return result;
    }

    const double lower_distance = q - q_min;
    const double upper_distance = q_max - q;
    result.min_hard_limit_distance_rad = i == 0 ?
      std::min(lower_distance, upper_distance) :
      std::min(
        result.min_hard_limit_distance_rad,
        std::min(lower_distance, upper_distance));
    if (lower_distance < -config.hard_limit_tolerance ||
      upper_distance < -config.hard_limit_tolerance)
    {
      result.status = SolverStatus::InvalidBounds;
      return result;
    }

    // URDF limits are the only hard position limits. Their one-step bounds
    // remain feasible for every measured state inside the hard envelope.
    double lower = std::max(-velocity_limit, (q_min - q) / input.dt);
    double upper = std::min(velocity_limit, (q_max - q) / input.dt);

    // Reserve the distance needed to stop an outward-moving joint. This is a
    // velocity constraint only: the URDF interval above remains the physical
    // hard range, and a measured violation is still handled by the node's
    // dedicated recovery state before the solver is called.
    if (config.joint_limit_prediction_enabled) {
      const double joint_range = q_max - q_min;
      const double prediction_margin = std::min(
        config.joint_limit_prediction_margin_rad, 0.5 * joint_range);
      const double speed = std::abs(input.qdot_measured[i]);
      const double braking_acceleration = config.max_joint_acceleration_rps2[i];
      const double raw_stopping_distance =
        speed * config.joint_limit_prediction_delay_sec +
        0.5 * speed * speed / braking_acceleration;
      // Keep the predictive interval feasible even if a transient feedback
      // derivative is much larger than the configured command velocity.
      const double max_stopping_distance = std::max(
        0.0, joint_range - 2.0 * prediction_margin);
      const double stopping_distance = std::min(
        std::max(raw_stopping_distance, 0.0), max_stopping_distance);
      const double lower_guard = q_min + prediction_margin +
        (input.qdot_measured[i] < 0.0 ? stopping_distance : 0.0);
      const double upper_guard = q_max - prediction_margin -
        (input.qdot_measured[i] > 0.0 ? stopping_distance : 0.0);

      result.joint_limit_prediction_active = true;
      const double predicted_distance = std::min(q - lower_guard, upper_guard - q);
      result.min_predicted_limit_distance_rad = i == 0 ?
        predicted_distance : std::min(
          result.min_predicted_limit_distance_rad, predicted_distance);
      // If the measured state is already inside the stopping-distance guard,
      // the exact one-cycle target can require more than the configured
      // command speed. Saturate that inward request at the velocity limit so
      // the QP remains feasible and commands the strongest safe recovery.
      lower = std::max(
        lower, std::min(velocity_limit, (lower_guard - q) / input.dt));
      upper = std::min(
        upper, std::max(-velocity_limit, (upper_guard - q) / input.dt));
    }

    // The margin is a velocity-damper influence zone, not a second hard
    // position range. Outward velocity decreases continuously to zero at the
    // corresponding hard limit while inward recovery velocity remains free.
    if (config.joint_limit_margin > 0.0) {
      const double lower_scale = std::clamp(
        config.joint_limit_damper_gain * lower_distance / config.joint_limit_margin,
        0.0, 1.0);
      const double upper_scale = std::clamp(
        config.joint_limit_damper_gain * upper_distance / config.joint_limit_margin,
        0.0, 1.0);
      const double damped_lower = -velocity_limit * lower_scale;
      const double damped_upper = velocity_limit * upper_scale;
      lower = std::max(lower, damped_lower);
      upper = std::min(upper, damped_upper);
      result.joint_limit_damper_active =
        result.joint_limit_damper_active || lower_scale < 1.0 || upper_scale < 1.0;
    }

    // Bound the per-cycle velocity change as well as the velocity itself.
    // If a discontinuous measured-state jump makes the intersection empty,
    // retain the already-feasible position/velocity bounds for this tick.
    const double acceleration_limit = config.max_joint_acceleration_rps2[i] * input.dt;
    const double acceleration_lower = input.qdot_previous[i] - acceleration_limit;
    const double acceleration_upper = input.qdot_previous[i] + acceleration_limit;
    const double bounded_lower = std::max(lower, acceleration_lower);
    const double bounded_upper = std::min(upper, acceleration_upper);
    if (bounded_lower <= bounded_upper) {
      lower = bounded_lower;
      upper = bounded_upper;
    }

    impl_->qp.model.l_box[i] = lower;
    impl_->qp.model.u_box[i] = upper;
    if (!std::isfinite(impl_->qp.model.l_box[i]) ||
      !std::isfinite(impl_->qp.model.u_box[i]) ||
      impl_->qp.model.l_box[i] > impl_->qp.model.u_box[i])
    {
      result.status = SolverStatus::InvalidBounds;
      return result;
    }
    result.qdot_lower[i] = impl_->qp.model.l_box[i];
    result.qdot_upper[i] = impl_->qp.model.u_box[i];
  }

  const auto solve_start = std::chrono::steady_clock::now();
  impl_->qp.update(
    impl_->qp.model.H, impl_->qp.model.g,
    impl_->qp.model.A, impl_->qp.model.b,
    proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,
    impl_->qp.model.l_box, impl_->qp.model.u_box, false);
  impl_->qp.solve();
  const auto solve_end = std::chrono::steady_clock::now();
  result.solve_time_us =
    std::chrono::duration<double, std::micro>(solve_end - solve_start).count();

  const auto proxqp_status = impl_->qp.results.info.status;
  result.proxqp_status = static_cast<int>(proxqp_status);
  if (proxqp_status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED ||
    impl_->qp.results.x.size() != kSingleArmDof)
  {
    result.status = SolverStatus::QpFailure;
    return result;
  }

  result.qdot = impl_->qp.results.x;
  if (!result.qdot.allFinite()) {
    result.qdot.setZero();
    result.status = SolverStatus::QpFailure;
    return result;
  }
  for (int i = 0; i < kSingleArmDof; ++i) {
    result.qdot[i] = std::clamp(
      result.qdot[i], impl_->qp.model.l_box[i], impl_->qp.model.u_box[i]);
  }

  result.achieved_linear_velocity = input.jacobian * result.qdot;
  if (!result.achieved_linear_velocity.allFinite()) {
    result.qdot.setZero();
    result.achieved_linear_velocity.setZero();
    result.status = SolverStatus::QpFailure;
    return result;
  }

  result.success = true;
  result.status = SolverStatus::Solved;
  result.qdot_position = result.qdot;
  return result;
}

HierarchicalDIKSolver::Result HierarchicalDIKSolver::solvePoseHierarchy(
  const PoseHierarchyInput & input)
{
  Result result = solvePositionPrimary(input.position);
  if (!result.success) {
    return result;
  }
  result.qdot_pose = result.qdot;

  result.rotation_error_norm = input.orientation_error.allFinite() ?
    input.orientation_error.norm() : 0.0;
  if (!input.angular_jacobian.allFinite() || !input.orientation_error.allFinite()) {
    result.orientation_status = SolverStatus::InvalidInput;
    result.qdot_pose = result.qdot;
    return result;
  }

  const Config & config = impl_->config;
  if (!std::isfinite(config.rotation_gain) || config.rotation_gain < 0.0 ||
    !std::isfinite(config.max_angular_velocity) || config.max_angular_velocity <= 0.0 ||
    !std::isfinite(config.orientation_error_deadband) ||
    config.orientation_error_deadband < 0.0 ||
    !std::isfinite(config.wrist_sigma_slowdown_start) ||
    !std::isfinite(config.wrist_sigma_stop) ||
    config.wrist_sigma_slowdown_start <= config.wrist_sigma_stop ||
    config.wrist_sigma_stop < 0.0 ||
    !std::isfinite(config.orientation_singularity_speed_scale_min) ||
    config.orientation_singularity_speed_scale_min < 0.0 ||
    config.orientation_singularity_speed_scale_min > 1.0 ||
    !std::isfinite(config.orientation_regularization) ||
    config.orientation_regularization < 0.0 ||
    !std::isfinite(config.orientation_smoothness_weight) ||
    config.orientation_smoothness_weight < 0.0 ||
    !std::isfinite(config.characteristic_length) || config.characteristic_length <= 0.0 ||
    !std::isfinite(config.max_orientation_position_degradation) ||
    config.max_orientation_position_degradation < 0.0 ||
    !std::isfinite(config.elbow_swivel_gain) ||
    !std::isfinite(config.max_elbow_swivel_velocity) ||
    config.max_elbow_swivel_velocity <= 0.0 ||
    !std::isfinite(config.elbow_posture_weight) ||
    config.elbow_posture_weight < 0.0 ||
    !std::isfinite(config.elbow_joint_centering_weight) ||
    config.elbow_joint_centering_weight < 0.0 ||
    !std::isfinite(config.elbow_smoothness_weight) ||
    config.elbow_smoothness_weight < 0.0 ||
    !std::isfinite(config.elbow_regularization) ||
    config.elbow_regularization < 0.0 ||
    !std::isfinite(config.elbow_sigma_fade_start) ||
    !std::isfinite(config.elbow_sigma_disable) ||
    config.elbow_sigma_fade_start <= config.elbow_sigma_disable ||
    config.elbow_sigma_disable <= 0.0 ||
    !std::isfinite(config.max_elbow_position_degradation) ||
    config.max_elbow_position_degradation < 0.0 ||
    !std::isfinite(config.max_elbow_orientation_degradation) ||
    config.max_elbow_orientation_degradation < 0.0)
  {
    result.orientation_status = SolverStatus::InvalidInput;
    result.qdot_pose = result.qdot;
    return result;
  }

  const Eigen::JacobiSVD<ArmLinearJacobian> svd(
    input.position.jacobian, Eigen::ComputeFullV);
  if (result.position_rank != 3) {
    result.orientation_status = SolverStatus::RankDeficient;
    return result;
  }

  const PositionNullBasis null_basis = svd.matrixV().rightCols<kPositionNullity>();

  ArmJacobian scaled_wrist_jacobian;
  scaled_wrist_jacobian.topRows<3>() = input.position.jacobian;
  scaled_wrist_jacobian.bottomRows<3>() =
    config.characteristic_length * input.angular_jacobian;
  const Eigen::JacobiSVD<ArmJacobian> wrist_svd(
    scaled_wrist_jacobian, Eigen::ComputeFullV);
  const auto & wrist_singular_values = wrist_svd.singularValues();
  result.wrist_sigma_max = wrist_singular_values[0];
  result.wrist_sigma_min = wrist_singular_values[5];
  result.wrist_condition_number = result.wrist_sigma_min > 0.0 ?
    result.wrist_sigma_max / result.wrist_sigma_min :
    std::numeric_limits<double>::infinity();
  for (const double singular_value : wrist_singular_values) {
    if (singular_value > config.rank_threshold) {
      ++result.wrist_rank;
    }
  }

  result.orientation_speed_scale = singularitySpeedScale(
    result.wrist_sigma_min, config.wrist_sigma_slowdown_start,
    config.wrist_sigma_stop, config.orientation_singularity_speed_scale_min);

  // A position singularity is a primary-task safety condition. A severely
  // ill-conditioned full wrist also cannot provide a reliable orientation
  // correction. Keep the valid primary result and do not inject a noisy
  // secondary command in either case.
  if (result.position_sigma_min <= config.position_sigma_stop) {
    result.orientation_status = SolverStatus::SingularityRejected;
    return result;
  }
  if (result.wrist_rank != kCartesianDof) {
    result.orientation_status = SolverStatus::RankDeficient;
    if (input.elbow_geometry_valid) {
      result.elbow_status = SolverStatus::RankDeficient;
    }
    return result;
  }
  if (result.wrist_sigma_min <= config.wrist_sigma_stop) {
    result.orientation_status = SolverStatus::SingularityRejected;
    return result;
  }

  const ReducedOrientationJacobian reduced_jacobian =
    input.angular_jacobian * null_basis;
  result.desired_angular_velocity = result.orientation_speed_scale * clampVectorNorm(
    config.rotation_gain * applyDeadband(
      input.orientation_error, config.orientation_error_deadband),
    config.max_angular_velocity);
  const Eigen::Vector3d angular_residual =
    result.desired_angular_velocity - input.angular_jacobian * result.qdot_position;

  const Eigen::Matrix<double, kPositionNullity, kPositionNullity> identity =
    Eigen::Matrix<double, kPositionNullity, kPositionNullity>::Identity();
  impl_->orientation_qp.model.H =
    reduced_jacobian.transpose() * reduced_jacobian +
    (config.orientation_regularization + config.orientation_smoothness_weight) * identity;
  impl_->orientation_qp.model.H = 0.5 *
    (impl_->orientation_qp.model.H + impl_->orientation_qp.model.H.transpose());
  impl_->orientation_qp.model.g =
    -reduced_jacobian.transpose() * angular_residual -
    config.orientation_smoothness_weight * null_basis.transpose() *
    (input.position.qdot_previous - result.qdot_position);
  impl_->orientation_qp.model.C = null_basis;
  impl_->orientation_qp.model.l = result.qdot_lower - result.qdot_position;
  impl_->orientation_qp.model.u = result.qdot_upper - result.qdot_position;

  const auto solve_start = std::chrono::steady_clock::now();
  impl_->orientation_qp.update(
    impl_->orientation_qp.model.H, impl_->orientation_qp.model.g,
    impl_->orientation_qp.model.A, impl_->orientation_qp.model.b,
    impl_->orientation_qp.model.C,
    impl_->orientation_qp.model.l, impl_->orientation_qp.model.u, false);
  impl_->orientation_qp.solve();
  const auto solve_end = std::chrono::steady_clock::now();
  result.orientation_solve_time_us =
    std::chrono::duration<double, std::micro>(solve_end - solve_start).count();
  result.solve_time_us += result.orientation_solve_time_us;

  const auto proxqp_status = impl_->orientation_qp.results.info.status;
  result.orientation_proxqp_status = static_cast<int>(proxqp_status);
  if (proxqp_status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED ||
    impl_->orientation_qp.results.x.size() != kPositionNullity ||
    !impl_->orientation_qp.results.x.allFinite())
  {
    result.orientation_status = SolverStatus::QpFailure;
    result.qdot_pose = result.qdot;
    return result;
  }

  ArmVector orientation_increment = null_basis * impl_->orientation_qp.results.x;
  double safe_scale = 1.0;
  for (int i = 0; i < kSingleArmDof; ++i) {
    if (orientation_increment[i] > 1.0e-12) {
      safe_scale = std::min(
        safe_scale,
        (result.qdot_upper[i] - result.qdot_position[i]) / orientation_increment[i]);
    } else if (orientation_increment[i] < -1.0e-12) {
      safe_scale = std::min(
        safe_scale,
        (result.qdot_lower[i] - result.qdot_position[i]) / orientation_increment[i]);
    }
  }
  safe_scale = std::clamp(safe_scale, 0.0, 1.0);
  orientation_increment *= safe_scale;
  const ArmVector candidate = result.qdot_position + orientation_increment;
  if (!candidate.allFinite() ||
    (candidate.array() < result.qdot_lower.array() - 1.0e-10).any() ||
    (candidate.array() > result.qdot_upper.array() + 1.0e-10).any())
  {
    result.orientation_status = SolverStatus::QpFailure;
    result.qdot_pose = result.qdot;
    return result;
  }

  result.position_degradation_mps =
    (input.position.jacobian * orientation_increment).norm();
  if (!std::isfinite(result.position_degradation_mps) ||
    result.position_degradation_mps > config.max_orientation_position_degradation)
  {
    result.orientation_status = SolverStatus::DegradationRejected;
    result.qdot_pose = result.qdot;
    return result;
  }

  result.qdot = candidate;
  result.qdot_pose = candidate;
  result.achieved_linear_velocity = input.position.jacobian * result.qdot;
  result.achieved_angular_velocity = input.angular_jacobian * result.qdot;
  result.orientation_applied = true;
  result.orientation_status = SolverStatus::Solved;

  // Phase 5.2: solve the anthropomorphic elbow as a scalar task in the
  // null-space of the complete 6D wrist Jacobian. Consequently this task can
  // only consume the single redundancy direction left by a 7-DoF arm and
  // cannot intentionally trade wrist position or orientation for elbow shape.
  if (!input.elbow_geometry_valid) {
    return result;
  }
  if (!input.elbow_swivel_jacobian.allFinite() ||
    !std::isfinite(input.elbow_swivel_error) || !input.q_rest.allFinite())
  {
    result.elbow_status = SolverStatus::InvalidInput;
    return result;
  }
  if (result.wrist_rank != kCartesianDof) {
    result.elbow_status = SolverStatus::RankDeficient;
    return result;
  }
  if (result.wrist_sigma_min <= config.elbow_sigma_disable) {
    result.elbow_status = SolverStatus::SingularityRejected;
    return result;
  }

  const WristNullBasis wrist_null_basis = wrist_svd.matrixV().rightCols<1>();
  if (!wrist_null_basis.allFinite()) {
    result.elbow_status = SolverStatus::SingularityRejected;
    return result;
  }

  const double fade_denominator =
    config.elbow_sigma_fade_start - config.elbow_sigma_disable;
  const double elbow_scale = std::clamp(
    (result.wrist_sigma_min - config.elbow_sigma_disable) / fade_denominator,
    0.0, 1.0);
  result.elbow_scale = elbow_scale;
  if (elbow_scale <= 0.0) {
    result.elbow_status = SolverStatus::SingularityRejected;
    return result;
  }

  const double reduced_swivel_jacobian =
    (input.elbow_swivel_jacobian * wrist_null_basis)[0];
  if (!std::isfinite(reduced_swivel_jacobian)) {
    result.elbow_status = SolverStatus::InvalidInput;
    return result;
  }

  const double desired_swivel_velocity = std::clamp(
    config.elbow_swivel_gain * input.elbow_swivel_error,
    -config.max_elbow_swivel_velocity,
    config.max_elbow_swivel_velocity) * elbow_scale;
  const double achieved_pose_swivel_velocity =
    (input.elbow_swivel_jacobian * result.qdot_pose)[0];
  const double swivel_residual =
    desired_swivel_velocity - achieved_pose_swivel_velocity;

  const double posture_target = std::clamp(
    wrist_null_basis.dot(input.q_rest - input.position.q_measured),
    -config.max_elbow_swivel_velocity,
    config.max_elbow_swivel_velocity);
  const ArmVector joint_center =
    0.5 * (input.position.q_min + input.position.q_max) -
    input.position.q_measured;
  const double centering_target = std::clamp(
    wrist_null_basis.dot(joint_center),
    -config.max_elbow_swivel_velocity,
    config.max_elbow_swivel_velocity);
  const double smoothness_target = wrist_null_basis.dot(
    input.position.qdot_previous - result.qdot_pose);
  const double weak_task_scale = elbow_scale;

  impl_->elbow_qp.model.H(0, 0) =
    reduced_swivel_jacobian * reduced_swivel_jacobian +
    weak_task_scale * (
      config.elbow_posture_weight + config.elbow_joint_centering_weight +
      config.elbow_smoothness_weight) + config.elbow_regularization;
  impl_->elbow_qp.model.g(0) =
    -reduced_swivel_jacobian * swivel_residual -
    weak_task_scale * (
      config.elbow_posture_weight * posture_target +
      config.elbow_joint_centering_weight * centering_target +
      config.elbow_smoothness_weight * smoothness_target);
  impl_->elbow_qp.model.C = wrist_null_basis;
  impl_->elbow_qp.model.l = result.qdot_lower - result.qdot_pose;
  impl_->elbow_qp.model.u = result.qdot_upper - result.qdot_pose;

  const auto elbow_solve_start = std::chrono::steady_clock::now();
  impl_->elbow_qp.update(
    impl_->elbow_qp.model.H, impl_->elbow_qp.model.g,
    impl_->elbow_qp.model.A, impl_->elbow_qp.model.b,
    impl_->elbow_qp.model.C, impl_->elbow_qp.model.l,
    impl_->elbow_qp.model.u, false);
  impl_->elbow_qp.solve();
  const auto elbow_solve_end = std::chrono::steady_clock::now();
  result.elbow_solve_time_us = std::chrono::duration<double, std::micro>(
    elbow_solve_end - elbow_solve_start).count();
  result.solve_time_us += result.elbow_solve_time_us;

  const auto elbow_proxqp_status = impl_->elbow_qp.results.info.status;
  result.elbow_proxqp_status = static_cast<int>(elbow_proxqp_status);
  if (elbow_proxqp_status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED ||
    impl_->elbow_qp.results.x.size() != 1 ||
    !impl_->elbow_qp.results.x.allFinite())
  {
    result.elbow_status = SolverStatus::QpFailure;
    return result;
  }

  const ArmVector elbow_increment = wrist_null_basis * impl_->elbow_qp.results.x[0];
  double elbow_safe_scale = 1.0;
  for (int i = 0; i < kSingleArmDof; ++i) {
    if (elbow_increment[i] > 1.0e-12) {
      elbow_safe_scale = std::min(
        elbow_safe_scale,
        (result.qdot_upper[i] - result.qdot_pose[i]) / elbow_increment[i]);
    } else if (elbow_increment[i] < -1.0e-12) {
      elbow_safe_scale = std::min(
        elbow_safe_scale,
        (result.qdot_lower[i] - result.qdot_pose[i]) / elbow_increment[i]);
    }
  }
  elbow_safe_scale = std::clamp(elbow_safe_scale, 0.0, 1.0);
  const ArmVector safe_elbow_increment = elbow_safe_scale * elbow_increment;
  const ArmVector elbow_candidate = result.qdot_pose + safe_elbow_increment;
  if (!elbow_candidate.allFinite() ||
    (elbow_candidate.array() < result.qdot_lower.array() - 1.0e-10).any() ||
    (elbow_candidate.array() > result.qdot_upper.array() + 1.0e-10).any())
  {
    result.elbow_status = SolverStatus::QpFailure;
    return result;
  }

  result.elbow_position_degradation_mps =
    (input.position.jacobian * safe_elbow_increment).norm();
  result.elbow_orientation_degradation_rps =
    (input.angular_jacobian * safe_elbow_increment).norm();
  if (!std::isfinite(result.elbow_position_degradation_mps) ||
    !std::isfinite(result.elbow_orientation_degradation_rps) ||
    result.elbow_position_degradation_mps > config.max_elbow_position_degradation ||
    result.elbow_orientation_degradation_rps > config.max_elbow_orientation_degradation)
  {
    result.elbow_status = SolverStatus::DegradationRejected;
    return result;
  }

  result.qdot = elbow_candidate;
  result.achieved_linear_velocity = input.position.jacobian * result.qdot;
  result.achieved_angular_velocity = input.angular_jacobian * result.qdot;
  result.elbow_swivel_velocity_desired = desired_swivel_velocity;
  result.elbow_swivel_velocity_achieved =
    (input.elbow_swivel_jacobian * result.qdot)[0];
  result.elbow_applied = true;
  result.elbow_status = SolverStatus::Solved;
  return result;
}

const HierarchicalDIKSolver::Config & HierarchicalDIKSolver::config() const
{
  return impl_->config;
}

}  // namespace qiling_kinematics
