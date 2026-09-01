#include "qiling_kinematics/arm_control_state.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace qiling_kinematics
{

ArmControlState::ArmControlState(std::string side_name)
: side_name_(std::move(side_name))
{
}

void ArmControlState::initialize(const ArmVector & measured)
{
  if (!measured.allFinite()) {
    return;
  }
  initialized_ = true;
  state_ = ArmRunState::Hold;
  release_required_ = false;
  grip_pressed_ = false;
  consecutive_solve_failures_ = 0;
  reason_ = "initialized";
  resetMotion(measured);
}

void ArmControlState::synchronizeReference(
  const ArmVector & reference, const std::string & reason)
{
  if (!initialized_ || !reference.allFinite()) {
    return;
  }
  reference_ = reference;
  previous_velocity_.setZero();
  reason_ = reason;
}

void ArmControlState::updateGripRequest(bool pressed, const ArmVector & measured)
{
  if (!initialized_ || !measured.allFinite()) {
    return;
  }

  // Limit recovery owns the command reference until the measured state has
  // returned to the activation-safe region. Grip edges must not cancel this
  // path by re-synchronizing the reference to a still-out-of-range position.
  if (state_ == ArmRunState::LimitRecovery) {
    grip_pressed_ = pressed;
    previous_velocity_.setZero();
    return;
  }

  const bool rising_edge = pressed && !grip_pressed_;
  const bool falling_edge = !pressed && grip_pressed_;
  grip_pressed_ = pressed;

  if (!pressed) {
    if (state_ == ArmRunState::Fault) {
      state_ = ArmRunState::Hold;
      release_required_ = false;
      consecutive_solve_failures_ = 0;
      reason_ = "Grip released";
      // A fault invalidates the previous command reference. Re-synchronize
      // with feedback before accepting another active command.
      resetMotion(measured);
    } else if (release_required_) {
      state_ = ArmRunState::Hold;
      release_required_ = false;
      consecutive_solve_failures_ = 0;
      reason_ = "Grip released";
      // A non-fault release latch is only an input-edge guard. The reference
      // has already been selected by the transition that requested release
      // (recording hold, timeout, activation rejection, or limit recovery).
      // Replacing it with a later measured pose here would create a second,
      // hidden command step after an otherwise continuous handoff.
      previous_velocity_.setZero();
    } else if (falling_edge) {
      state_ = ArmRunState::Hold;
      consecutive_solve_failures_ = 0;
      reason_ = "Grip released";
      // Keep the last active reference so releasing the grip does not send a
      // measured-position step to the robot. The velocity history is cleared
      // because the next active solve starts from a stopped command state.
      previous_velocity_.setZero();
    }
    return;
  }

  if (rising_edge && state_ == ArmRunState::Hold && !release_required_) {
    state_ = ArmRunState::Active;
    consecutive_solve_failures_ = 0;
    reason_ = "Grip engaged";
    // Preserve the latched Hold/last-command reference. This makes the
    // Hold -> Active edge bumpless; measured is intentionally not copied here.
    previous_velocity_.setZero();
  }
}

void ArmControlState::enterHold(
  const ArmVector & measured, bool require_release, const std::string & reason)
{
  if (!initialized_ || !measured.allFinite()) {
    return;
  }
  const bool entering_hold = state_ != ArmRunState::Hold;
  const bool newly_requiring_release = require_release && !release_required_;
  state_ = ArmRunState::Hold;
  release_required_ = release_required_ || require_release;
  reason_ = reason;
  // Latch the hold target only when Hold is entered. Repeated calls from the
  // control loop must not replace it with a gravity-displaced measurement.
  if (entering_hold || newly_requiring_release) {
    resetMotion(measured);
  }
}

void ArmControlState::rejectActivation(
  bool pressed, const ArmVector & measured, const std::string & reason)
{
  if (!initialized_ || !measured.allFinite()) {
    return;
  }
  const bool entering_rejected_hold = state_ != ArmRunState::Hold || !release_required_;
  state_ = ArmRunState::Hold;
  release_required_ = true;
  grip_pressed_ = pressed;
  consecutive_solve_failures_ = 0;
  reason_ = reason;
  if (entering_rejected_hold) {
    resetMotion(measured);
  } else {
    previous_velocity_.setZero();
  }
}

void ArmControlState::enterLimitRecovery(
  const ArmVector & initial_reference, bool pressed, const std::string & reason)
{
  if (!initialized_ || !initial_reference.allFinite()) {
    return;
  }
  state_ = ArmRunState::LimitRecovery;
  release_required_ = true;
  grip_pressed_ = pressed;
  consecutive_solve_failures_ = 0;
  reason_ = reason;
  reference_ = initial_reference;
  previous_velocity_.setZero();
}

void ArmControlState::updateRecoveryReference(const ArmVector & reference)
{
  if (state_ != ArmRunState::LimitRecovery || !reference.allFinite()) {
    return;
  }
  reference_ = reference;
  previous_velocity_.setZero();
}

void ArmControlState::finishLimitRecovery(bool pressed, const std::string & reason)
{
  if (state_ != ArmRunState::LimitRecovery) {
    return;
  }
  state_ = ArmRunState::Hold;
  release_required_ = pressed;
  grip_pressed_ = pressed;
  consecutive_solve_failures_ = 0;
  reason_ = reason;
  previous_velocity_.setZero();
}

void ArmControlState::enterFault(const ArmVector & measured, const std::string & reason)
{
  if (!initialized_ || !measured.allFinite()) {
    return;
  }
  state_ = ArmRunState::Fault;
  release_required_ = true;
  reason_ = reason;
  resetMotion(measured);
}

void ArmControlState::recordSolveSuccess()
{
  consecutive_solve_failures_ = 0;
}

void ArmControlState::recordSolveFailure(
  const ArmVector & measured, int failures_to_fault, const std::string & reason)
{
  if (!initialized_ || !measured.allFinite()) {
    return;
  }

  ++consecutive_solve_failures_;
  resetMotion(measured);
  reason_ = reason;

  if (consecutive_solve_failures_ >= std::max(failures_to_fault, 1)) {
    state_ = ArmRunState::Fault;
    release_required_ = true;
  }
}

bool ArmControlState::integrateReference(
  const ArmVector & qdot, double dt, const ArmVector & measured,
  const ArmVector & q_min, const ArmVector & q_max, double max_tracking_error)
{
  if (!initialized_ || state_ != ArmRunState::Active ||
    !qdot.allFinite() || !measured.allFinite() || !q_min.allFinite() ||
    !q_max.allFinite() || !std::isfinite(dt) || dt <= 0.0 ||
    !std::isfinite(max_tracking_error) || max_tracking_error < 0.0)
  {
    return false;
  }

  for (int i = 0; i < kSingleArmDof; ++i) {
    if (q_min[i] > q_max[i] || measured[i] < q_min[i] || measured[i] > q_max[i]) {
      return false;
    }
    const double integrated = reference_[i] + dt * qdot[i];
    const double tracking_min = std::max(q_min[i], measured[i] - max_tracking_error);
    const double tracking_max = std::min(q_max[i], measured[i] + max_tracking_error);
    if (tracking_min > tracking_max) {
      return false;
    }
    reference_[i] = std::clamp(integrated, tracking_min, tracking_max);
  }
  previous_velocity_ = qdot;
  return reference_.allFinite();
}

bool ArmControlState::initialized() const
{
  return initialized_;
}

bool ArmControlState::active() const
{
  return state_ == ArmRunState::Active;
}

bool ArmControlState::recovering() const
{
  return state_ == ArmRunState::LimitRecovery;
}

bool ArmControlState::releaseRequired() const
{
  return release_required_;
}

ArmRunState ArmControlState::state() const
{
  return state_;
}

int ArmControlState::consecutiveSolveFailures() const
{
  return consecutive_solve_failures_;
}

const std::string & ArmControlState::sideName() const
{
  return side_name_;
}

const std::string & ArmControlState::reason() const
{
  return reason_;
}

const ArmVector & ArmControlState::reference() const
{
  return reference_;
}

const ArmVector & ArmControlState::previousVelocity() const
{
  return previous_velocity_;
}

void ArmControlState::resetMotion(const ArmVector & measured)
{
  reference_ = measured;
  previous_velocity_.setZero();
}

}  // namespace qiling_kinematics
