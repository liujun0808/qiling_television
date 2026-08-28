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

void ArmControlState::updateGripRequest(bool pressed, const ArmVector & measured)
{
  if (!initialized_ || !measured.allFinite()) {
    return;
  }

  const bool rising_edge = pressed && !grip_pressed_;
  const bool falling_edge = !pressed && grip_pressed_;
  grip_pressed_ = pressed;

  if (!pressed) {
    if (state_ == ArmRunState::Fault || release_required_ || falling_edge) {
      state_ = ArmRunState::Hold;
      release_required_ = false;
      consecutive_solve_failures_ = 0;
      reason_ = "Grip released";
      resetMotion(measured);
    }
    return;
  }

  if (rising_edge && state_ == ArmRunState::Hold && !release_required_) {
    state_ = ArmRunState::Active;
    consecutive_solve_failures_ = 0;
    reason_ = "Grip engaged";
    resetMotion(measured);
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
  const ArmVector & qdot, double dt,
  const ArmVector & q_min, const ArmVector & q_max)
{
  if (!initialized_ || state_ != ArmRunState::Active ||
    !qdot.allFinite() || !q_min.allFinite() || !q_max.allFinite() ||
    !std::isfinite(dt) || dt <= 0.0)
  {
    return false;
  }

  for (int i = 0; i < kSingleArmDof; ++i) {
    if (q_min[i] > q_max[i]) {
      return false;
    }
    reference_[i] = std::clamp(reference_[i] + dt * qdot[i], q_min[i], q_max[i]);
  }
  previous_velocity_ = qdot;
  return reference_.allFinite();
}

bool ArmControlState::updateMeasuredStepReference(
  const ArmVector & qdot, double dt, const ArmVector & measured,
  const ArmVector & q_min, const ArmVector & q_max)
{
  if (!initialized_ || state_ != ArmRunState::Active ||
    !qdot.allFinite() || !measured.allFinite() ||
    !q_min.allFinite() || !q_max.allFinite() ||
    !std::isfinite(dt) || dt <= 0.0)
  {
    return false;
  }

  const ArmVector candidate = measured + dt * qdot;
  if (!candidate.allFinite()) {
    return false;
  }

  for (int i = 0; i < kSingleArmDof; ++i) {
    if (q_min[i] > q_max[i] || candidate[i] < q_min[i] || candidate[i] > q_max[i]) {
      return false;
    }
  }

  // Real-robot mode deliberately does not accumulate a long-lived reference.
  // Each command is one measured-state step, which prevents stale tracking
  // error from integrating indefinitely when the hardware lags the target.
  reference_ = candidate;
  previous_velocity_ = qdot;
  return true;
}

bool ArmControlState::initialized() const
{
  return initialized_;
}

bool ArmControlState::active() const
{
  return state_ == ArmRunState::Active;
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
