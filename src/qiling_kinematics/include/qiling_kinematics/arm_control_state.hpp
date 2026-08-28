#pragma once

#include <string>

#include "qiling_kinematics/ik_types.hpp"

namespace qiling_kinematics
{

class ArmControlState
{
public:
  explicit ArmControlState(std::string side_name);

  void initialize(const ArmVector & measured);
  void updateGripRequest(bool pressed, const ArmVector & measured);
  void enterHold(
    const ArmVector & measured, bool require_release, const std::string & reason);
  void enterFault(const ArmVector & measured, const std::string & reason);

  void recordSolveSuccess();
  void recordSolveFailure(
    const ArmVector & measured, int failures_to_fault, const std::string & reason);

  bool integrateReference(
    const ArmVector & qdot, double dt,
    const ArmVector & q_min, const ArmVector & q_max);

  bool initialized() const;
  bool active() const;
  bool releaseRequired() const;
  ArmRunState state() const;
  int consecutiveSolveFailures() const;
  const std::string & sideName() const;
  const std::string & reason() const;
  const ArmVector & reference() const;
  const ArmVector & previousVelocity() const;

private:
  void resetMotion(const ArmVector & measured);

  std::string side_name_;
  std::string reason_{"not initialized"};
  ArmRunState state_{ArmRunState::Hold};
  ArmVector reference_{ArmVector::Zero()};
  ArmVector previous_velocity_{ArmVector::Zero()};
  bool initialized_{false};
  bool release_required_{false};
  bool grip_pressed_{false};
  int consecutive_solve_failures_{0};
};

}  // namespace qiling_kinematics

