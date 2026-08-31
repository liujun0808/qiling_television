#pragma once

#include <Eigen/Core>

namespace qiling_kinematics
{

inline constexpr int kSingleArmDof = 7;
inline constexpr int kCartesianDof = 6;

using ArmVector = Eigen::Matrix<double, kSingleArmDof, 1>;
using ArmJacobian = Eigen::Matrix<double, kCartesianDof, kSingleArmDof>;
using ArmLinearJacobian = Eigen::Matrix<double, 3, kSingleArmDof>;
using ArmScalarJacobian = Eigen::Matrix<double, 1, kSingleArmDof>;
using CartesianVector = Eigen::Matrix<double, kCartesianDof, 1>;

enum class ArmSide
{
  Left = 0,
  Right = 1,
};

enum class ArmRunState
{
  Hold,
  Active,
  LimitRecovery,
  Fault,
};

enum class SolverStatus
{
  Solved,
  InvalidInput,
  InvalidBounds,
  QpFailure,
  RankDeficient,
  SingularityRejected,
  DegradationRejected,
};

inline const char * toString(ArmRunState state)
{
  switch (state) {
    case ArmRunState::Hold:
      return "HOLD";
    case ArmRunState::Active:
      return "ACTIVE";
    case ArmRunState::LimitRecovery:
      return "LIMIT_RECOVERY";
    case ArmRunState::Fault:
      return "FAULT";
  }
  return "UNKNOWN";
}

inline const char * toString(SolverStatus status)
{
  switch (status) {
    case SolverStatus::Solved:
      return "SOLVED";
    case SolverStatus::InvalidInput:
      return "INVALID_INPUT";
    case SolverStatus::InvalidBounds:
      return "INVALID_BOUNDS";
    case SolverStatus::QpFailure:
      return "QP_FAILURE";
    case SolverStatus::RankDeficient:
      return "RANK_DEFICIENT";
    case SolverStatus::SingularityRejected:
      return "SINGULARITY_REJECTED";
    case SolverStatus::DegradationRejected:
      return "DEGRADATION_REJECTED";
  }
  return "UNKNOWN";
}

}  // namespace qiling_kinematics
