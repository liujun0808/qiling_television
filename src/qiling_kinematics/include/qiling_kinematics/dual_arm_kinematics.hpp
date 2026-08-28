#pragma once

#include <array>
#include <filesystem>
#include <string>

#include <Eigen/Core>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "qiling_kinematics/ik_types.hpp"

namespace qiling_kinematics
{

struct ArmKinematics
{
  pinocchio::SE3 shoulder_pose{pinocchio::SE3::Identity()};
  pinocchio::SE3 elbow_pose{pinocchio::SE3::Identity()};
  pinocchio::SE3 wrist_pose{pinocchio::SE3::Identity()};
  ArmLinearJacobian elbow_linear_jacobian{ArmLinearJacobian::Zero()};
  ArmJacobian wrist_jacobian{ArmJacobian::Zero()};
};

/**
 * Fixed-base Pinocchio model for the two 7-DoF arms.
 *
 * Every Cartesian quantity returned by this class is expressed along base_link
 * axes. Jacobians use LOCAL_WORLD_ALIGNED as required by the Phase 0 contract.
 */
class DualArmKinematics
{
public:
  explicit DualArmKinematics(const std::filesystem::path & urdf_path);

  std::array<ArmKinematics, 2> evaluate(
    const ArmVector & left_q, const ArmVector & right_q);

  // Generalized gravity torques in the same joint order as the two arm
  // vectors. The fixed base is part of the URDF model.
  std::array<ArmVector, 2> gravityTorques(
    const ArmVector & left_q, const ArmVector & right_q);

  std::array<ArmVector, 2> effortLimits() const;

  Eigen::VectorXd makeConfiguration(
    const ArmVector & left_q, const ArmVector & right_q) const;

  ArmVector lowerPositionLimits(ArmSide side) const;
  ArmVector upperPositionLimits(ArmSide side) const;

  const std::array<std::string, kSingleArmDof> & jointNames(ArmSide side) const;
  const pinocchio::Model & model() const {return model_;}

private:
  static constexpr int sideIndex(ArmSide side)
  {
    return side == ArmSide::Left ? 0 : 1;
  }

  pinocchio::FrameIndex requireFrame(const std::string & name) const;
  ArmJacobian frameJacobian(pinocchio::FrameIndex frame_id, ArmSide side);
  ArmVector positionLimits(ArmSide side, bool upper) const;

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;

  std::array<std::array<std::string, kSingleArmDof>, 2> joint_names_{};
  std::array<std::array<pinocchio::JointIndex, kSingleArmDof>, 2> joint_ids_{};
  std::array<std::array<int, kSingleArmDof>, 2> q_indices_{};
  std::array<std::array<int, kSingleArmDof>, 2> v_indices_{};
  std::array<pinocchio::FrameIndex, 2> shoulder_frames_{};
  std::array<pinocchio::FrameIndex, 2> elbow_frames_{};
  std::array<pinocchio::FrameIndex, 2> wrist_frames_{};
};

}  // namespace qiling_kinematics
