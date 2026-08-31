#include "qiling_kinematics/dual_arm_kinematics.hpp"

#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace qiling_kinematics
{

DualArmKinematics::DualArmKinematics(const std::filesystem::path & urdf_path)
{
  if (!std::filesystem::is_regular_file(urdf_path)) {
    throw std::runtime_error("Pinocchio URDF does not exist: " + urdf_path.string());
  }

  pinocchio::urdf::buildModel(urdf_path.string(), model_);
  data_ = pinocchio::Data(model_);
  q_ = pinocchio::neutral(model_);

  if (model_.nq != 2 * kSingleArmDof || model_.nv != 2 * kSingleArmDof) {
    throw std::runtime_error(
            "dual-arm URDF must expose exactly 14 movable DoF; got nq=" +
            std::to_string(model_.nq) + " nv=" + std::to_string(model_.nv));
  }

  joint_names_[0] = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"};
  joint_names_[1] = {
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};

  for (int side = 0; side < 2; ++side) {
    for (int column = 0; column < kSingleArmDof; ++column) {
      const pinocchio::JointIndex joint_id = model_.getJointId(joint_names_[side][column]);
      if (joint_id == 0 || joint_id >= static_cast<pinocchio::JointIndex>(model_.njoints)) {
        throw std::runtime_error("joint missing from Pinocchio model: " + joint_names_[side][column]);
      }
      if (model_.nqs[joint_id] != 1 || model_.nvs[joint_id] != 1) {
        throw std::runtime_error("expected one DoF joint: " + joint_names_[side][column]);
      }
      joint_ids_[side][column] = joint_id;
      q_indices_[side][column] = model_.idx_qs[joint_id];
      v_indices_[side][column] = model_.idx_vs[joint_id];
    }
  }

  shoulder_frames_[0] = requireFrame("left_shoulder_pitch_link");
  shoulder_frames_[1] = requireFrame("right_shoulder_pitch_link");
  elbow_frames_[0] = requireFrame("left_elbow_link");
  elbow_frames_[1] = requireFrame("right_elbow_link");
  wrist_frames_[0] = requireFrame("LH_hand_base_link");
  wrist_frames_[1] = requireFrame("RH_hand_base_link");
}

pinocchio::FrameIndex DualArmKinematics::requireFrame(const std::string & name) const
{
  const pinocchio::FrameIndex frame_id = model_.getFrameId(name);
  if (frame_id >= static_cast<pinocchio::FrameIndex>(model_.nframes)) {
    throw std::runtime_error("frame missing from Pinocchio model: " + name);
  }
  return frame_id;
}

Eigen::VectorXd DualArmKinematics::makeConfiguration(
  const ArmVector & left_q, const ArmVector & right_q) const
{
  if (!left_q.allFinite() || !right_q.allFinite()) {
    throw std::invalid_argument("arm configuration contains non-finite values");
  }

  Eigen::VectorXd result = pinocchio::neutral(model_);
  for (int column = 0; column < kSingleArmDof; ++column) {
    result[q_indices_[0][column]] = left_q[column];
    result[q_indices_[1][column]] = right_q[column];
  }
  return result;
}

ArmJacobian DualArmKinematics::frameJacobian(
  pinocchio::FrameIndex frame_id, ArmSide side)
{
  const Eigen::Matrix<double, 6, Eigen::Dynamic> full = pinocchio::getFrameJacobian(
    model_, data_, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  ArmJacobian result = ArmJacobian::Zero();
  const int index = sideIndex(side);
  for (int column = 0; column < kSingleArmDof; ++column) {
    result.col(column) = full.col(v_indices_[index][column]);
  }
  return result;
}

std::array<ArmKinematics, 2> DualArmKinematics::evaluate(
  const ArmVector & left_q, const ArmVector & right_q)
{
  q_ = makeConfiguration(left_q, right_q);
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::computeJointJacobians(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);

  std::array<ArmKinematics, 2> result;
  for (int side = 0; side < 2; ++side) {
    result[side].shoulder_pose = data_.oMf[shoulder_frames_[side]];
    result[side].elbow_pose = data_.oMf[elbow_frames_[side]];
    result[side].wrist_pose = data_.oMf[wrist_frames_[side]];
    result[side].wrist_jacobian = frameJacobian(
      wrist_frames_[side], side == 0 ? ArmSide::Left : ArmSide::Right);
    result[side].elbow_linear_jacobian = frameJacobian(
      elbow_frames_[side], side == 0 ? ArmSide::Left : ArmSide::Right).topRows<3>();
  }
  return result;
}

std::array<ArmVector, 2> DualArmKinematics::gravityTorques(
  const ArmVector & left_q, const ArmVector & right_q)
{
  q_ = makeConfiguration(left_q, right_q);
  pinocchio::computeGeneralizedGravity(model_, data_, q_);

  std::array<ArmVector, 2> result;
  for (int side = 0; side < 2; ++side) {
    for (int column = 0; column < kSingleArmDof; ++column) {
      result[side][column] = data_.g[v_indices_[side][column]];
    }
  }
  return result;
}

std::array<ArmVector, 2> DualArmKinematics::effortLimits() const
{
  std::array<ArmVector, 2> result;
  for (int side = 0; side < 2; ++side) {
    for (int column = 0; column < kSingleArmDof; ++column) {
      result[side][column] = model_.effortLimit[v_indices_[side][column]];
    }
  }
  return result;
}

ArmVector DualArmKinematics::positionLimits(ArmSide side, bool upper) const
{
  ArmVector result;
  const int index = sideIndex(side);
  const Eigen::VectorXd & source = upper ? model_.upperPositionLimit : model_.lowerPositionLimit;
  for (int column = 0; column < kSingleArmDof; ++column) {
    result[column] = source[q_indices_[index][column]];
  }
  return result;
}

ArmVector DualArmKinematics::lowerPositionLimits(ArmSide side) const
{
  return positionLimits(side, false);
}

ArmVector DualArmKinematics::upperPositionLimits(ArmSide side) const
{
  return positionLimits(side, true);
}

const std::array<std::string, kSingleArmDof> & DualArmKinematics::jointNames(
  ArmSide side) const
{
  return joint_names_[sideIndex(side)];
}

}  // namespace qiling_kinematics
