#include <filesystem>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "qiling_kinematics/dual_arm_kinematics.hpp"
#include "qiling_kinematics/anthropomorphic_elbow.hpp"
#include "qiling_kinematics/hierarchical_dik_solver.hpp"

namespace
{

using qiling_kinematics::ArmSide;
using qiling_kinematics::ArmVector;
using qiling_kinematics::AnthropomorphicElbow;
using qiling_kinematics::AnthropomorphicElbowConfig;
using qiling_kinematics::DualArmKinematics;
using qiling_kinematics::HierarchicalDIKSolver;
using qiling_kinematics::computeElbowSwivelJacobian;

std::filesystem::path urdfPath()
{
  return std::filesystem::path(
    ament_index_cpp::get_package_share_directory("qi_robot_description")) /
         "urdf" / "s4_dual_arm.urdf";
}

ArmVector leftHome()
{
  ArmVector q;
  q << -0.45, 0.30, 0.02, -1.40, 1.40, 0.0, 0.0;
  return q;
}

ArmVector rightHome()
{
  ArmVector q;
  q << -0.45, -0.30, 0.02, -1.40, 1.40, 0.0, 0.0;
  return q;
}

TEST(PoseHierarchyKinematics, BothS4ArmsTrackOrientationInPositionNullSpace)
{
  DualArmKinematics kinematics(urdfPath());
  const ArmVector left = leftHome();
  const ArmVector right = rightHome();
  const auto state = kinematics.evaluate(left, right);

  HierarchicalDIKSolver::Config config;
  config.position_gain = 3.0;
  config.max_linear_velocity = 0.20;
  config.position_regularization = 1.0e-8;
  config.position_smoothness_weight = 0.0;
  config.rotation_gain = 3.0;
  config.max_angular_velocity = 0.80;
  config.orientation_regularization = 1.0e-6;
  config.orientation_smoothness_weight = 0.0;
  config.rank_threshold = 1.0e-5;
  config.max_orientation_position_degradation = 1.0e-7;
  config.max_joint_velocity_rps.setConstant(2.0);

  for (const ArmSide side : {ArmSide::Left, ArmSide::Right}) {
    const int index = side == ArmSide::Left ? 0 : 1;
    for (int axis = 0; axis < 3; ++axis) {
      HierarchicalDIKSolver solver(config);
      HierarchicalDIKSolver::PoseHierarchyInput input;
      input.position.jacobian = state[index].wrist_jacobian.topRows<3>();
      input.position.q_measured = index == 0 ? left : right;
      input.position.q_min = kinematics.lowerPositionLimits(side);
      input.position.q_max = kinematics.upperPositionLimits(side);
      input.position.dt = 0.02;
      input.angular_jacobian = state[index].wrist_jacobian.bottomRows<3>();
      input.orientation_error[axis] = 0.10;

      const auto result = solver.solvePoseHierarchy(input);
      ASSERT_TRUE(result.success) << "side=" << index << " axis=" << axis;
      ASSERT_TRUE(result.orientation_applied) <<
        "side=" << index << " axis=" << axis <<
        " rank=" << result.position_rank;
      EXPECT_EQ(result.position_rank, 3);
      EXPECT_EQ(result.wrist_rank, 6);
      EXPECT_GT(result.position_sigma_min, config.rank_threshold);
      EXPECT_GT(result.wrist_sigma_min, config.rank_threshold);
      EXPECT_LT(result.position_degradation_mps, 1.0e-7);
      EXPECT_GT(result.achieved_angular_velocity[axis], 0.20)
        << "side=" << index << " axis=" << axis;
      EXPECT_LE(result.qdot.cwiseAbs().maxCoeff(), 2.0 + 1.0e-10);
    }
  }
}

TEST(PoseHierarchyKinematics, BothS4ArmsMaintainOrientationDuringTranslation)
{
  DualArmKinematics kinematics(urdfPath());
  const ArmVector left = leftHome();
  const ArmVector right = rightHome();
  const auto state = kinematics.evaluate(left, right);

  HierarchicalDIKSolver::Config config;
  config.position_gain = 3.0;
  config.max_linear_velocity = 0.20;
  config.position_regularization = 1.0e-8;
  config.position_smoothness_weight = 0.0;
  config.rotation_gain = 3.0;
  config.max_angular_velocity = 0.80;
  config.orientation_regularization = 1.0e-6;
  config.orientation_smoothness_weight = 0.0;
  config.rank_threshold = 1.0e-5;
  config.max_orientation_position_degradation = 1.0e-7;
  config.max_joint_velocity_rps.setConstant(2.0);

  for (const ArmSide side : {ArmSide::Left, ArmSide::Right}) {
    const int index = side == ArmSide::Left ? 0 : 1;
    for (int axis = 0; axis < 3; ++axis) {
      HierarchicalDIKSolver solver(config);
      HierarchicalDIKSolver::PoseHierarchyInput input;
      input.position.jacobian = state[index].wrist_jacobian.topRows<3>();
      input.position.position_error[axis] = 0.01;
      input.position.q_measured = index == 0 ? left : right;
      input.position.q_min = kinematics.lowerPositionLimits(side);
      input.position.q_max = kinematics.upperPositionLimits(side);
      input.position.dt = 0.02;
      input.angular_jacobian = state[index].wrist_jacobian.bottomRows<3>();

      const auto result = solver.solvePoseHierarchy(input);
      ASSERT_TRUE(result.success) << "side=" << index << " axis=" << axis;
      ASSERT_TRUE(result.orientation_applied) << "side=" << index << " axis=" << axis;
      EXPECT_GT(result.achieved_linear_velocity[axis], 0.02);
      EXPECT_LT(result.position_degradation_mps, 1.0e-7);
      EXPECT_LT(result.achieved_angular_velocity.norm(), 5.0e-3)
        << "side=" << index << " axis=" << axis;
    }
  }
}

TEST(PoseHierarchyKinematics, S4ElbowTaskUsesWristNullspaceAtHome)
{
  DualArmKinematics kinematics(urdfPath());
  const ArmVector left = leftHome();
  const ArmVector right = rightHome();
  const auto state = kinematics.evaluate(left, right);

  HierarchicalDIKSolver::Config config;
  config.position_gain = 3.0;
  config.max_linear_velocity = 0.20;
  config.position_regularization = 1.0e-8;
  config.position_smoothness_weight = 0.0;
  config.rotation_gain = 3.0;
  config.max_angular_velocity = 0.80;
  config.orientation_regularization = 1.0e-6;
  config.orientation_smoothness_weight = 0.0;
  config.rank_threshold = 1.0e-5;
  config.max_orientation_position_degradation = 1.0e-7;
  config.max_elbow_position_degradation = 1.0e-7;
  config.max_elbow_orientation_degradation = 1.0e-6;
  config.max_joint_velocity_rps.setConstant(2.0);

  AnthropomorphicElbowConfig elbow_config;
  AnthropomorphicElbow elbow;
  for (const ArmSide side : {ArmSide::Left, ArmSide::Right}) {
    const int index = side == ArmSide::Left ? 0 : 1;
    const ArmVector & q = index == 0 ? left : right;
    const auto & arm = state[index];
    const Eigen::Vector3d shoulder = arm.shoulder_pose.translation();
    const Eigen::Vector3d elbow_position = arm.elbow_pose.translation();
    const Eigen::Vector3d wrist = arm.wrist_pose.translation();
    const Eigen::Vector3d axis = (wrist - shoulder).normalized();
    Eigen::Vector3d home_direction =
      (Eigen::Matrix3d::Identity() - axis * axis.transpose()) *
      (elbow_position - shoulder);
    home_direction.normalize();
    const auto geometry = elbow.compute(
      side, shoulder, elbow_position, wrist, home_direction, elbow_config);
    ASSERT_TRUE(geometry.valid) << "side=" << index;

    HierarchicalDIKSolver::PoseHierarchyInput input;
    input.position.jacobian = arm.wrist_jacobian.topRows<3>();
    input.position.q_measured = q;
    input.position.q_min = kinematics.lowerPositionLimits(side);
    input.position.q_max = kinematics.upperPositionLimits(side);
    input.position.dt = 0.02;
    input.angular_jacobian = arm.wrist_jacobian.bottomRows<3>();
    input.elbow_geometry_valid = true;
    input.elbow_swivel_error = 0.10;
    input.q_rest = q;
    input.elbow_swivel_jacobian = computeElbowSwivelJacobian(
      shoulder, elbow_position, wrist, arm.elbow_linear_jacobian,
      arm.wrist_jacobian.topRows<3>(), geometry.shoulder_wrist_axis,
      geometry.current_direction, geometry.current_projection_radius);

    HierarchicalDIKSolver solver(config);
    const auto result = solver.solvePoseHierarchy(input);
    ASSERT_TRUE(result.success) << "side=" << index;
    EXPECT_EQ(result.wrist_rank, 6);
    EXPECT_TRUE(result.orientation_applied);
    EXPECT_EQ(result.elbow_status, qiling_kinematics::SolverStatus::Solved);
    EXPECT_TRUE(result.elbow_applied);
    EXPECT_LT(result.elbow_position_degradation_mps, 1.0e-7);
    EXPECT_LT(result.elbow_orientation_degradation_rps, 1.0e-6);
  }
}

}  // namespace
