#include <cmath>

#include <gtest/gtest.h>

#include "qiling_kinematics/hierarchical_dik_solver.hpp"

namespace
{

using qiling_kinematics::HierarchicalDIKSolver;
using qiling_kinematics::SolverStatus;
using qiling_kinematics::kSingleArmDof;

HierarchicalDIKSolver::Config hierarchyConfig()
{
  HierarchicalDIKSolver::Config config;
  config.position_gain = 3.0;
  config.max_linear_velocity = 0.20;
  config.position_regularization = 1.0e-8;
  config.position_smoothness_weight = 0.0;
  config.rotation_gain = 3.0;
  config.max_angular_velocity = 0.80;
  config.orientation_regularization = 1.0e-8;
  config.orientation_smoothness_weight = 0.0;
  config.rank_threshold = 1.0e-6;
  config.max_orientation_position_degradation = 1.0e-8;
  config.max_joint_velocity_rps.setConstant(2.0);
  return config;
}

HierarchicalDIKSolver::PoseHierarchyInput decoupledInput()
{
  HierarchicalDIKSolver::PoseHierarchyInput input;
  input.position.jacobian.block<3, 3>(0, 0).setIdentity();
  input.angular_jacobian.block<3, 3>(0, 3).setIdentity();
  input.position.q_measured.setZero();
  input.position.q_min.setConstant(-2.0);
  input.position.q_max.setConstant(2.0);
  input.position.dt = 0.02;
  return input;
}

HierarchicalDIKSolver::PoseHierarchyInput elbowInput()
{
  auto input = decoupledInput();
  input.elbow_geometry_valid = true;
  input.elbow_swivel_jacobian(0, 6) = 1.0;
  input.elbow_swivel_error = 0.10;
  return input;
}

TEST(PoseHierarchySolver, OrientationTracksAllAxesWithoutChangingPrimaryPosition)
{
  for (int axis = 0; axis < 3; ++axis) {
    HierarchicalDIKSolver solver(hierarchyConfig());
    auto input = decoupledInput();
    input.position.position_error << 0.02, -0.01, 0.015;
    input.orientation_error[axis] = 0.10;

    const auto result = solver.solvePoseHierarchy(input);
    ASSERT_TRUE(result.success) << "axis=" << axis;
    ASSERT_TRUE(result.orientation_applied) << "axis=" << axis;
    EXPECT_EQ(result.orientation_status, SolverStatus::Solved);
    EXPECT_EQ(result.position_rank, 3);
    EXPECT_EQ(result.wrist_rank, 6);
    EXPECT_NEAR(result.wrist_sigma_min, 0.25, 1.0e-12);
    EXPECT_LT(
      (input.position.jacobian * (result.qdot - result.qdot_position)).norm(), 1.0e-10);
    EXPECT_LT(result.position_degradation_mps, 1.0e-10);
    EXPECT_GT(result.achieved_angular_velocity[axis], 0.299);
  }
}

TEST(PoseHierarchySolver, OrientationNormLimitAndFinalJointBoundsAreEnforced)
{
  auto config = hierarchyConfig();
  config.max_angular_velocity = 0.20;
  config.max_joint_velocity_rps.setConstant(0.08);
  HierarchicalDIKSolver solver(config);
  auto input = decoupledInput();
  input.orientation_error << 2.0, -3.0, 4.0;

  const auto result = solver.solvePoseHierarchy(input);
  ASSERT_TRUE(result.success);
  ASSERT_TRUE(result.orientation_applied);
  EXPECT_NEAR(result.desired_angular_velocity.norm(), 0.20, 1.0e-12);
  for (int i = 0; i < kSingleArmDof; ++i) {
    EXPECT_GE(result.qdot[i], result.qdot_lower[i] - 1.0e-12);
    EXPECT_LE(result.qdot[i], result.qdot_upper[i] + 1.0e-12);
  }
  EXPECT_LT(result.position_degradation_mps, 1.0e-10);
}

TEST(PoseHierarchySolver, OrientationLayerCancelsPrimaryAngularLeakage)
{
  HierarchicalDIKSolver solver(hierarchyConfig());
  auto input = decoupledInput();
  input.angular_jacobian.block<3, 3>(0, 0).setIdentity();
  input.position.position_error << 0.02, -0.01, 0.015;

  const auto result = solver.solvePoseHierarchy(input);
  ASSERT_TRUE(result.success);
  ASSERT_TRUE(result.orientation_applied);
  EXPECT_GT((input.angular_jacobian * result.qdot_position).norm(), 0.05);
  EXPECT_LT(result.achieved_angular_velocity.norm(), 1.0e-7);
  EXPECT_LT(
    (input.position.jacobian * (result.qdot - result.qdot_position)).norm(), 1.0e-10);
}

TEST(PoseHierarchySolver, RankDeficiencyFallsBackToPositionPrimary)
{
  HierarchicalDIKSolver solver(hierarchyConfig());
  auto input = decoupledInput();
  input.position.jacobian.row(2).setZero();
  input.position.position_error << 0.02, 0.01, 0.0;
  input.orientation_error.z() = 0.2;

  const auto result = solver.solvePoseHierarchy(input);
  ASSERT_TRUE(result.success);
  EXPECT_FALSE(result.orientation_applied);
  EXPECT_EQ(result.orientation_status, SolverStatus::RankDeficient);
  EXPECT_EQ(result.position_rank, 2);
  EXPECT_TRUE(result.qdot.isApprox(result.qdot_position, 0.0));
}

TEST(PoseHierarchySolver, InvalidOrientationInputFallsBackToPositionPrimary)
{
  HierarchicalDIKSolver solver(hierarchyConfig());
  auto input = decoupledInput();
  input.position.position_error.x() = 0.02;
  input.orientation_error.y() = std::nan("");

  const auto result = solver.solvePoseHierarchy(input);
  ASSERT_TRUE(result.success);
  EXPECT_FALSE(result.orientation_applied);
  EXPECT_EQ(result.orientation_status, SolverStatus::InvalidInput);
  EXPECT_TRUE(result.qdot.isApprox(result.qdot_position, 0.0));
}

TEST(PoseHierarchySolver, ElbowUsesOnlyCompleteWristNullspace)
{
  HierarchicalDIKSolver solver(hierarchyConfig());
  auto input = elbowInput();
  input.position.position_error << 0.02, -0.01, 0.015;
  input.orientation_error << 0.03, -0.02, 0.01;

  const auto result = solver.solvePoseHierarchy(input);
  ASSERT_TRUE(result.success);
  ASSERT_TRUE(result.orientation_applied);
  ASSERT_TRUE(result.elbow_applied);
  EXPECT_EQ(result.elbow_status, SolverStatus::Solved);
  EXPECT_EQ(result.wrist_rank, 6);
  EXPECT_GT(result.elbow_swivel_velocity_achieved, 0.0);
  EXPECT_LT(result.elbow_position_degradation_mps, 1.0e-10);
  EXPECT_LT(result.elbow_orientation_degradation_rps, 1.0e-10);
  EXPECT_LT(
    (input.position.jacobian * (result.qdot - result.qdot_pose)).norm(), 1.0e-10);
  EXPECT_LT(
    (input.angular_jacobian * (result.qdot - result.qdot_pose)).norm(), 1.0e-10);
}

TEST(PoseHierarchySolver, ElbowIsRejectedWhenCompleteWristIsRankDeficient)
{
  HierarchicalDIKSolver solver(hierarchyConfig());
  auto input = elbowInput();
  input.angular_jacobian.row(2).setZero();

  const auto result = solver.solvePoseHierarchy(input);
  ASSERT_TRUE(result.success);
  EXPECT_FALSE(result.elbow_applied);
  EXPECT_EQ(result.elbow_status, SolverStatus::RankDeficient);
  EXPECT_TRUE(result.qdot.isApprox(result.qdot_pose, 0.0));
}

TEST(PoseHierarchySolver, OrientationIsHeldBelowWristSingularityThreshold)
{
  auto config = hierarchyConfig();
  config.wrist_sigma_slowdown_start = 0.08;
  config.wrist_sigma_stop = 0.001;
  HierarchicalDIKSolver solver(config);
  auto input = decoupledInput();
  input.angular_jacobian.block<3, 3>(0, 3) *= 0.003;
  input.orientation_error.x() = 0.5;

  const auto result = solver.solvePoseHierarchy(input);
  ASSERT_TRUE(result.success);
  EXPECT_FALSE(result.orientation_applied);
  EXPECT_EQ(result.orientation_status, SolverStatus::SingularityRejected);
  EXPECT_EQ(result.orientation_speed_scale, 0.0);
  EXPECT_TRUE(result.qdot.isApprox(result.qdot_position, 0.0));
}

}  // namespace
