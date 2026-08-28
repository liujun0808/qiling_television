#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "qiling_kinematics/hierarchical_dik_solver.hpp"

namespace
{

using qiling_kinematics::HierarchicalDIKSolver;
using qiling_kinematics::SolverStatus;
using qiling_kinematics::kSingleArmDof;

static_assert(
  decltype(HierarchicalDIKSolver::PositionInput{}.jacobian)::RowsAtCompileTime == 3,
  "Phase 3 position-primary input must not expose angular Jacobian rows");

HierarchicalDIKSolver::PositionInput nominalInput()
{
  HierarchicalDIKSolver::PositionInput input;
  input.jacobian.setZero();
  input.jacobian.block<3, 3>(0, 0).setIdentity();
  input.position_error.setZero();
  input.q_measured.setZero();
  input.q_min.setConstant(-2.0);
  input.q_max.setConstant(2.0);
  input.qdot_previous.setZero();
  input.dt = 0.02;
  return input;
}

HierarchicalDIKSolver::Config unregularizedConfig()
{
  HierarchicalDIKSolver::Config config;
  config.position_gain = 3.0;
  config.max_linear_velocity = 0.20;
  config.position_regularization = 1.0e-9;
  config.position_smoothness_weight = 0.0;
  config.max_joint_velocity_rps.setConstant(2.0);
  return config;
}

TEST(PositionPrimarySolver, ZeroErrorProducesZeroVelocity)
{
  HierarchicalDIKSolver solver(unregularizedConfig());
  const auto result = solver.solvePositionPrimary(nominalInput());

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.status, SolverStatus::Solved);
  EXPECT_TRUE(result.qdot.isZero(1.0e-9));
  EXPECT_TRUE(result.desired_linear_velocity.isZero(0.0));
}

TEST(PositionPrimarySolver, TracksPositiveAndNegativeBaseAxes)
{
  HierarchicalDIKSolver solver(unregularizedConfig());
  for (int axis = 0; axis < 3; ++axis) {
    for (const double sign : {-1.0, 1.0}) {
      auto input = nominalInput();
      input.position_error[axis] = sign * 0.02;
      const auto result = solver.solvePositionPrimary(input);
      ASSERT_TRUE(result.success) << "axis=" << axis << " sign=" << sign;
      EXPECT_GT(sign * result.achieved_linear_velocity[axis], 0.059)
        << "axis=" << axis << " sign=" << sign;
      for (int other = 0; other < 3; ++other) {
        if (other != axis) {
          EXPECT_NEAR(result.achieved_linear_velocity[other], 0.0, 1.0e-7);
        }
      }
    }
  }
}

TEST(PositionPrimarySolver, UnreachableTargetUsesNormLimitedCartesianVelocity)
{
  auto config = unregularizedConfig();
  config.position_gain = 10.0;
  config.max_linear_velocity = 0.20;
  HierarchicalDIKSolver solver(config);
  auto input = nominalInput();
  input.position_error << 10.0, -6.0, 3.0;

  const auto result = solver.solvePositionPrimary(input);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.desired_linear_velocity.norm(), config.max_linear_velocity, 1.0e-12);
  EXPECT_LE(result.achieved_linear_velocity.norm(), config.max_linear_velocity + 1.0e-7);
  EXPECT_GT(
    result.achieved_linear_velocity.dot(input.position_error.normalized()), 0.199);
}

TEST(PositionPrimarySolver, PerJointVelocityLimitsAreEnforced)
{
  auto config = unregularizedConfig();
  for (int i = 0; i < kSingleArmDof; ++i) {
    config.max_joint_velocity_rps[i] = 0.05 + 0.01 * i;
  }
  HierarchicalDIKSolver solver(config);
  auto input = nominalInput();
  input.position_error.setConstant(1.0);

  const auto result = solver.solvePositionPrimary(input);
  ASSERT_TRUE(result.success);
  for (int i = 0; i < kSingleArmDof; ++i) {
    EXPECT_LE(std::abs(result.qdot[i]), config.max_joint_velocity_rps[i] + 1.0e-12);
  }
}

TEST(PositionPrimarySolver, HardPositionBoundPreventsOutwardMotion)
{
  HierarchicalDIKSolver solver(unregularizedConfig());
  auto input = nominalInput();
  input.q_measured[0] = input.q_max[0];
  input.position_error.x() = 0.10;

  const auto result = solver.solvePositionPrimary(input);
  ASSERT_TRUE(result.success);
  EXPECT_LE(result.qdot[0], 1.0e-12);
}

TEST(PositionPrimarySolver, SoftMarginUsesFeasibleVelocityDamper)
{
  HierarchicalDIKSolver solver(unregularizedConfig());
  auto input = nominalInput();
  input.q_measured[0] = input.q_max[0] - 0.5 * solver.config().joint_limit_margin;
  input.position_error.x() = 1.0;

  const auto result = solver.solvePositionPrimary(input);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.joint_limit_damper_active);
  EXPECT_EQ(result.status, SolverStatus::Solved);
  EXPECT_LE(result.qdot[0], 1.0 + 1.0e-9);
  EXPECT_GE(result.qdot_upper[0], 0.0);
}

TEST(PositionPrimarySolver, HardLimitStillAllowsInwardRecovery)
{
  HierarchicalDIKSolver solver(unregularizedConfig());
  auto input = nominalInput();
  input.q_measured[0] = input.q_max[0];
  input.position_error.x() = -0.10;

  const auto result = solver.solvePositionPrimary(input);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.joint_limit_damper_active);
  EXPECT_LE(result.qdot_upper[0], 0.0);
  EXPECT_LT(result.qdot[0], -0.1);
}

TEST(PositionPrimarySolver, SmallHardLimitOvershootCommandsRecoveryWithoutInvalidBounds)
{
  auto config = unregularizedConfig();
  config.hard_limit_tolerance = 0.005;
  HierarchicalDIKSolver solver(config);
  auto input = nominalInput();
  input.q_measured[0] = input.q_max[0] + 0.003;
  input.position_error.x() = 0.10;

  const auto result = solver.solvePositionPrimary(input);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.joint_limit_damper_active);
  EXPECT_LE(result.qdot_upper[0], -0.149);
  EXPECT_LT(result.qdot[0], 0.0);
}

TEST(PositionPrimarySolver, ReportsPositionSingularValuesAndRank)
{
  HierarchicalDIKSolver solver(unregularizedConfig());
  const auto result = solver.solvePositionPrimary(nominalInput());

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.position_rank, 3);
  EXPECT_NEAR(result.position_sigma_min, 1.0, 1.0e-12);
  EXPECT_NEAR(result.position_sigma_max, 1.0, 1.0e-12);
  EXPECT_NEAR(result.position_condition_number, 1.0, 1.0e-12);
}

TEST(PositionPrimarySolver, InvalidBoundsFailWithZeroCommand)
{
  HierarchicalDIKSolver solver(unregularizedConfig());
  auto input = nominalInput();
  input.q_measured[2] = 3.0;

  const auto result = solver.solvePositionPrimary(input);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, SolverStatus::InvalidBounds);
  EXPECT_LT(result.min_hard_limit_distance_rad, -0.9);
  EXPECT_TRUE(result.qdot.isZero(0.0));
}

TEST(PositionPrimarySolver, NonfiniteInputFailsWithZeroCommand)
{
  HierarchicalDIKSolver solver(unregularizedConfig());
  auto input = nominalInput();
  input.position_error.y() = std::nan("");

  const auto result = solver.solvePositionPrimary(input);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, SolverStatus::InvalidInput);
  EXPECT_TRUE(result.qdot.isZero(0.0));
  EXPECT_TRUE(result.desired_linear_velocity.isZero(0.0));
}

TEST(PositionPrimarySolver, DeadbandSuppressesSmallCartesianNoise)
{
  auto config = unregularizedConfig();
  config.position_error_deadband = 0.005;
  HierarchicalDIKSolver solver(config);
  auto input = nominalInput();
  input.position_error << 0.003, -0.002, 0.001;

  const auto result = solver.solvePositionPrimary(input);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.desired_linear_velocity.isZero(0.0));
  EXPECT_TRUE(result.qdot.isZero(1.0e-9));
}

TEST(PositionPrimarySolver, AccelerationBoundLimitsVelocityChange)
{
  auto config = unregularizedConfig();
  config.max_joint_acceleration_rps2.setConstant(0.5);
  HierarchicalDIKSolver solver(config);
  auto input = nominalInput();
  input.position_error.setConstant(1.0);
  input.qdot_previous.setConstant(0.02);

  const auto result = solver.solvePositionPrimary(input);
  ASSERT_TRUE(result.success);
  for (int i = 0; i < kSingleArmDof; ++i) {
    EXPECT_LE(
      std::abs(result.qdot[i] - input.qdot_previous[i]),
      config.max_joint_acceleration_rps2[i] * input.dt + 1.0e-10);
  }
}

TEST(PositionPrimarySolver, SingularityScaleReducesCommand)
{
  auto config = unregularizedConfig();
  config.position_sigma_slowdown_start = 0.10;
  config.position_sigma_stop = 0.01;
  config.position_singularity_speed_scale_min = 0.0;
  HierarchicalDIKSolver solver(config);
  auto input = nominalInput();
  input.jacobian.block<3, 3>(0, 0) *= 0.055;
  input.position_error.x() = 0.10;

  const auto result = solver.solvePositionPrimary(input);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.position_speed_scale, 0.5, 1.0e-12);
  EXPECT_NEAR(result.desired_linear_velocity.x(), 0.10, 1.0e-12);
}

}  // namespace
