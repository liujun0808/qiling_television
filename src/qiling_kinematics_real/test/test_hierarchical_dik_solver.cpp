#include <gtest/gtest.h>

#include <cmath>

#include "qiling_kinematics/hierarchical_dik_solver.hpp"

namespace
{

using qiling_kinematics::ArmVector;
using qiling_kinematics::HierarchicalDIKSolver;
using qiling_kinematics::SolverStatus;

HierarchicalDIKSolver::Input nominalInput()
{
  HierarchicalDIKSolver::Input input;
  input.jacobian.setZero();
  input.jacobian.block<6, 6>(0, 0).setIdentity();
  input.pose_error.setZero();
  input.q_measured.setZero();
  input.q_reference.setZero();
  input.q_nominal.setZero();
  input.q_min.setConstant(-2.0);
  input.q_max.setConstant(2.0);
  input.dt = 0.02;
  return input;
}

TEST(HierarchicalDIKSolver, ZeroErrorProducesFiniteZeroVelocity)
{
  HierarchicalDIKSolver solver(HierarchicalDIKSolver::Config{});
  const auto result = solver.solveLegacyFullPose(nominalInput());

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.status, SolverStatus::Solved);
  EXPECT_TRUE(result.qdot.allFinite());
  EXPECT_TRUE(result.qdot.isZero(1.0e-10));
}

TEST(HierarchicalDIKSolver, NonzeroErrorProducesBoundedVelocity)
{
  HierarchicalDIKSolver::Config config;
  config.max_joint_velocity = 0.5;
  HierarchicalDIKSolver solver(config);

  auto input = nominalInput();
  input.pose_error[0] = 0.10;
  input.pose_error[4] = 0.20;
  const auto result = solver.solveLegacyFullPose(input);

  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.qdot.allFinite());
  EXPECT_LE(result.qdot.cwiseAbs().maxCoeff(), config.max_joint_velocity + 1.0e-8);
  EXPECT_GT(result.qdot.norm(), 0.0);
}

TEST(HierarchicalDIKSolver, InvalidDtReturnsZeroFailure)
{
  HierarchicalDIKSolver solver(HierarchicalDIKSolver::Config{});
  auto input = nominalInput();
  input.dt = 0.0;

  const auto result = solver.solveLegacyFullPose(input);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, SolverStatus::InvalidInput);
  EXPECT_TRUE(result.qdot.isZero(0.0));
}

TEST(HierarchicalDIKSolver, InvalidBoundsReturnsZeroFailure)
{
  HierarchicalDIKSolver solver(HierarchicalDIKSolver::Config{});
  auto input = nominalInput();
  input.q_measured[0] = 3.0;

  const auto result = solver.solveLegacyFullPose(input);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, SolverStatus::InvalidBounds);
  EXPECT_TRUE(result.qdot.isZero(0.0));
}

TEST(HierarchicalDIKSolver, NonfiniteInputReturnsZeroFailure)
{
  HierarchicalDIKSolver solver(HierarchicalDIKSolver::Config{});
  auto input = nominalInput();
  input.pose_error[2] = std::nan("");

  const auto result = solver.solveLegacyFullPose(input);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, SolverStatus::InvalidInput);
  EXPECT_TRUE(result.qdot.isZero(0.0));
}

}  // namespace

