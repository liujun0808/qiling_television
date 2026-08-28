#include <gtest/gtest.h>

#include "qiling_kinematics/arm_control_state.hpp"

namespace
{

using qiling_kinematics::ArmControlState;
using qiling_kinematics::ArmRunState;
using qiling_kinematics::ArmVector;

TEST(ArmControlState, HoldNeverIntegrates)
{
  ArmControlState state("left");
  const ArmVector measured = ArmVector::Constant(0.2);
  state.initialize(measured);

  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));

  const ArmVector qdot = ArmVector::Ones();
  const ArmVector q_min = ArmVector::Constant(-2.0);
  const ArmVector q_max = ArmVector::Constant(2.0);
  EXPECT_FALSE(state.integrateReference(qdot, 0.02, q_min, q_max));
  EXPECT_TRUE(state.reference().isApprox(measured));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));
}

TEST(ArmControlState, GripReleaseResetsReferenceAndVelocity)
{
  ArmControlState state("left");
  const ArmVector measured = ArmVector::Zero();
  state.initialize(measured);
  state.updateGripRequest(true, measured);
  ASSERT_TRUE(state.active());

  const ArmVector qdot = ArmVector::Constant(0.5);
  const ArmVector q_min = ArmVector::Constant(-2.0);
  const ArmVector q_max = ArmVector::Constant(2.0);
  ASSERT_TRUE(state.integrateReference(qdot, 0.02, q_min, q_max));
  EXPECT_FALSE(state.reference().isZero(0.0));

  const ArmVector released_measured = ArmVector::Constant(0.1);
  state.updateGripRequest(false, released_measured);
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_TRUE(state.reference().isApprox(released_measured));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));
}

TEST(ArmControlState, HoldKeepsLatchedReference)
{
  ArmControlState state("left");
  const ArmVector home = ArmVector::Constant(0.2);
  state.initialize(home);

  state.enterHold(home, false, "home reached");
  const ArmVector drooped = ArmVector::Constant(0.35);
  state.enterHold(drooped, false, "Grip released");

  EXPECT_TRUE(state.reference().isApprox(home));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));
}

TEST(ArmControlState, MeasuredStepUsesCurrentMeasurementEveryCycle)
{
  ArmControlState state("left");
  ArmVector measured = ArmVector::Constant(0.5);
  ArmVector q_min = ArmVector::Constant(-2.0);
  ArmVector q_max = ArmVector::Constant(2.0);
  ArmVector qdot = ArmVector::Constant(0.2);

  state.initialize(measured);
  state.updateGripRequest(true, measured);
  ASSERT_TRUE(state.active());

  ASSERT_TRUE(state.updateMeasuredStepReference(qdot, 0.1, measured, q_min, q_max));
  EXPECT_TRUE(state.reference().isApprox(ArmVector::Constant(0.52)));

  measured.setConstant(0.8);
  ASSERT_TRUE(state.updateMeasuredStepReference(qdot, 0.1, measured, q_min, q_max));
  EXPECT_TRUE(state.reference().isApprox(ArmVector::Constant(0.82)));
  EXPECT_TRUE(state.previousVelocity().isApprox(qdot));
}

TEST(ArmControlState, TimeoutRequiresReleaseBeforeReactivation)
{
  ArmControlState state("right");
  const ArmVector measured = ArmVector::Zero();
  state.initialize(measured);
  state.updateGripRequest(true, measured);
  ASSERT_TRUE(state.active());

  state.enterHold(measured, true, "target timeout");
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_TRUE(state.releaseRequired());

  state.updateGripRequest(true, measured);
  EXPECT_FALSE(state.active());

  state.updateGripRequest(false, measured);
  EXPECT_FALSE(state.releaseRequired());
  EXPECT_EQ(state.state(), ArmRunState::Hold);

  state.updateGripRequest(true, measured);
  EXPECT_TRUE(state.active());
}

TEST(ArmControlState, ConsecutiveFailuresLatchFault)
{
  ArmControlState state("right");
  const ArmVector measured = ArmVector::Zero();
  state.initialize(measured);
  state.updateGripRequest(true, measured);

  state.recordSolveFailure(measured, 3, "QP failure");
  EXPECT_EQ(state.state(), ArmRunState::Active);
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));

  state.recordSolveFailure(measured, 3, "QP failure");
  EXPECT_EQ(state.state(), ArmRunState::Active);

  state.recordSolveFailure(measured, 3, "QP failure");
  EXPECT_EQ(state.state(), ArmRunState::Fault);
  EXPECT_TRUE(state.releaseRequired());

  state.updateGripRequest(true, measured);
  EXPECT_EQ(state.state(), ArmRunState::Fault);

  state.updateGripRequest(false, measured);
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_FALSE(state.releaseRequired());
}

}  // namespace
