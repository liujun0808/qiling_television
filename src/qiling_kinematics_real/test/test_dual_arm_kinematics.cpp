#include <array>
#include <filesystem>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "qiling_kinematics/dual_arm_kinematics.hpp"

namespace qiling_kinematics
{
namespace
{

constexpr double kFiniteDifferenceStep = 1e-7;
constexpr double kJacobianTolerance = 2e-6;

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

struct NumericalJacobians
{
  ArmLinearJacobian wrist_linear{ArmLinearJacobian::Zero()};
  ArmLinearJacobian wrist_angular{ArmLinearJacobian::Zero()};
  ArmLinearJacobian elbow_linear{ArmLinearJacobian::Zero()};
};

NumericalJacobians finiteDifference(
  DualArmKinematics & kinematics, ArmSide side,
  const ArmVector & left, const ArmVector & right)
{
  NumericalJacobians result;
  for (int column = 0; column < kSingleArmDof; ++column) {
    ArmVector left_plus = left;
    ArmVector left_minus = left;
    ArmVector right_plus = right;
    ArmVector right_minus = right;
    if (side == ArmSide::Left) {
      left_plus[column] += kFiniteDifferenceStep;
      left_minus[column] -= kFiniteDifferenceStep;
    } else {
      right_plus[column] += kFiniteDifferenceStep;
      right_minus[column] -= kFiniteDifferenceStep;
    }

    const int index = side == ArmSide::Left ? 0 : 1;
    const auto plus = kinematics.evaluate(left_plus, right_plus)[index];
    const auto minus = kinematics.evaluate(left_minus, right_minus)[index];
    result.wrist_linear.col(column) =
      (plus.wrist_pose.translation() - minus.wrist_pose.translation()) /
      (2.0 * kFiniteDifferenceStep);
    result.wrist_angular.col(column) = pinocchio::log3(
      plus.wrist_pose.rotation() * minus.wrist_pose.rotation().transpose()) /
      (2.0 * kFiniteDifferenceStep);
    result.elbow_linear.col(column) =
      (plus.elbow_pose.translation() - minus.elbow_pose.translation()) /
      (2.0 * kFiniteDifferenceStep);
  }
  return result;
}

void expectJacobiansAt(
  DualArmKinematics & kinematics, const ArmVector & left, const ArmVector & right)
{
  const auto analytic = kinematics.evaluate(left, right);
  for (const ArmSide side : {ArmSide::Left, ArmSide::Right}) {
    const int index = side == ArmSide::Left ? 0 : 1;
    const NumericalJacobians numerical = finiteDifference(kinematics, side, left, right);
    EXPECT_LT(
      (analytic[index].wrist_jacobian.topRows<3>() - numerical.wrist_linear).cwiseAbs().maxCoeff(),
      kJacobianTolerance);
    EXPECT_LT(
      (analytic[index].wrist_jacobian.bottomRows<3>() - numerical.wrist_angular).cwiseAbs().maxCoeff(),
      kJacobianTolerance);
    EXPECT_LT(
      (analytic[index].elbow_linear_jacobian - numerical.elbow_linear).cwiseAbs().maxCoeff(),
      kJacobianTolerance);
  }
}

TEST(DualArmKinematicsTest, ResolvesContractFramesJointOrderAndLimits)
{
  DualArmKinematics kinematics(urdfPath());
  EXPECT_EQ(kinematics.model().nq, 14);
  EXPECT_EQ(kinematics.model().nv, 14);
  EXPECT_EQ(kinematics.jointNames(ArmSide::Left).front(), "left_shoulder_pitch_joint");
  EXPECT_EQ(kinematics.jointNames(ArmSide::Left).back(), "left_wrist_yaw_joint");
  EXPECT_EQ(kinematics.jointNames(ArmSide::Right).front(), "right_shoulder_pitch_joint");
  EXPECT_EQ(kinematics.jointNames(ArmSide::Right).back(), "right_wrist_yaw_joint");
  EXPECT_TRUE((kinematics.lowerPositionLimits(ArmSide::Left).array() <
    kinematics.upperPositionLimits(ArmSide::Left).array()).all());
  EXPECT_TRUE((kinematics.lowerPositionLimits(ArmSide::Right).array() <
    kinematics.upperPositionLimits(ArmSide::Right).array()).all());
}

TEST(DualArmKinematicsTest, ShoulderReferenceOriginsAreFixedInBaseLink)
{
  DualArmKinematics kinematics(urdfPath());
  ArmVector left = leftHome();
  ArmVector right = rightHome();
  left.array() += ArmVector(0.11, -0.09, 0.13, 0.08, -0.07, 0.05, -0.04).array();
  right.array() += ArmVector(-0.08, 0.07, -0.12, 0.09, 0.06, -0.05, 0.03).array();
  const auto result = kinematics.evaluate(left, right);
  EXPECT_TRUE(result[0].shoulder_pose.translation().isApprox(Eigen::Vector3d(0.0, 0.157, 0.455), 1e-12));
  EXPECT_TRUE(result[1].shoulder_pose.translation().isApprox(Eigen::Vector3d(0.0, -0.157, 0.455), 1e-12));
}

TEST(DualArmKinematicsTest, LocalWorldAlignedJacobiansMatchFiniteDifferenceAtHome)
{
  DualArmKinematics kinematics(urdfPath());
  expectJacobiansAt(kinematics, leftHome(), rightHome());
}

TEST(DualArmKinematicsTest, LocalWorldAlignedJacobiansMatchFiniteDifferenceAtPerturbedPose)
{
  DualArmKinematics kinematics(urdfPath());
  ArmVector left = leftHome();
  ArmVector right = rightHome();
  left += ArmVector(0.12, -0.08, 0.19, 0.11, -0.17, 0.09, -0.13);
  right += ArmVector(-0.10, 0.06, -0.16, 0.14, -0.15, -0.08, 0.12);
  expectJacobiansAt(kinematics, left, right);
}

TEST(DualArmKinematicsTest, RejectsNonFiniteConfiguration)
{
  DualArmKinematics kinematics(urdfPath());
  ArmVector invalid = leftHome();
  invalid[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(kinematics.evaluate(invalid, rightHome()), std::invalid_argument);
}

TEST(DualArmKinematicsTest, ComputesFiniteGravityFeedforwardInJointOrder)
{
  DualArmKinematics kinematics(urdfPath());
  const auto gravity = kinematics.gravityTorques(leftHome(), rightHome());
  const auto limits = kinematics.effortLimits();
  for (int side = 0; side < 2; ++side) {
    EXPECT_TRUE(gravity[side].allFinite());
    EXPECT_TRUE((limits[side].array() > 0.0).all());
    EXPECT_GT(gravity[side].cwiseAbs().maxCoeff(), 0.5);
  }
}

}  // namespace
}  // namespace qiling_kinematics
