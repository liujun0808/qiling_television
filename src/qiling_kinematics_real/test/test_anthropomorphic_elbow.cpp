#include <cmath>
#include <array>
#include <filesystem>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "qiling_kinematics/anthropomorphic_elbow.hpp"
#include "qiling_kinematics/dual_arm_kinematics.hpp"

namespace
{

using qiling_kinematics::AnthropomorphicElbow;
using qiling_kinematics::AnthropomorphicElbowConfig;
using qiling_kinematics::ArmSide;
using qiling_kinematics::ArmLinearJacobian;
using qiling_kinematics::ArmVector;
using qiling_kinematics::computeElbowSwivelJacobian;
using qiling_kinematics::DualArmKinematics;

AnthropomorphicElbowConfig geometryConfig()
{
  AnthropomorphicElbowConfig config;
  config.outward_weight = 1.0;
  config.downward_weight = 0.30;
  config.home_weight = 0.80;
  config.min_shoulder_wrist_distance = 0.05;
  config.min_elbow_projection_radius = 0.02;
  return config;
}

Eigen::Vector3d homeDirection(
  const Eigen::Vector3d & shoulder,
  const Eigen::Vector3d & elbow,
  const Eigen::Vector3d & wrist)
{
  const Eigen::Vector3d axis = (wrist - shoulder).normalized();
  Eigen::Vector3d direction =
    (Eigen::Matrix3d::Identity() - axis * axis.transpose()) * (elbow - shoulder);
  direction.normalize();
  return direction;
}

TEST(AnthropomorphicElbow, UsesOppositeBaseYOutwardDirections)
{
  AnthropomorphicElbow elbow;
  const auto config = geometryConfig();
  const Eigen::Vector3d shoulder(0.0, 0.0, 0.0);
  const Eigen::Vector3d wrist(0.40, 0.0, 0.20);

  const auto left = elbow.compute(
    ArmSide::Left, shoulder, Eigen::Vector3d(0.20, 0.20, 0.0), wrist,
    Eigen::Vector3d(0.0, 1.0, 0.0), config);
  const auto right = elbow.compute(
    ArmSide::Right, shoulder, Eigen::Vector3d(0.20, -0.20, 0.0), wrist,
    Eigen::Vector3d(0.0, -1.0, 0.0), config);

  ASSERT_TRUE(left.valid);
  ASSERT_TRUE(right.valid);
  EXPECT_GT(left.preferred_direction.y(), 0.5);
  EXPECT_LT(right.preferred_direction.y(), -0.5);
  EXPECT_NEAR(left.current_projection_radius, 0.2190890230, 1.0e-9);
  EXPECT_NEAR(right.current_projection_radius, 0.2190890230, 1.0e-9);
}

TEST(AnthropomorphicElbow, FallsBackToPreviousThenHomeWithoutRandomDirection)
{
  AnthropomorphicElbow elbow;
  auto config = geometryConfig();
  config.outward_weight = 1.0;
  config.downward_weight = 0.0;
  config.home_weight = 0.0;
  const Eigen::Vector3d shoulder(0.0, 0.0, 0.0);
  const Eigen::Vector3d wrist(0.0, 0.40, 0.0);
  const Eigen::Vector3d current_elbow(0.20, 0.20, 0.0);

  const auto first = elbow.compute(
    ArmSide::Left, shoulder, current_elbow, wrist,
    Eigen::Vector3d(1.0, 0.0, 0.0), config);
  ASSERT_TRUE(first.valid);
  EXPECT_TRUE(first.used_home_direction);

  config.home_weight = 0.0;
  const auto second = elbow.compute(
    ArmSide::Left, shoulder, current_elbow, wrist,
    Eigen::Vector3d(0.0, 1.0, 0.0), config);
  ASSERT_TRUE(second.valid);
  EXPECT_TRUE(second.used_previous_direction);
  EXPECT_GT(second.preferred_direction.dot(first.preferred_direction), 0.99);
}

TEST(AnthropomorphicElbow, RejectsDegenerateShoulderWristAxis)
{
  AnthropomorphicElbow elbow;
  const auto result = elbow.compute(
    ArmSide::Left,
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.1, 0.1, 0.0),
    Eigen::Vector3d(0.01, 0.0, 0.0),
    Eigen::Vector3d(0.0, 1.0, 0.0), geometryConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_LT(result.shoulder_wrist_distance, 0.05);
}

TEST(AnthropomorphicElbow, RejectsDegenerateElbowProjectionRadius)
{
  AnthropomorphicElbow elbow;
  const auto result = elbow.compute(
    ArmSide::Left,
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.20, 0.0, 0.0),
    Eigen::Vector3d(0.40, 0.0, 0.0),
    Eigen::Vector3d(0.0, 1.0, 0.0), geometryConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.current_direction_valid);
  EXPECT_LT(result.current_projection_radius, 0.02);
}

TEST(AnthropomorphicElbow, PreferredDirectionDoesNotFlipAcrossProjectedOutwardDegeneracy)
{
  AnthropomorphicElbow elbow;
  auto config = geometryConfig();
  config.downward_weight = 0.0;
  config.home_weight = 0.0;
  const Eigen::Vector3d shoulder = Eigen::Vector3d::Zero();
  const Eigen::Vector3d current_elbow(0.20, 0.0, 0.10);
  Eigen::Vector3d previous = Eigen::Vector3d::Zero();

  for (int sample = -8; sample <= 8; ++sample) {
    const double angle = 0.1 * static_cast<double>(sample);
    const Eigen::Vector3d axis(std::sin(angle), std::cos(angle), 0.0);
    const auto output = elbow.compute(
      ArmSide::Left, shoulder, current_elbow, axis,
      Eigen::Vector3d(0.0, 0.0, 1.0), config);
    ASSERT_TRUE(output.valid) << "sample=" << sample;
    if (sample != -8) {
      EXPECT_GT(output.preferred_direction.dot(previous), 0.0)
        << "sample=" << sample;
    }
    previous = output.preferred_direction;
  }
}

TEST(AnthropomorphicElbow, S4HomeDirectionsAreFiniteAndOutwardBiased)
{
  const auto share = ament_index_cpp::get_package_share_directory("qi_robot_description");
  DualArmKinematics kinematics(
    std::filesystem::path(share) / "urdf" / "s4_dual_arm.urdf");
  ArmVector left;
  ArmVector right;
  left << -0.45, 0.30, 0.02, -1.40, 1.40, 0.0, 0.0;
  right << -0.45, -0.30, 0.02, -1.40, 1.40, 0.0, 0.0;
  const auto state = kinematics.evaluate(left, right);
  AnthropomorphicElbow elbow;
  const auto config = geometryConfig();

  for (const ArmSide side : {ArmSide::Left, ArmSide::Right}) {
    const int index = side == ArmSide::Left ? 0 : 1;
    const auto & arm = state[index];
    const auto home = homeDirection(
      arm.shoulder_pose.translation(), arm.elbow_pose.translation(),
      arm.wrist_pose.translation());
    const auto output = elbow.compute(
      side, arm.shoulder_pose.translation(), arm.elbow_pose.translation(),
      arm.wrist_pose.translation(), home, config);
    ASSERT_TRUE(output.valid) << "side=" << index;
    EXPECT_TRUE(output.current_direction_valid);
    EXPECT_TRUE(output.preferred_direction.allFinite());
    EXPECT_GT(
      output.preferred_direction.dot(
        Eigen::Vector3d(0.0, side == ArmSide::Left ? 1.0 : -1.0, 0.0)), 0.0)
      << "side=" << index;
    EXPECT_TRUE(std::isfinite(output.signed_swivel_error));
  }
}

TEST(AnthropomorphicElbow, S4PerturbedConfigurationsKeepDirectionContinuous)
{
  const auto share = ament_index_cpp::get_package_share_directory("qi_robot_description");
  DualArmKinematics kinematics(
    std::filesystem::path(share) / "urdf" / "s4_dual_arm.urdf");
  ArmVector left_home;
  ArmVector right_home;
  left_home << -0.45, 0.30, 0.02, -1.40, 1.40, 0.0, 0.0;
  right_home << -0.45, -0.30, 0.02, -1.40, 1.40, 0.0, 0.0;
  const auto home_state = kinematics.evaluate(left_home, right_home);
  std::array<Eigen::Vector3d, 2> home_directions{};
  for (int index = 0; index < 2; ++index) {
    home_directions[index] = homeDirection(
      home_state[index].shoulder_pose.translation(),
      home_state[index].elbow_pose.translation(),
      home_state[index].wrist_pose.translation());
  }

  AnthropomorphicElbowConfig config = geometryConfig();
  for (const ArmSide side : {ArmSide::Left, ArmSide::Right}) {
    const int index = side == ArmSide::Left ? 0 : 1;
    AnthropomorphicElbow elbow;
    Eigen::Vector3d previous = Eigen::Vector3d::Zero();
    for (int sample = 0; sample < 80; ++sample) {
      const double phase = 0.08 * static_cast<double>(sample);
      ArmVector left = left_home;
      ArmVector right = right_home;
      ArmVector & q = index == 0 ? left : right;
      q[0] += 0.12 * std::sin(phase);
      q[1] += 0.10 * std::cos(phase);
      q[2] += 0.08 * std::sin(0.7 * phase);
      q[3] += 0.10 * std::cos(0.5 * phase);
      const auto state = kinematics.evaluate(left, right);
      const auto output = elbow.compute(
        side, state[index].shoulder_pose.translation(),
        state[index].elbow_pose.translation(), state[index].wrist_pose.translation(),
        home_directions[index], config);
      ASSERT_TRUE(output.valid) << "side=" << index << " sample=" << sample;
      if (sample > 0) {
        EXPECT_GT(output.preferred_direction.dot(previous), 0.0)
          << "side=" << index << " sample=" << sample;
      }
      EXPECT_GT(
        output.preferred_direction.dot(
          Eigen::Vector3d(0.0, side == ArmSide::Left ? 1.0 : -1.0, 0.0)), 0.0)
        << "side=" << index << " sample=" << sample;
      previous = output.preferred_direction;
    }
  }
}

TEST(AnthropomorphicElbow, SwivelJacobianUsesElbowMotionAroundShoulderWristAxis)
{
  ArmLinearJacobian elbow_jacobian = ArmLinearJacobian::Zero();
  ArmLinearJacobian wrist_jacobian = ArmLinearJacobian::Zero();
  elbow_jacobian(2, 0) = 1.0;

  const auto jacobian = computeElbowSwivelJacobian(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0.5, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0), elbow_jacobian, wrist_jacobian,
    Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0), 0.5);

  EXPECT_NEAR(jacobian[0], 2.0, 1.0e-12);
  for (int i = 1; i < 7; ++i) {
    EXPECT_NEAR(jacobian[i], 0.0, 1.0e-12);
  }
}

}  // namespace
