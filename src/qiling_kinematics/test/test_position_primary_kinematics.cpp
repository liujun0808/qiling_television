#include <filesystem>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "qiling_kinematics/dual_arm_kinematics.hpp"
#include "qiling_kinematics/hierarchical_dik_solver.hpp"

namespace
{

using qiling_kinematics::ArmSide;
using qiling_kinematics::ArmVector;
using qiling_kinematics::DualArmKinematics;
using qiling_kinematics::HierarchicalDIKSolver;

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

TEST(PositionPrimaryKinematics, BothArmsTrackEveryBaseLinkTranslationAxis)
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
  config.max_joint_velocity_rps.setConstant(2.0);

  for (const ArmSide side : {ArmSide::Left, ArmSide::Right}) {
    const int index = side == ArmSide::Left ? 0 : 1;
    for (int axis = 0; axis < 3; ++axis) {
      HierarchicalDIKSolver solver(config);
      HierarchicalDIKSolver::PositionInput input;
      input.jacobian = state[index].wrist_jacobian.topRows<3>();
      input.position_error[axis] = 0.02;
      input.q_measured = index == 0 ? left : right;
      input.q_min = kinematics.lowerPositionLimits(side);
      input.q_max = kinematics.upperPositionLimits(side);
      input.dt = 0.02;

      const auto result = solver.solvePositionPrimary(input);
      ASSERT_TRUE(result.success) << "side=" << index << " axis=" << axis;
      EXPECT_GT(result.achieved_linear_velocity[axis], 0.055)
        << "side=" << index << " axis=" << axis;
      Eigen::Vector3d cross_axis = result.achieved_linear_velocity;
      cross_axis[axis] = 0.0;
      EXPECT_LT(cross_axis.norm(), 2.0e-5)
        << "side=" << index << " axis=" << axis;
    }
  }
}

}  // namespace
