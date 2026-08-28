#include <array>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>

#include "qiling_kinematics/dual_arm_kinematics.hpp"

namespace qiling_kinematics
{
namespace
{

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

std::filesystem::path descriptionPath(const std::filesystem::path & relative)
{
  return std::filesystem::path(
    ament_index_cpp::get_package_share_directory("qi_robot_description")) / relative;
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

Eigen::Vector3d bodyPosition(const mjData & data, int body_id)
{
  return Eigen::Map<const Eigen::Vector3d>(data.xpos + 3 * body_id);
}

Eigen::Matrix3d bodyRotation(const mjData & data, int body_id)
{
  return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
    data.xmat + 9 * body_id);
}

int requireId(const mjModel & model, mjtObj type, const std::string & name)
{
  const int id = mj_name2id(&model, type, name.c_str());
  if (id < 0) {
    throw std::runtime_error("MuJoCo object missing: " + name);
  }
  return id;
}

class ModelAgreementTest : public ::testing::Test
{
protected:
  ModelAgreementTest()
  : pinocchio_(descriptionPath("urdf/s4_dual_arm.urdf")),
    model_(nullptr, mj_deleteModel), data_(nullptr, mj_deleteData)
  {
    std::array<char, 2048> error{};
    const std::string scene = descriptionPath(
      "new_scene/scene_S4_40DOF_fullbody.xml").string();
    model_.reset(mj_loadXML(scene.c_str(), nullptr, error.data(), error.size()));
    if (!model_) {
      throw std::runtime_error("failed to load MuJoCo scene: " + std::string(error.data()));
    }
    data_.reset(mj_makeData(model_.get()));
    if (!data_) {
      throw std::runtime_error("failed to allocate MuJoCo data");
    }
  }

  void setArm(const std::array<std::string, kSingleArmDof> & names, const ArmVector & q)
  {
    for (int column = 0; column < kSingleArmDof; ++column) {
      const int joint_id = requireId(*model_, mjOBJ_JOINT, names[column]);
      data_->qpos[model_->jnt_qposadr[joint_id]] = q[column];
    }
  }

  void expectFrameAgreement(
    const char * body_name, const pinocchio::SE3 & pinocchio_pose,
    double position_tolerance = 3e-6, double angle_tolerance = 1e-5)
  {
    const int base_id = requireId(*model_, mjOBJ_BODY, "base_link");
    const int body_id = requireId(*model_, mjOBJ_BODY, body_name);
    const Eigen::Vector3d base_position = bodyPosition(*data_, base_id);
    const Eigen::Matrix3d base_rotation = bodyRotation(*data_, base_id);
    const Eigen::Vector3d position_in_base =
      base_rotation.transpose() * (bodyPosition(*data_, body_id) - base_position);
    const Eigen::Matrix3d rotation_in_base =
      base_rotation.transpose() * bodyRotation(*data_, body_id);

    EXPECT_LT((position_in_base - pinocchio_pose.translation()).norm(), position_tolerance)
      << body_name;
    EXPECT_LT(
      Eigen::AngleAxisd(pinocchio_pose.rotation().transpose() * rotation_in_base).angle(),
      angle_tolerance) << body_name;
  }

  void expectAgreementAt(const ArmVector & left, const ArmVector & right)
  {
    const int key_id = requireId(*model_, mjOBJ_KEY, "teleop_home");
    mj_resetDataKeyframe(model_.get(), data_.get(), key_id);
    setArm(pinocchio_.jointNames(ArmSide::Left), left);
    setArm(pinocchio_.jointNames(ArmSide::Right), right);
    mj_forward(model_.get(), data_.get());

    const auto pin = pinocchio_.evaluate(left, right);
    expectFrameAgreement("left_elbow_link", pin[0].elbow_pose);
    expectFrameAgreement("LH_hand_base_link", pin[0].wrist_pose);
    expectFrameAgreement("right_elbow_link", pin[1].elbow_pose);
    expectFrameAgreement("RH_hand_base_link", pin[1].wrist_pose);
  }

  DualArmKinematics pinocchio_;
  ModelPtr model_;
  DataPtr data_;
};

TEST_F(ModelAgreementTest, WristAndElbowFramesAgreeAtTeleopHome)
{
  expectAgreementAt(leftHome(), rightHome());
}

TEST_F(ModelAgreementTest, WristAndElbowFramesAgreeAtPerturbedConfiguration)
{
  const ArmVector left = leftHome() +
    ArmVector(0.12, -0.08, 0.19, 0.11, -0.17, 0.09, -0.13);
  const ArmVector right = rightHome() +
    ArmVector(-0.10, 0.06, -0.16, 0.14, -0.15, -0.08, 0.12);
  expectAgreementAt(left, right);
}

}  // namespace
}  // namespace qiling_kinematics
