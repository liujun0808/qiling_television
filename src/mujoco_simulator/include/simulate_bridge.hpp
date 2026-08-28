#pragma once
#include <iostream>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "mujoco/mujoco.h"
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"
#include <tabulate/tabulate.hpp>
#include "mit_msgs/msg/mit_joint_commands.hpp"
#include "mit_msgs/msg/mit_low_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"
#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <utility>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>

/**
 * @brief 模型参数结构体
 * 
 */
struct modelParam{
    std::string modelName;
    float timeStep;
    std::vector<std::string> jointName,linkNames;
    std::vector<double> jointFri,jointDamp, linkMass;
    std::vector<std::pair<double,double>> jointPosRange,jointTorqueRange;
    std::vector<std::vector<std::string>> sensorType;
    std::vector<int> jntIdqpos, jntIdqvel, jntIddctl;
    int keyFrameCount = 0;
    size_t jointPosHeadID = 99999;
    size_t jointVelHeadID = 99999;
    size_t jointTorHeadID = 99999;
    size_t imuQuatHeadID = 99999;
    size_t imuGyroHeadID = 99999;
    size_t imuAccHeadId = 99999;
    size_t realPosHeadID = 99999;
    size_t realVelHeadID = 99999;
    bool readErrorFlag = false;

};

struct lockedJointState {
    int jointId = -1;
    double qpos = 0.0;
};


using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief mujoco与ROS2消息交互的类
 * 
 */
class SimulateBridge : public rclcpp::Node
{
private:
    // mujoco指针
    mjData* mj_data_ = nullptr;
    mjModel* mj_model_ = nullptr; 
    mujoco::Simulate& mj_sim_;
    // ROS2通信接口
    rclcpp::Publisher<mit_msgs::msg::MITLowState>::SharedPtr lowStatePub_;
    rclcpp::Subscription<mit_msgs::msg::MITJointCommands>::SharedPtr jointCommandsSub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr UnPauseServer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr qposPub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> worldFramePub_;

    mit_msgs::msg::MITJointCommands jointCommands_;
    mit_msgs::msg::MITLowState lowState_;
    // ROS2通信名称
    std::string lowStateTopic_;
    std::string jointCommandsTopic_;
    std::string qposTopic_;
    std::string UnPauseServiceService_;
    // 模型参数
    modelParam modelParam_;
    // 标志位
    bool initPauseFlag_ = false;
    bool modelTableFlag_ = true;
    bool fixedBase_ = false;
    bool freezeLegs_ = false;
    bool externalCommandReceived_ = false;
    double idleHoldKp_ = 40.0;
    double idleHoldKd_ = 2.0;
    // Safety slew limiter for the torque-like MuJoCo control signal.
    double maxControlRateNmPerSec_ = 1000.0;
    std::vector<double> previousControl_;
    std::vector<double> idleHoldPosition_;
    int floatingBaseJointId_ = -1;
    std::vector<double> floatingBaseQpos_;
    std::vector<lockedJointState> lockedLegJoints_;
    std::vector<bool> frozenLegActuators_;
    // int cmdCount_ = 0;

    template <typename T> T ReadRosParam_(const std::string& param_name, const T& default_value);
    void JointCommandSubCallBack(const mit_msgs::msg::MITJointCommands jointCommand);
    void UnPauseServiceCallBack(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void ReadModel();
    void ShowModel();
    void InitializeZeroActuatedState();
    
public:
    RCLCPP_SMART_PTR_DEFINITIONS(SimulateBridge); // 用于生成智能指针 
    void LowStatePublish();
    void JointStatePublish();
    void QposPublish();
    // 将第一阶段遥操仿真的固定基座和腿部恢复到锁定状态。
    // 该函数只修改仿真状态，不改变 /human_lower_command 消息长度。
    void ApplyKinematicLocks();
    bool IsFrozenLegActuator(size_t actuatorIndex) const;
    bool GetIdleHoldTarget(size_t actuatorIndex, double & position) const;
    bool HasExternalCommand() const { return externalCommandReceived_; }
    double LimitControlRate(size_t actuatorIndex, double desired, double dt);
    double IdleHoldKp() const { return idleHoldKp_; }
    double IdleHoldKd() const { return idleHoldKd_; }

    SimulateBridge(mjData* d, mjModel* m, mujoco::Simulate& sim);
    ~SimulateBridge();

    mit_msgs::msg::MITJointCommands GetJointCommands() {return jointCommands_;};
    int GetSimRun() {return mj_sim_.run;};
};

/**
 * @brief 统一读取ros参数的函数
 * 
 * @tparam T 读取参数的类型
 * @param param_name 参数的名称
 * @param default_value 参数默认值
 * @return T 读取到参数的值
 */
template <typename T>
T SimulateBridge::ReadRosParam_(const std::string& param_name, const T& default_value) 
{
    T value;
    
    // 声明参数并设置默认值
    this->declare_parameter<T>(param_name, default_value);
    
    // 尝试获取参数并进行参数检查
    if (!this->get_parameter(param_name, value)) {
        RCLCPP_ERROR(this->get_logger(), "参数 [%s] 未找到，使用默认值", param_name.c_str());
        return default_value;
    }
    
    return value;
}
