#include "simulate_bridge.hpp"

namespace {

bool IsLegJointName(const std::string & name)
{
    return name.rfind("left_hip_", 0) == 0 ||
           name.rfind("right_hip_", 0) == 0 ||
           name == "left_knee_joint" ||
           name == "right_knee_joint" ||
           name == "left_foot_pitch_joint" ||
           name == "left_foot_roll_joint" ||
           name == "right_foot_pitch_joint" ||
           name == "right_foot_roll_joint";
}

}  // namespace

/**
 * @brief 仿真器类的构造函数
 * 
 * @param d mujoco模型数据指针
 * @param m mujoco模型指针
 * @param sim 仿真UI类的引用
 */
SimulateBridge::SimulateBridge(mjData* d, mjModel* m, mujoco::Simulate& sim) : Node("mujoco_simulator_node"),mj_sim_(sim)
{
    // FIXME:解决穿模问题
    // 将模型数据指针复制进来
    mj_data_ = d;
    mj_model_ = m;

    // 修改仿真参数
    mj_model_->opt.timestep = 0.001; // 修改仿真频率为1KHz

    // 从yaml中读取参数
    const std::string pkg_path = ament_index_cpp::get_package_share_directory("mujoco_simulator"); // 获取包路径
    const std::string yaml_path = pkg_path + "/config/simulate.yaml"; // 拼接yaml路径
    YAML::Node config = YAML::LoadFile(yaml_path)["mujoco_simulator"];
    lowStateTopic_ = config["lowStateTopic"].as<std::string>();
    jointCommandsTopic_ = config["jointCommandsTopic"].as<std::string>();
    qposTopic_ = config["qposTopic"] ? config["qposTopic"].as<std::string>() : "/mujoco/qpos";
    UnPauseServiceService_ = config["unPauseService"].as<std::string>();
    initPauseFlag_ = config["initPauseFlag"].as<bool>();
    modelTableFlag_ = config["modelTableFlag"].as<bool>();
    fixedBase_ = config["fixedBase"] ? config["fixedBase"].as<bool>() : false;
    freezeLegs_ = config["freezeLegs"] ? config["freezeLegs"].as<bool>() : false;
    idleHoldKp_ = config["idleHoldKp"] ? config["idleHoldKp"].as<double>() : 40.0;
    idleHoldKd_ = config["idleHoldKd"] ? config["idleHoldKd"].as<double>() : 2.0;
    maxControlRateNmPerSec_ = config["maxControlRateNmPerSec"] ?
        config["maxControlRateNmPerSec"].as<double>() : 1000.0;
    
    // 如有设置,则暂停仿真
    if(initPauseFlag_) mj_sim_.run = 0;

    // 读取模型内容参数。home 不在 MuJoCo 内部执行，由 qiling_kinematics
    // 作为唯一的外部命令发布者执行。
    ReadModel();
    InitializeZeroActuatedState();
    // 输出相关的ID
    ShowModel();

    // 创建话题通信接口
    lowStatePub_ = this->create_publisher<mit_msgs::msg::MITLowState>(
        lowStateTopic_,
        1
    );
    jointCommandsSub_ = this->create_subscription<mit_msgs::msg::MITJointCommands>(
        jointCommandsTopic_,
        1,
        std::bind(&SimulateBridge::JointCommandSubCallBack, this, _1)
    );
    jointStatePub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states",
        1
    );
    qposPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        qposTopic_,
        1
    );
    worldFramePub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // 初始化服务通信接口
    UnPauseServer_ = this->create_service<std_srvs::srv::Empty>(
        UnPauseServiceService_,
        std::bind(&SimulateBridge::UnPauseServiceCallBack, this, _1, _2)
    );
    // 可视化机器人坐标系
    // std::string robotDescriptionString = ReadRosParam_<std::string>("robot_description", "");
    // std::cout << "robot_description: " << robotDescriptionString << std::endl;
    // urdf::Model urdfModel;
    // std::string urdfPath = "/home/coral-jyz/project/rl_dar/src/robot_description/ShenNong/urdf/whole_gazebo.urdf";
    // if (!urdfModel.initFile(urdfPath)) {
    //     RCLCPP_ERROR(this->get_logger(),"Could not read URDF from: %s",urdfPath.c_str());
    // return;
    // } else {
    //     KDL::Tree kdlTree;
    //     if (!kdl_parser::treeFromUrdfModel(urdfModel, kdlTree)) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to build KDL tree");
    //     }
    //     rclcpp::NodeOptions options;
    //     robotStatePublisherPtr_ = std::make_unique<robot_state_publisher::RobotStatePublisher>(kdlTree);
    //     robotStatePublisherPtr_->publishFixedTransforms();
    //     std::cout << "robot state published!" << std::endl;
    // }

    // 初始化通信vector长度
    lowState_.joint_states.position.assign(mj_model_->nu, 0);
    lowState_.joint_states.velocity.assign(mj_model_->nu, 0);
    lowState_.joint_states.effort.assign(mj_model_->nu, 0);
    jointCommands_.commands.resize(mj_model_->nu);
    previousControl_.assign(mj_model_->nu, 0.0);
    for(size_t i = 0; i < m->nu; i++){
        jointCommands_.commands[i].pos = 0;
        jointCommands_.commands[i].vel = 0;
        jointCommands_.commands[i].eff = 0;
        jointCommands_.commands[i].kp = 0;
        jointCommands_.commands[i].kd = 0;
      }

    frozenLegActuators_.assign(mj_model_->nu, false);
    for (int actuatorIndex = 0; actuatorIndex < mj_model_->nu; ++actuatorIndex) {
        const int jointId = mj_model_->actuator_trnid[2 * actuatorIndex];
        if (jointId >= 0 && jointId < mj_model_->njnt) {
            const char * jointName = mj_id2name(mj_model_, mjOBJ_JOINT, jointId);
            frozenLegActuators_[actuatorIndex] =
                freezeLegs_ && jointName != nullptr && IsLegJointName(jointName);
        }
    }

    ApplyKinematicLocks();
    idleHoldPosition_.assign(mj_model_->nu, 0.0);
    for (int actuatorIndex = 0; actuatorIndex < mj_model_->nu; ++actuatorIndex) {
        const int jointId = mj_model_->actuator_trnid[2 * actuatorIndex];
        if (jointId >= 0 && jointId < mj_model_->njnt) {
            const int qposAdr = mj_model_->jnt_qposadr[jointId];
            idleHoldPosition_[actuatorIndex] = mj_data_->qpos[qposAdr];
        }
    }
    RCLCPP_INFO(
        this->get_logger(),
        "第一阶段遥操仿真锁定: fixedBase=%s, frozenLegJoints=%zu, nu=%d",
        fixedBase_ ? "true" : "false",
        lockedLegJoints_.size(),
        mj_model_->nu);
    if (fixedBase_ && floatingBaseJointId_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "fixedBase=true，但模型中未找到 floating_base_joint");
    }
    if (freezeLegs_ && lockedLegJoints_.size() != 12) {
        RCLCPP_ERROR(
            this->get_logger(),
            "freezeLegs=true，但只找到 %zu 个腿部关节，期望 12 个",
            lockedLegJoints_.size());
    }
}

double SimulateBridge::LimitControlRate(size_t actuatorIndex, double desired, double dt)
{
    if (actuatorIndex >= previousControl_.size() ||
        !std::isfinite(desired) || !std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(maxControlRateNmPerSec_) || maxControlRateNmPerSec_ <= 0.0) {
        return desired;
    }
    const double maxDelta = maxControlRateNmPerSec_ * dt;
    const double limited = std::clamp(
        desired, previousControl_[actuatorIndex] - maxDelta,
        previousControl_[actuatorIndex] + maxDelta);
    previousControl_[actuatorIndex] = limited;
    return limited;
}

/**
 * @brief 电机命令接收回调
 * 
 * @param jointCommand 接收到的电机命令
 */
void SimulateBridge::JointCommandSubCallBack(const mit_msgs::msg::MITJointCommands jointCommand){

    // 如果模型不符合需求,则不执行操作
    if(modelParam_.readErrorFlag) return;
    // 收到的命令长度与模型关节数不匹配时进行处理
    if(jointCommand.commands.size() > mj_model_->nu){
        RCLCPP_ERROR(this->get_logger(), "命令长度大于模型关节数,请检查");
        return; 
    }
    else if(jointCommand.commands.size() < mj_model_->nu){
        RCLCPP_ERROR(this->get_logger(), "命令长度小于模型关节数,请检查");
        return; 
    }
    // 将命令值保存到成员变量
    jointCommands_ = jointCommand;
    externalCommandReceived_ = true;
}

/**
 * @brief 继续mujoco物理仿真的服务回调
 * 
 * @param request 
 * @param response 
 */
void SimulateBridge::UnPauseServiceCallBack(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response
) {
    // 避免未使用变量警告
    (void)request; 
    (void)response;
    // 如果初始暂停了物理仿真,则继续
    if(initPauseFlag_ && mj_sim_.run == 0) mj_sim_.run = 1;

    // 解除暂停只恢复物理运行，不重置 keyframe，也不启动内部 home。
    ApplyKinematicLocks();
}

void SimulateBridge::InitializeZeroActuatedState()
{
    // 保留 free base 的初始位置和单位四元数，仅将所有可驱动关节
    // （腿、双臂及 O6）置为零位，保证仿真启动时不会跳到 XML keyframe。
    for (int jointId = 0; jointId < mj_model_->njnt; ++jointId) {
        if (mj_model_->jnt_type[jointId] == mjJNT_FREE) {
            continue;
        }
        const int qposAdr = mj_model_->jnt_qposadr[jointId];
        const int dofAdr = mj_model_->jnt_dofadr[jointId];
        mj_data_->qpos[qposAdr] = 0.0;
        mj_data_->qvel[dofAdr] = 0.0;
    }
    for (auto & lockedJoint : lockedLegJoints_) {
        lockedJoint.qpos = 0.0;
    }
    mj_forward(mj_model_, mj_data_);
}

void SimulateBridge::ApplyKinematicLocks()
{
    if (fixedBase_ && floatingBaseJointId_ >= 0 && floatingBaseQpos_.size() == 7) {
        const int qposAdr = mj_model_->jnt_qposadr[floatingBaseJointId_];
        const int dofAdr = mj_model_->jnt_dofadr[floatingBaseJointId_];
        std::copy(floatingBaseQpos_.begin(), floatingBaseQpos_.end(), mj_data_->qpos + qposAdr);
        std::fill(mj_data_->qvel + dofAdr, mj_data_->qvel + dofAdr + 6, 0.0);
    }

    if (freezeLegs_) {
        for (const auto & lockedJoint : lockedLegJoints_) {
            const int qposAdr = mj_model_->jnt_qposadr[lockedJoint.jointId];
            const int dofAdr = mj_model_->jnt_dofadr[lockedJoint.jointId];
            mj_data_->qpos[qposAdr] = lockedJoint.qpos;
            mj_data_->qvel[dofAdr] = 0.0;
        }
    }
}

bool SimulateBridge::IsFrozenLegActuator(size_t actuatorIndex) const
{
    return actuatorIndex < frozenLegActuators_.size() &&
           frozenLegActuators_[actuatorIndex];
}

bool SimulateBridge::GetIdleHoldTarget(size_t actuatorIndex, double & position) const
{
    if (actuatorIndex >= idleHoldPosition_.size()) {
        return false;
    }
    position = idleHoldPosition_[actuatorIndex];
    return true;
}

/**
 * @brief 发送电机与IMU状态到ROS消息
 * 
 */
void SimulateBridge::LowStatePublish(){
    // 如果模型不符合需求,则不执行操作
    if(modelParam_.readErrorFlag) return;
    // 更新电机状态
    for(int i = 0; i < mj_model_->nu; i++){
        lowState_.joint_states.position[i] = mj_data_->sensordata[i + modelParam_.jointPosHeadID];
        lowState_.joint_states.velocity[i] = mj_data_->sensordata[i + modelParam_.jointVelHeadID];
        lowState_.joint_states.effort[i] = mj_data_->sensordata[i + modelParam_.jointTorHeadID];
    }
    // 更新IMU状态
    lowState_.imu.orientation.w = mj_data_->sensordata[modelParam_.imuQuatHeadID + 0];
    lowState_.imu.orientation.x = mj_data_->sensordata[modelParam_.imuQuatHeadID + 1];
    lowState_.imu.orientation.y = mj_data_->sensordata[modelParam_.imuQuatHeadID + 2];
    lowState_.imu.orientation.z = mj_data_->sensordata[modelParam_.imuQuatHeadID + 3];
    lowState_.imu.angular_velocity.x = mj_data_->sensordata[modelParam_.imuGyroHeadID + 0];
    lowState_.imu.angular_velocity.y = mj_data_->sensordata[modelParam_.imuGyroHeadID + 1];
    lowState_.imu.angular_velocity.z = mj_data_->sensordata[modelParam_.imuGyroHeadID + 2];
    lowState_.imu.linear_acceleration.x = mj_data_->sensordata[modelParam_.imuAccHeadId + 0];
    lowState_.imu.linear_acceleration.y = mj_data_->sensordata[modelParam_.imuAccHeadId + 1];
    lowState_.imu.linear_acceleration.z = mj_data_->sensordata[modelParam_.imuAccHeadId + 2];
    // 设置时间戳
    lowState_.stamp = this->get_clock()->now();
    lowState_.imu.header.stamp = this->get_clock()->now();
    // 发布状态消息
    lowStatePub_->publish(lowState_);

}

void SimulateBridge::JointStatePublish() {
    // 发布关节信息
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = this->get_clock()->now();
    jointState.name.resize(mj_model_->nu);
    jointState.position.resize(mj_model_->nu);
    for (size_t i = 0; i < mj_model_->nu; i++) {
        jointState.name[i]=modelParam_.jointName[i];
        jointState.position[i]=lowState_.joint_states.position[i];
    }
    jointStatePub_->publish(jointState);
    // 发布世界坐标信息
    // geometry_msgs::msg::TransformStamped transform;
    // transform.header.stamp = this->get_clock()->now();
    // transform.header.frame_id = "world";
    // transform.child_frame_id = "base_link";
    // transform.transform.translation.x = mj_data_->sensordata[modelParam_.realPosHeadID+0];
    // transform.transform.translation.y = mj_data_->sensordata[modelParam_.realPosHeadID+1];
    // transform.transform.translation.z = mj_data_->sensordata[modelParam_.realPosHeadID+2];
    // transform.transform.rotation = lowState_.imu.orientation;
    // worldFramePub_->sendTransform(transform);
}

/**
 * @brief 发布MuJoCo完整qpos状态,供外部渲染节点同步完整场景
 * 
 */
void SimulateBridge::QposPublish() {
    std_msgs::msg::Float64MultiArray qposMsg;
    qposMsg.data.resize(mj_model_->nq);
    for (int i = 0; i < mj_model_->nq; i++) {
        qposMsg.data[i] = mj_data_->qpos[i];
    }
    qposPub_->publish(qposMsg);
}

/**
 * @brief 输出读取到的MJCF文件信息
 * 
 */
void SimulateBridge::ShowModel(){

    using namespace tabulate;

    // 总表
    Table modelInfo;
    // 子项
    Table allJointsInfo;
    Table allLinksInfo;
    Table allSensorsInfo;
    // 子项中的表
    Table jointInfo;
    Table linkInfo;
    Table sensorInfo;

    // 总表头
    modelInfo.add_row({"model information"});
    modelInfo.add_row(RowStream{} << "model name: " + modelParam_.modelName
                                  + " time step: " + std::to_string(modelParam_.timeStep) + "s");
    modelInfo.format().font_align(FontAlign::center);
    modelInfo[1].format().hide_border_top().hide_border_bottom().font_align(FontAlign::center);


    // 输出joint信息
    allJointsInfo.add_row({"joints information"});
    allJointsInfo.format().font_align(FontAlign::center);
    allJointsInfo[0].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();
    jointInfo.add_row({"ID", "name", "posLimit(rad)", "torLimit(Nm)", "friction", "damping"});
    jointInfo[0].format().font_align(FontAlign::center).font_color(Color::yellow).hide_border_bottom();
    for(size_t i = 0; i<modelParam_.jointName.size(); i++){
        // 组织限幅字符串
        std::ostringstream posLimitStr;
        std::ostringstream torLimitStr;
        posLimitStr << std::fixed << std::setprecision(2) << std::fixed << modelParam_.jointPosRange[i].first << " ~ " << modelParam_.jointPosRange[i].second;
        torLimitStr << std::fixed << std::setprecision(2) << std::fixed << modelParam_.jointTorqueRange[i].first << " ~ " << modelParam_.jointTorqueRange[i].second;
        jointInfo.add_row(RowStream{} 
            << std::fixed << std::setprecision(2) << i << modelParam_.jointName[i] << posLimitStr.str() << torLimitStr.str() 
            << modelParam_.jointFri[i] << modelParam_.jointDamp[i]);
            jointInfo[i].format().font_align(FontAlign::center);
        if(i >= 1) jointInfo[i+1].format().hide_border_top().font_align(FontAlign::center);
    }
    allJointsInfo.add_row({jointInfo});
    allJointsInfo[1].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();

    // 输出link信息
    allLinksInfo.add_row({"links information"});
    allLinksInfo.format().font_align(FontAlign::center);
    allLinksInfo[0].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();
    linkInfo.add_row({"ID", "name", "mass(kg)"});
    linkInfo[0].format().font_align(FontAlign::center).font_color(Color::yellow).hide_border_bottom();
    for(size_t i = 0; i<modelParam_.linkNames.size(); i++){
        linkInfo.add_row(RowStream{} 
            << std::fixed << std::setprecision(2) << i << modelParam_.linkNames[i] << modelParam_.linkMass[i]);
        linkInfo[i].format().font_align(FontAlign::center);
        if(i >= 1) linkInfo[i+1].format().hide_border_top().font_align(FontAlign::center);
    }
    allLinksInfo.add_row({linkInfo});
    allLinksInfo[1].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();

    // 输出sensor信息
    allSensorsInfo.add_row({"sensors information"});
    allSensorsInfo.format().font_align(FontAlign::center);
    allSensorsInfo[0].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();
    sensorInfo.add_row({"ID", "name", "type", "attach", "head"});
    sensorInfo[0].format().font_align(FontAlign::center).font_color(Color::yellow).hide_border_bottom();
    for(size_t i = 0; i<modelParam_.sensorType.size(); i++){
        std::string headIDName;
        if(i == modelParam_.jointPosHeadID) headIDName = "joint pos head";
        else if(i == modelParam_.jointVelHeadID) headIDName = "joint vel head";
        else if(i == modelParam_.jointTorHeadID) headIDName = "joint torque head";
        else if(i == modelParam_.imuQuatHeadID) headIDName = "imu quat head";
        else if(i == modelParam_.imuGyroHeadID) headIDName = "imu gyro head";
        else if(i == modelParam_.imuAccHeadId) headIDName = "imu acc head";
        else if(i == modelParam_.realPosHeadID) headIDName = "real pos head";
        else if(i == modelParam_.realVelHeadID) headIDName = "real vel head";
        else headIDName = "";
        sensorInfo.add_row(RowStream{} 
            << i << modelParam_.sensorType[i][0] << modelParam_.sensorType[i][1] << modelParam_.sensorType[i][2] << headIDName);
        sensorInfo[i].format().font_align(FontAlign::center);
        if(i >= 1) sensorInfo[i+1].format().hide_border_top().font_align(FontAlign::center);
    }
    allSensorsInfo.add_row({sensorInfo});
    allSensorsInfo[1].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();

    // 将子表加入总表
    modelInfo.add_row({allJointsInfo});
    modelInfo.add_row({allLinksInfo});
    modelInfo.add_row({allSensorsInfo});

    // 输出模型信息
    if(modelTableFlag_){
        std::cout << "------------读取到的环境与模型信息如下------------" << std::endl;
        std::cout << modelInfo << std::endl;
        std::cout << "如果仿真遇到问题,请检查上述信息是否正确,物理仿真进行中..." << std::endl;
    }


    if(modelParam_.keyFrameCount == 0 ){ 
        RCLCPP_WARN(this->get_logger(), "未发现keyframe,请检查模型");
    }

    // 传感器错误检查
    if(modelParam_.jointPosHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现关节位置传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.jointVelHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现关节速度传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.jointTorHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现关节力矩传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.imuQuatHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现四元数传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.imuGyroHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现角速度传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.imuAccHeadId==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现线加速度传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.readErrorFlag){
        RCLCPP_ERROR(this->get_logger(), "传感器参数缺失,将不会进行ROS通信");
    }


}

/**
 * @brief 读取加载的模型信息并保存到类成员中
 * 
 */
void SimulateBridge::ReadModel(){

    // 读取加载的模型名称
    modelParam_.modelName = mj_model_->names;
    // 读取加载的模型时间步长
    modelParam_.timeStep = mj_model_->opt.timestep;
    // 仅记录模型中的 keyframe 数量用于模型信息显示；仿真启动和解除暂停
    // 均不再读取或重置 keyframe。
    modelParam_.keyFrameCount = mj_model_->nkey;

    // 保存浮动基座和腿部的 home 状态。它们在第一阶段只作为可视模型，
    // 不应因重力或双臂反作用力改变。
    floatingBaseJointId_ = -1;
    floatingBaseQpos_.clear();
    lockedLegJoints_.clear();
    for(int jointId = 0; jointId < mj_model_->njnt; ++jointId){
        const char * jointName = mj_id2name(mj_model_, mjOBJ_JOINT, jointId);
        const std::string name = jointName != nullptr ? jointName : "";
        if (mj_model_->jnt_type[jointId] == mjJNT_FREE &&
            name == "floating_base_joint") {
            floatingBaseJointId_ = jointId;
            if (fixedBase_) {
                const int qposAdr = mj_model_->jnt_qposadr[jointId];
                floatingBaseQpos_.assign(
                    mj_data_->qpos + qposAdr,
                    mj_data_->qpos + qposAdr + 7);
            }
        } else if (freezeLegs_ && IsLegJointName(name)) {
            lockedLegJoints_.push_back({
                jointId,
                mj_data_->qpos[mj_model_->jnt_qposadr[jointId]]});
        }
    }

    // 遍历所有joint,读取参数
    for(int i = 0; i<mj_model_->njnt; i++){
        if(mj_model_->jnt_type[i] == mjJNT_FREE) continue;// 注意:这里删去了free joint
        modelParam_.jointName.push_back(mj_id2name(mj_model_,mjOBJ_JOINT,i)); // 关节名称
        modelParam_.jointPosRange.push_back(std::make_pair(mj_model_->jnt_range[2*i],mj_model_->jnt_range[2*i+1])); // 关节位置限幅
        modelParam_.jointTorqueRange.push_back(std::make_pair(mj_model_->jnt_actfrcrange[2*i],mj_model_->jnt_actfrcrange[2*i+1])); // 关节速度限幅
        int joint_dofadr = mj_model_->jnt_dofadr[i];
        modelParam_.jointFri.push_back(mj_model_->dof_frictionloss[joint_dofadr]); // 关节摩擦系数
        modelParam_.jointDamp.push_back(mj_model_->dof_damping[joint_dofadr]); // 关节阻尼
    }

    // 遍历所有link,读取参数
    for(int i = 0; i<mj_model_->nbody; i++){
        if(std::string(mj_id2name(mj_model_,mjOBJ_BODY,i)) == "world")  continue;// 忽略world link
        modelParam_.linkNames.push_back(mj_id2name(mj_model_,mjOBJ_BODY,i));
        modelParam_.linkMass.push_back(mj_model_->body_mass[i]);
    }

    // 遍历所有sensor,读取参数
    for(size_t i = 0; i < static_cast<size_t>(mj_model_ -> nsensor); i++){
        // 创建临时变量
        std::string tempName;
        std::string tempType;
        std::string tempAttch;
        // 获取sensor名称
        tempName = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        // 根据不同类型的sensor进行不同的处理
        if(mj_model_->sensor_type[i] == mjSENS_JOINTPOS){ // 关节位置
            tempType = "joint pos";
            if(modelParam_.jointPosHeadID == 99999) modelParam_.jointPosHeadID = modelParam_.sensorType.size();
        }
            
        else if(mj_model_->sensor_type[i] == mjSENS_JOINTVEL) { // 关节速度
            tempType = "joint vel";
            if(modelParam_.jointVelHeadID == 99999) modelParam_.jointVelHeadID = modelParam_.sensorType.size();
        }
            
        else if(mj_model_->sensor_type[i] == mjSENS_JOINTACTFRC) { // 关节力矩
            tempType = "joint torque";
            if(modelParam_.jointTorHeadID == 99999) modelParam_.jointTorHeadID = modelParam_.sensorType.size();
        }
            
        else if(mj_model_->sensor_type[i] == mjSENS_FRAMEQUAT) { // imu四元数
            tempType = "imu quat";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.imuQuatHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_w",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_GYRO) { // imu角速度
            tempType = "imu gyro";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.imuGyroHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_ACCELEROMETER) { // imu线加速度
            tempType = "imu linear acc";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.imuAccHeadId = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_FRAMEPOS) { // 实际位置
            tempType = "real position";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.realPosHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_FRAMELINVEL) { // 实际速度
            tempType = "real velocity";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.realVelHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else tempType = "unknown";

        tempAttch = mj_id2name(mj_model_, mjOBJ_JOINT, mj_model_->sensor_objid[i]);
        modelParam_.sensorType.push_back(std::vector<std::string>{tempName,tempType,tempAttch});
    }

}

/**
 * @brief 析构函数,释放消息发布资源
 * 
 */
SimulateBridge::~SimulateBridge()
{
    lowStatePub_.reset();
}
