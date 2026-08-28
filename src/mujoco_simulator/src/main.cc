// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

#include "simulate_bridge.hpp"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#include <sys/errno.h>
#include <unistd.h>
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

std::string ResolveModelPath(const std::string& path_text) {
  constexpr const char* kPackagePrefix = "package://";
  if (path_text.rfind(kPackagePrefix, 0) == 0) {
    const std::string relative = path_text.substr(std::strlen(kPackagePrefix));
    const std::size_t split_pos = relative.find('/');
    if (split_pos == std::string::npos || split_pos == 0 ||
        split_pos + 1 >= relative.size()) {
      throw std::runtime_error("非法 package 模型路径: " + path_text);
    }

    const std::string package_name = relative.substr(0, split_pos);
    const std::string package_path = relative.substr(split_pos + 1);
    return (std::filesystem::path(
      ament_index_cpp::get_package_share_directory(package_name)) /
      package_path).string();
  }

  const std::filesystem::path path(path_text);
  if (path.is_absolute()) {
    return path.string();
  }
  return (std::filesystem::current_path() / path).string();
}

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

// 声明全局类指针,此时还没有初始化
SimulateBridge::SharedPtr SimulateBridgeNodePtr = nullptr;


using Seconds = std::chrono::duration<double>;


//------------------------------------------- simulation -------------------------------------------

const char* Diverged(int disableflags, const mjData* d) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (d->warning[w].number > 0) {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  if (!mnew) {
    std::printf("%s\n", loadError);
    mju::strcpy_arr(sim.load_error, loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  // if no error and load took more than 1/4 seconds, report load time
  else if (load_seconds > 0.25) {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  return mnew;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;

        // 给ros2节点指针赋值
        SimulateBridgeNodePtr = nullptr; // 先使资源释放
        SimulateBridgeNodePtr = std::make_shared<SimulateBridge>(d, m, sim);
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        // 给ros2节点指针赋值
        SimulateBridgeNodePtr = nullptr;
        SimulateBridgeNodePtr = std::make_shared<SimulateBridge>(d, m, sim);
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned =
              std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // run single step, let next iteration deal with timing
            mj_step(m, d);

            const char* message = Diverged(m->opt.disableflags, d);
            if (message) {
              sim.run = 0;
              mju::strcpy_arr(sim.load_error, message);
            } else {
              stepped = true;
            }
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

              // inject noise
              sim.InjectNoise();

              // call mj_step
              mj_step(m, d);

              const char* message = Diverged(m->opt.disableflags, d);
              if (message) {
                sim.run = 0;
                mju::strcpy_arr(sim.load_error, message);
              } else {
                stepped = true;
              }

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            // 第一阶段遥操模式下，固定 base_link 并冻结腿部；腿部命令不会
            // 进入控制回调，但 40 维命令接口仍保持不变。
            SimulateBridgeNodePtr->ApplyKinematicLocks();
            mj_forward(m, d);
            sim.AddToHistory();
            SimulateBridgeNodePtr->LowStatePublish();
            SimulateBridgeNodePtr->JointStatePublish();
            SimulateBridgeNodePtr->QposPublish();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          SimulateBridgeNodePtr->ApplyKinematicLocks();
          mj_forward(m, d);
          SimulateBridgeNodePtr->LowStatePublish();
          SimulateBridgeNodePtr->JointStatePublish();
          SimulateBridgeNodePtr->QposPublish();
          sim.speed_changed = true;
        }
        // 执行ros2回调以执行unPause回调
        rclcpp::spin_some(SimulateBridgeNodePtr);
      } 
    }  // release std::lock_guard<std::mutex>
    
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      // 给ros2节点指针赋值
      SimulateBridgeNodePtr = std::make_shared<SimulateBridge>(d, m, *sim);
      mj_forward(m, d);

    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  mj_deleteData(d);
  mj_deleteModel(m);
}

// 控制回调函数
void PDControllerCallBack(const mjModel* m, mjData* d){
  // 如果没有实例化类,就返回
  if(SimulateBridgeNodePtr == nullptr) return;
  // 暂停的时候不执行控制
  if(SimulateBridgeNodePtr->GetSimRun() == 0) return;
  // 执行ros2回调获取命令值
  rclcpp::spin_some(SimulateBridgeNodePtr);
  // 获取命令
  mit_msgs::msg::MITJointCommands cmd = SimulateBridgeNodePtr->GetJointCommands();
  // 运行PD控制器
  for(size_t i = 0; i < m->nu; i++){
    // 腿部仅用于显示，不接受外部 MIT 命令。随后由
    // ApplyKinematicLocks() 将其 qpos/qvel 恢复到 home 状态。
    if (SimulateBridgeNodePtr->IsFrozenLegActuator(i)) {
      d->ctrl[i] = 0.0;
    } else {
      double idleHoldPosition = 0.0;
      if (!SimulateBridgeNodePtr->HasExternalCommand() &&
          SimulateBridgeNodePtr->GetIdleHoldTarget(i, idleHoldPosition)) {
        // 在 qiling_kinematics 启动前保持全零姿态，避免仅解除暂停就因
        // 重力导致双臂下落。该保持不是 home 流程，收到首个外部命令后
        // 立即交由 qiling_kinematics 的 home 状态机控制。
        d->ctrl[i] = SimulateBridgeNodePtr->IdleHoldKp() *
              (idleHoldPosition - d->sensordata[i + 0 * m->nu])
            - SimulateBridgeNodePtr->IdleHoldKd() *
              d->sensordata[i + 1 * m->nu];
      } else if (i < cmd.commands.size()) {
      d->ctrl[i] = cmd.commands[i].kp * (cmd.commands[i].pos - d->sensordata[i + 0 * m->nu])
                  + cmd.commands[i].kd * (cmd.commands[i].vel - d->sensordata[i + 1 * m->nu])
                  + cmd.commands[i].eff;
      } else {
        d->ctrl[i] = 0.0;
      }
    }
    // 处理掉异常值
    if(isnan(d->ctrl[i])) d->ctrl[i] = 0;
    // Keep the user-facing MIT/PD layer from producing an unbounded motor
    // command. MuJoCo also enforces ctrlrange internally, but clamping here
    // makes the value shown in the control panel and the applied value agree.
    if (m->actuator_ctrllimited[i]) {
      d->ctrl[i] = std::clamp(
        d->ctrl[i], m->actuator_ctrlrange[2 * i], m->actuator_ctrlrange[2 * i + 1]);
    }
    // Limit abrupt changes in the torque-like actuator command. This is a
    // simulator-side safeguard for the same kind of command slew limiter
    // that must exist in the real MIT command path.
    d->ctrl[i] = SimulateBridgeNodePtr->LimitControlRate(
      i, d->ctrl[i], m->opt.timestep);
  }
}

//------------------------------------------ main --------------------------------------------------

// run event loop
int main(int argc, char** argv) {

  // 初始化ros2上下文
  rclcpp::init(argc, argv);
  // 声明一个临时节点输出消息
  auto tempNode = rclcpp::Node::make_shared("mujoco_simulator_node");

  // print version, check compatibility
  RCLCPP_INFO(tempNode->get_logger(),"MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    RCLCPP_ERROR(tempNode->get_logger(),"Headers and library have different versions");
  }
  // 释放临时节点
  tempNode = nullptr;

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );


  // 读取yaml路径
  const std::string mujoco_pkg_path = ament_index_cpp::get_package_share_directory("mujoco_simulator"); // 获取包路径
  const std::string yaml_path = mujoco_pkg_path + "/config/simulate.yaml"; // 拼接yaml路径
  YAML::Node config = YAML::LoadFile(yaml_path)["mujoco_simulator"];

  // 从 YAML 读取 MuJoCo 场景路径。支持 package:// URI 和绝对路径。
  if (!config["modelPath"]) {
    throw std::runtime_error(
      "simulate.yaml 缺少 mujoco_simulator.modelPath，"
      "应指向 qi_robot_description/new_scene/scene_S4_40DOF_fullbody.xml");
  }
  const std::string mjcf_path = ResolveModelPath(
    config["modelPath"].as<std::string>());
  if (!std::filesystem::exists(mjcf_path)) {
    throw std::runtime_error("MuJoCo模型文件不存在: " + mjcf_path);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("mujoco_simulator_node"),
    "加载MuJoCo模型: %s", mjcf_path.c_str());


  // 设置控制回调函数
  mjcb_control = PDControllerCallBack; 

  // 启动物理仿真线程
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), mjcf_path.c_str());


  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  // 释放资源
  rclcpp::shutdown();
  pthread_exit(NULL);
  return 0;
}
