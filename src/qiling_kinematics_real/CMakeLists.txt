cmake_minimum_required(VERSION 3.16)
project(qiling_kinematics_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mit_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(proxsuite REQUIRED)
find_package(qi REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(qiling_dik_real_core
  src/arm_control_state.cpp
  src/anthropomorphic_elbow.cpp
  src/dual_arm_kinematics.cpp
  src/hierarchical_dik_solver.cpp
)

target_include_directories(qiling_dik_real_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(qiling_dik_real_core PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  proxsuite::proxsuite
)
target_compile_features(qiling_dik_real_core PUBLIC cxx_std_17)

find_library(PXREA_ROBOT_SDK
  NAMES PXREARobotSDK
  PATHS /opt/apps/roboticsservice/SDK/x64
  NO_DEFAULT_PATH
)
if(NOT PXREA_ROBOT_SDK)
  message(FATAL_ERROR
    "XRoboToolkit SDK not found. Expected /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so")
endif()

add_executable(differential_ik_real_node src/differential_ik_node.cpp)
add_executable(pose_target_demo src/pose_target_demo.cpp)
add_executable(xr_pose_clutch_bridge_real src/xr_pose_clutch_bridge.cpp)
add_executable(xr_pose_demo src/xr_pose_demo.cpp)
add_executable(xrobotoolkit_pxrea_adapter_real src/xrobotoolkit_pxrea_adapter.cpp)
add_executable(o6_trigger_state_node src/o6_trigger_state_node.cpp)
add_executable(o6_command_adapter_node src/o6_command_adapter_node.cpp)
add_executable(left_hand_test_node src/left_hand_test_node.cpp)

ament_target_dependencies(differential_ik_real_node
  ament_index_cpp geometry_msgs mit_msgs rclcpp sensor_msgs std_msgs)
target_link_libraries(differential_ik_real_node qiling_dik_real_core pinocchio::pinocchio)

ament_target_dependencies(pose_target_demo
  ament_index_cpp geometry_msgs rclcpp sensor_msgs std_msgs)
target_link_libraries(pose_target_demo Eigen3::Eigen pinocchio::pinocchio)

ament_target_dependencies(xr_pose_clutch_bridge_real
  geometry_msgs rclcpp sensor_msgs std_msgs)
target_link_libraries(xr_pose_clutch_bridge_real Eigen3::Eigen)

ament_target_dependencies(xr_pose_demo geometry_msgs rclcpp sensor_msgs)

ament_target_dependencies(xrobotoolkit_pxrea_adapter_real
  geometry_msgs rclcpp sensor_msgs)
target_include_directories(xrobotoolkit_pxrea_adapter_real PRIVATE
  /opt/apps/roboticsservice/SDK/include)
target_link_libraries(xrobotoolkit_pxrea_adapter_real ${PXREA_ROBOT_SDK})
set_target_properties(xrobotoolkit_pxrea_adapter_real PROPERTIES
  BUILD_RPATH "/opt/apps/roboticsservice/SDK/x64"
  INSTALL_RPATH "/opt/apps/roboticsservice/SDK/x64")

ament_target_dependencies(o6_trigger_state_node
  rclcpp sensor_msgs std_msgs)
target_compile_features(o6_trigger_state_node PUBLIC cxx_std_17)

ament_target_dependencies(o6_command_adapter_node
  qi rclcpp std_msgs)
target_compile_features(o6_command_adapter_node PUBLIC cxx_std_17)

ament_target_dependencies(left_hand_test_node
  qi rclcpp)
target_compile_features(left_hand_test_node PUBLIC cxx_std_17)

target_compile_features(differential_ik_real_node PUBLIC cxx_std_17)
target_compile_features(pose_target_demo PUBLIC cxx_std_17)
target_compile_features(xr_pose_clutch_bridge_real PUBLIC cxx_std_17)
target_compile_features(xr_pose_demo PUBLIC cxx_std_17)
target_compile_features(xrobotoolkit_pxrea_adapter_real PUBLIC cxx_std_17)

install(TARGETS
  differential_ik_real_node
  pose_target_demo
  xr_pose_clutch_bridge_real
  xr_pose_demo
  xrobotoolkit_pxrea_adapter_real
  o6_trigger_state_node
  o6_command_adapter_node
  left_hand_test_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS qiling_dik_real_core
  EXPORT export_qiling_dik_real_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config docs launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_arm_control_state test/test_arm_control_state.cpp)
  target_link_libraries(test_arm_control_state qiling_dik_real_core)

  ament_add_gtest(test_anthropomorphic_elbow test/test_anthropomorphic_elbow.cpp)
  ament_target_dependencies(test_anthropomorphic_elbow ament_index_cpp)
  target_link_libraries(test_anthropomorphic_elbow qiling_dik_real_core)

  ament_add_gtest(test_hierarchical_dik_solver test/test_hierarchical_dik_solver.cpp)
  target_link_libraries(test_hierarchical_dik_solver qiling_dik_real_core)

  ament_add_gtest(test_position_primary_solver test/test_position_primary_solver.cpp)
  target_link_libraries(test_position_primary_solver qiling_dik_real_core)

  ament_add_gtest(test_pose_hierarchy_solver test/test_pose_hierarchy_solver.cpp)
  target_link_libraries(test_pose_hierarchy_solver qiling_dik_real_core)

  ament_add_gtest(test_dual_arm_kinematics test/test_dual_arm_kinematics.cpp)
  ament_target_dependencies(test_dual_arm_kinematics ament_index_cpp)
  target_link_libraries(test_dual_arm_kinematics qiling_dik_real_core)

  ament_add_gtest(test_position_primary_kinematics test/test_position_primary_kinematics.cpp)
  ament_target_dependencies(test_position_primary_kinematics ament_index_cpp)
  target_link_libraries(test_position_primary_kinematics qiling_dik_real_core)

  ament_add_gtest(test_pose_hierarchy_kinematics test/test_pose_hierarchy_kinematics.cpp)
  ament_target_dependencies(test_pose_hierarchy_kinematics ament_index_cpp)
  target_link_libraries(test_pose_hierarchy_kinematics qiling_dik_real_core)
endif()

ament_export_targets(export_qiling_dik_real_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 pinocchio proxsuite)
ament_package()
