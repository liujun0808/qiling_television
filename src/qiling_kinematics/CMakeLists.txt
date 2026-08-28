cmake_minimum_required(VERSION 3.16)
project(qiling_kinematics)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(qiling_dik_core
  src/arm_control_state.cpp
  src/anthropomorphic_elbow.cpp
  src/dual_arm_kinematics.cpp
  src/hierarchical_dik_solver.cpp
)

target_include_directories(qiling_dik_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(qiling_dik_core PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  proxsuite::proxsuite
)

target_compile_features(qiling_dik_core PUBLIC cxx_std_17)

find_library(PXREA_ROBOT_SDK
  NAMES PXREARobotSDK
  PATHS /opt/apps/roboticsservice/SDK/x64
  NO_DEFAULT_PATH
)
if(NOT PXREA_ROBOT_SDK)
  message(FATAL_ERROR
    "XRoboToolkit SDK not found. Expected /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so")
endif()

add_executable(differential_ik_node
  src/differential_ik_node.cpp
)

add_executable(pose_target_demo
  src/pose_target_demo.cpp
)

add_executable(xr_pose_clutch_bridge
  src/xr_pose_clutch_bridge.cpp
)

add_executable(xr_pose_demo
  src/xr_pose_demo.cpp
)

add_executable(xrobotoolkit_pxrea_adapter
  src/xrobotoolkit_pxrea_adapter.cpp
)

ament_target_dependencies(differential_ik_node
  ament_index_cpp
  geometry_msgs
  mit_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(differential_ik_node
  qiling_dik_core
  pinocchio::pinocchio
)

ament_target_dependencies(pose_target_demo
  ament_index_cpp
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(xr_pose_clutch_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(xr_pose_demo
  geometry_msgs
  rclcpp
  sensor_msgs
)

ament_target_dependencies(xrobotoolkit_pxrea_adapter
  geometry_msgs
  rclcpp
  sensor_msgs
)

target_link_libraries(pose_target_demo
  Eigen3::Eigen
  pinocchio::pinocchio
)

target_link_libraries(xr_pose_clutch_bridge
  Eigen3::Eigen
)

target_compile_features(differential_ik_node PUBLIC cxx_std_17)
target_compile_features(pose_target_demo PUBLIC cxx_std_17)
target_compile_features(xr_pose_clutch_bridge PUBLIC cxx_std_17)
target_compile_features(xr_pose_demo PUBLIC cxx_std_17)
target_compile_features(xrobotoolkit_pxrea_adapter PUBLIC cxx_std_17)

target_include_directories(xrobotoolkit_pxrea_adapter PRIVATE
  /opt/apps/roboticsservice/SDK/include
)

target_link_libraries(xrobotoolkit_pxrea_adapter
  ${PXREA_ROBOT_SDK}
)

set_target_properties(xrobotoolkit_pxrea_adapter PROPERTIES
  BUILD_RPATH "/opt/apps/roboticsservice/SDK/x64"
  INSTALL_RPATH "/opt/apps/roboticsservice/SDK/x64"
)

install(TARGETS differential_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS qiling_dik_core
  EXPORT export_qiling_dik_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pose_target_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS xr_pose_clutch_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS xr_pose_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS xrobotoolkit_pxrea_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_arm_control_state
    test/test_arm_control_state.cpp
  )
  target_link_libraries(test_arm_control_state qiling_dik_core)

  ament_add_gtest(test_anthropomorphic_elbow
    test/test_anthropomorphic_elbow.cpp
  )
  ament_target_dependencies(test_anthropomorphic_elbow ament_index_cpp)
  target_link_libraries(test_anthropomorphic_elbow qiling_dik_core)

  ament_add_gtest(test_hierarchical_dik_solver
    test/test_hierarchical_dik_solver.cpp
  )
  target_link_libraries(test_hierarchical_dik_solver qiling_dik_core)

  ament_add_gtest(test_position_primary_solver
    test/test_position_primary_solver.cpp
  )
  target_link_libraries(test_position_primary_solver qiling_dik_core)

  ament_add_gtest(test_pose_hierarchy_solver
    test/test_pose_hierarchy_solver.cpp
  )
  target_link_libraries(test_pose_hierarchy_solver qiling_dik_core)

  ament_add_gtest(test_dual_arm_kinematics
    test/test_dual_arm_kinematics.cpp
  )
  ament_target_dependencies(test_dual_arm_kinematics ament_index_cpp)
  target_link_libraries(test_dual_arm_kinematics qiling_dik_core)

  ament_add_gtest(test_position_primary_kinematics
    test/test_position_primary_kinematics.cpp
  )
  ament_target_dependencies(test_position_primary_kinematics ament_index_cpp)
  target_link_libraries(test_position_primary_kinematics qiling_dik_core)

  ament_add_gtest(test_pose_hierarchy_kinematics
    test/test_pose_hierarchy_kinematics.cpp
  )
  ament_target_dependencies(test_pose_hierarchy_kinematics ament_index_cpp)
  target_link_libraries(test_pose_hierarchy_kinematics qiling_dik_core)

  set(QILING_MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../mujoco_simulator/third_party/mujoco")
  set(QILING_MUJOCO_LIBRARY "${QILING_MUJOCO_ROOT}/lib/libmujoco.so.3.3.0")
  if(EXISTS "${QILING_MUJOCO_ROOT}/include/mujoco/mujoco.h" AND
      EXISTS "${QILING_MUJOCO_LIBRARY}")
    ament_add_gtest(test_mujoco_pinocchio_fk
      test/test_mujoco_pinocchio_fk.cpp
    )
    ament_target_dependencies(test_mujoco_pinocchio_fk ament_index_cpp)
    target_include_directories(test_mujoco_pinocchio_fk PRIVATE
      "${QILING_MUJOCO_ROOT}/include"
    )
    target_link_libraries(test_mujoco_pinocchio_fk
      qiling_dik_core
      "${QILING_MUJOCO_LIBRARY}"
    )
    set_target_properties(test_mujoco_pinocchio_fk PROPERTIES
      BUILD_RPATH "${QILING_MUJOCO_ROOT}/lib"
    )
  else()
    message(WARNING "MuJoCo tree not found; skipping Pinocchio-MuJoCo FK regression test")
  endif()
endif()

ament_export_targets(export_qiling_dik_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 pinocchio proxsuite)

ament_package()
