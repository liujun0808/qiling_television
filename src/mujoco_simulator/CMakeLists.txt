cmake_minimum_required(VERSION 3.8)
project(mujoco_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(glfw3 REQUIRED)
find_package(mit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(communicate_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)


# 设置mujoco文件路径
set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco)
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIB_DIR ${MUJOCO_DIR}/lib/libmujoco.so.3.3.0)

# 设置构建时的 RPATH（编译阶段生效）
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
# 添加 MUJOCO_DIR/lib 到 RPATH 搜索路径
set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MUJOCO_DIR}/lib")
# 防止系统路径覆盖自定义 RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)



aux_source_directory(src/demo SIMULATE_SOURCES)
aux_source_directory(third_party/lodepng SIMULATE_SOURCES)
add_executable(simulate
${SIMULATE_SOURCES}
src/main.cc
src/simulate_bridge.cpp
)
# 添加ros依赖
ament_target_dependencies(
        simulate
        "rclcpp"
        "ament_index_cpp"
        "mit_msgs"
        "std_msgs"
        "std_srvs"
        "communicate_interface"
        "urdf"
        "kdl_parser"
        "robot_state_publisher"
        "sensor_msgs"
        "tf2_ros"
)
# 添加第三方依赖
target_link_libraries(simulate ${MUJOCO_LIB_DIR} glfw yaml-cpp)
target_include_directories(simulate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lodepng
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tabulate/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/demo
  
)
target_compile_features(simulate PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS simulate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
