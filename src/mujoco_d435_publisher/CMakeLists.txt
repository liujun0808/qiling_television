cmake_minimum_required(VERSION 3.8)
project(mujoco_d435_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glfw3 REQUIRED)

set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../mujoco_simulator/third_party/mujoco)
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIB ${MUJOCO_DIR}/lib/libmujoco.so.3.3.0)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MUJOCO_DIR}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(d435_camera_publisher
  src/d435_camera_publisher.cpp
)

ament_target_dependencies(d435_camera_publisher
  ament_index_cpp
  rclcpp
  sensor_msgs
  std_msgs
)

target_include_directories(d435_camera_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(d435_camera_publisher
  ${MUJOCO_LIB}
  glfw
)

target_compile_features(d435_camera_publisher PUBLIC cxx_std_17)

install(TARGETS d435_camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
