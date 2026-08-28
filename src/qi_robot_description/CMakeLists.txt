cmake_minimum_required(VERSION 3.8)
project(qi_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf new_scene
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
