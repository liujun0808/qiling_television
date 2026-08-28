
cmake_minimum_required(VERSION 3.8)
project(communicate_interface)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
        ${PROJECT_NAME}
        "srv/ControlState.srv"
        DEPENDENCIES builtin_interfaces std_msgs  
)

ament_package()
