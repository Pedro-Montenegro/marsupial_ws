cmake_minimum_required(VERSION 3.8)
project(marsupial_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecutePath.action"
  DEPENDENCIES builtin_interfaces nav_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

