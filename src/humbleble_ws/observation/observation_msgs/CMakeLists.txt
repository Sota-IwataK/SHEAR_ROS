cmake_minimum_required(VERSION 3.5)
project(observation_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)


set(dependencies
  builtin_interfaces
  std_msgs
  action_msgs
  moveit_msgs
  nav2_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathEvaluation.msg"
  "msg/Object.msg"
  "srv/SetObjects.srv"
  "srv/GetObjects.srv"
  "srv/PathCheck.srv"
  DEPENDENCIES ${dependencies}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
