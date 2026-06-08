cmake_minimum_required(VERSION 3.5)
project(my_nav_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

set(dependencies
  builtin_interfaces
  std_msgs
  action_msgs
  visualization_msgs
  nav2_msgs
)

set(msg_files
  msg/PathInfo.msg
  msg/PathWithCondition.msg
)

set(action_files
  action/ComputeMultiPath.action
)

set(srv_files
  srv/SetObjCost.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES ${dependencies}
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
