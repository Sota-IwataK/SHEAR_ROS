cmake_minimum_required(VERSION 3.5)
project(observation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(observation_msgs REQUIRED)
# find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)




# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_msgs
  tf2
  tf2_ros
  darknet_ros_msgs
  cv_bridge
  pcl_conversions
  # vision_msgs
  observation_msgs
  visualization_msgs
)

# add_executable(odom_tf2_broadcaster src/odom_tf2_broadcaster.cpp)
# ament_target_dependencies(
#     odom_tf2_broadcaster
#     tf2_geometry_msgs
#     nav_msgs
#     sensor_msgs
#     rclcpp
#     tf2
#     tf2_ros
# )

# add_executable(robot_manager src/robot_manager.cpp)
# ament_target_dependencies(
#   robot_manager
#   rclcpp
#   rcl_interfaces
# )

add_executable(my_pose_publisher src/my_pose_publisher.cpp)
ament_target_dependencies(
  my_pose_publisher
  ${dependencies}
)

# add_executable(nav2_send_goal src/nav2_send_goal.cpp)
# ament_target_dependencies(
#   nav2_send_goal
#   rclcpp
#   rclcpp_action
#   nav2_msgs
# )

add_executable(observer src/observer.cpp)
ament_target_dependencies(
  observer
  ${dependencies}
)

# add_executable(object_callback src/object_callback.cpp)
# ament_target_dependencies(
#   object_callback
#   ${dependencies}
# )


# target_link_libraries(object_callback
#  ${PCL_LIBRARIES}
#  )


# add_executable(dummy_obj_pub src/dummy_obj_pub.cpp)
# ament_target_dependencies(
#   dummy_obj_pub
#   ${dependencies}
# )

# add_executable(planning_scene src/planning_scene.cpp)
# ament_target_dependencies(
#   planning_scene
#   ${dependencies}# add_executable(my_pose_publisher src/my_pose_publisher.cpp)
#   # ament_target_dependencies(
#   #   my_pose_publisher
#   #   ${dependencies}
#   # )
  
# )
# install(TARGETS
#     odom_tf2_broadcaster
#     DESTINATION lib/${PROJECT_NAME})

# install(TARGETS
#   my_pose_publisher
#   DESTINATION lib/${PROJECT_NAME})

# install(TARGETS
#   nav2_send_goal
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  my_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

# install(TARGETS
#   object_callback planning_scene
#   DESTINATION lib/${PROJECT_NAME})

# install(TARGETS
#   dummy_obj_pub
#   DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(pcl_conversions)

ament_package()
