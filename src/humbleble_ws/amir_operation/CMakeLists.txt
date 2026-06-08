cmake_minimum_required(VERSION 3.8)
project(amir_operation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amir_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(line_msg REQUIRED)
find_package(bebop_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(realsense2_camera_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)      
find_package(trajectory_msgs REQUIRED)

include_directories(
  include
)

add_executable(initial_posi 
  src/initial_posi.cpp
)
add_executable(initial_posi_gz 
  src/initial_posi_gz.cpp
)
add_executable(pick 
  src/pick.cpp
)
add_executable(place 
  src/place.cpp
)
#add_executable(pick_using_d_optimality 
#  src/pick_using_d_optimality.cpp
#)
add_executable(search_obj 
  src/search_obj.cpp
)
add_executable(move_arm 
  src/move_arm.cpp
)
add_executable(grasp 
  src/grasp.cpp
)
add_executable(move_meca 
  src/move_meca.cpp
)
#add_executable(yolov5_camera 
#  src/yolov5_camera.cpp
#)
#add_executable(object_pub 
#  src/object_pub.cpp
#)
add_executable(before_arm
  src/before_arm.cpp
)
add_executable(after_arm
  src/after_arm.cpp
)
add_executable(key_posi 
  src/key_posi.cpp
)
#add_executable(ik_pub 
#  src/kinematics_ik_publisher.cpp
#  src/kinematics.cpp
#)
add_executable(search_amir
  src/search_amir.cpp
)

#add_executable(amir_motion
#  src/amir_motion.cpp
#)

add_executable(pick_place_hum
  src/pick_place_humble.cpp
)

add_executable(jointfix
  src/jointfix_pick_place.cpp
)

add_executable(ultra
  src/pick_place_ultra.cpp
)

add_executable(pick_place_hum_2
  src/pick_place_humble_2.cpp
)

add_executable(pick_place_hum_3
  src/pick_place_humble_3.cpp
)

add_executable(survey_ik
  src/survey_ik.cpp
)


set(dependencies
  rclcpp 
  std_msgs
  amir_interfaces
  geometry_msgs
  rclcpp_action
  behavior_tree_msgs
  nav_msgs
  Eigen3
  tf2
  tf2_geometry_msgs
  line_msg
  bebop_msgs
  sensor_msgs
  cv_bridge
  #realsense2_camera_msgs
  darknet_ros_msgs
  pcl_conversions
  pcl_msgs
  PCL
  OpenCV
  image_transport
  moveit_ros_planning_interface
  trajectory_msgs
  control_msgs
)

ament_target_dependencies(initial_posi ${dependencies})
ament_target_dependencies(pick ${dependencies})
ament_target_dependencies(place ${dependencies})
ament_target_dependencies(search_obj ${dependencies})
ament_target_dependencies(move_arm ${dependencies})
#ament_target_dependencies(pick_using_d_optimality ${dependencies})
ament_target_dependencies(grasp ${dependencies})
ament_target_dependencies(move_meca ${dependencies})
#ament_target_dependencies(yolov5_camera ${dependencies})
#ament_target_dependencies(object_pub ${dependencies})
ament_target_dependencies(before_arm ${dependencies})
ament_target_dependencies(after_arm ${dependencies})
ament_target_dependencies(key_posi ${dependencies})
ament_target_dependencies(search_amir ${dependencies})
#ament_target_dependencies(ik_pub ${dependencies})
#ament_target_dependencies(amir_motion ${dependencies})
ament_target_dependencies(pick_place_hum ${dependencies})
ament_target_dependencies(pick_place_hum_2 ${dependencies})
ament_target_dependencies(pick_place_hum_3 ${dependencies})
ament_target_dependencies(initial_posi_gz ${dependencies})
ament_target_dependencies(survey_ik ${dependencies})
ament_target_dependencies(jointfix ${dependencies})
ament_target_dependencies(ultra ${dependencies})

install(TARGETS
  initial_posi
  initial_posi_gz
  pick
  place
  search_obj
  move_arm
  grasp
  move_meca
  #object_pub
  before_arm
  after_arm
  key_posi
  #ik_pub
  search_amir
  #amir_motion
  pick_place_hum
  pick_place_hum_2
  pick_place_hum_3
  jointfix
  survey_ik
  ultra
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME})



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
