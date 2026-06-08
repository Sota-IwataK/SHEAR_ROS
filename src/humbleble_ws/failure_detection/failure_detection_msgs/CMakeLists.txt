cmake_minimum_required(VERSION 3.8)
project(failure_detection_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(my_nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(behavior_tree_msgs REQUIRED)


set(dependencies
  my_nav_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  behavior_tree_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Solution.msg"
  "action/Analysys.action"
  "srv/PathCheck.srv"
  
  "srv/BeforePick.srv"
  "srv/AfterPick.srv"
  DEPENDENCIES ${dependencies}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
