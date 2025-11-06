cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(robot_controller_msgs REQUIRED)

add_executable(robot_controller_node 
  src/main.cpp
  src/move_group.cpp
  src/robot_ctlr.cpp

  src/service/collision_object.cpp
  src/service/execution.cpp
  src/service/utils.cpp

  src/bspline/bspline.cpp
)
target_include_directories(robot_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_controller_node 
  rclcpp 
  rclcpp_action
  sensor_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs

  robot_controller_msgs
)
target_compile_features(robot_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS robot_controller_node
  DESTINATION lib/${PROJECT_NAME})

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
