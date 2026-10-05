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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(robot_controller_msgs REQUIRED)


add_library(robot_controller_core SHARED
  src/move_group_interface.cpp
  src/bspline.cpp
)
target_include_directories(robot_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_controller_core PUBLIC cxx_std_17)
target_compile_options(robot_controller_core PRIVATE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion
)
ament_target_dependencies(robot_controller_core
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  Eigen3
)

# ---------------------------------------------------------------------------
# Node library: RobotControllerNode
# ---------------------------------------------------------------------------
add_library(robot_controller_node_component SHARED
  src/robot_controller_node.cpp
  src/services/execution_services.cpp
  src/services/collision_object_services.cpp
  src/services/planning_services.cpp
  src/services/query_services.cpp
)
target_include_directories(robot_controller_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robot_controller_node_component PUBLIC cxx_std_17)
target_compile_options(robot_controller_node_component PRIVATE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion
)
target_link_libraries(robot_controller_node_component
  robot_controller_core
)
ament_target_dependencies(robot_controller_node_component
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  robot_controller_msgs
)

rclcpp_components_register_nodes(robot_controller_node_component
  "robot_controller::RobotControllerNode"
)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(robot_controller_node src/main.cpp)
target_link_libraries(robot_controller_node
  robot_controller_node_component
)
target_compile_features(robot_controller_node PRIVATE cxx_std_17)
target_compile_options(robot_controller_node PRIVATE
  -Wall -Wextra -Wpedantic
)

install(TARGETS
    robot_controller_core
    robot_controller_node_component
    robot_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

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
