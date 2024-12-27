cmake_minimum_required(VERSION 3.8)
project(my_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build a library for traj_process
add_library(traj_process src/traj_process.cpp)

target_include_directories(traj_process PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  traj_process
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  moveit_visual_tools
)

# Executable 1: obs_planner (depends on traj_process)
add_executable(obs_planner src/obs_planner.cpp)
target_link_libraries(obs_planner traj_process)  # Link with traj_process
ament_target_dependencies(
  obs_planner
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  moveit_visual_tools
)

# Executable 2: my_moveit
add_executable(my_moveit src/my_moveit.cpp)
target_link_libraries(my_moveit traj_process)  # Optional, if used in my_moveit
ament_target_dependencies(
  my_moveit
  moveit_ros_planning_interface
  rclcpp
  std_msgs
)

# Executable 3: my_node_interface
add_executable(my_node_interface src/my_node_interface.cpp)
ament_target_dependencies(
  my_node_interface
  moveit_ros_planning_interface
  rclcpp
  moveit_visual_tools
)

# Executable 4: test_node101
add_executable(test_node101 src/test_node101.cpp)
ament_target_dependencies(
  test_node101
  moveit_visual_tools
  moveit_ros_planning_interface
  rclcpp
)

# Installation of Targets
install(TARGETS
  my_moveit
  my_node_interface
  test_node101
  obs_planner
  traj_process  # Include the library
  DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
