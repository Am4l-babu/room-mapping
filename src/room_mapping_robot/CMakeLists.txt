# =============================================================================
# CMakeLists.txt — Build configuration for room_mapping_robot
# =============================================================================
#
# This CMake file:
#   1. Declares the ament_cmake package
#   2. Finds all required ROS2 dependencies
#   3. Installs launch files, URDF, config, worlds, rviz, and scripts
#      into the correct share/ directory so ros2 launch can find them
#
# No C++ nodes are built here — all simulation is handled by Gazebo plugins
# and existing ROS2 nodes (robot_state_publisher, slam_toolbox, etc.)
# =============================================================================

cmake_minimum_required(VERSION 3.8)
project(room_mapping_robot)

# Default to C++17 for ROS2 Jazzy compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Find Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# ---- Install launch files ----
# These Python launch files orchestrate Gazebo, robot spawning, SLAM, etc.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ---- Install URDF/Xacro robot model ----
# The robot model defines links, joints, sensors, and Gazebo plugins
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# ---- Install configuration files ----
# SLAM parameters, sensor configs, etc.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ---- Install Gazebo world files ----
# The simulated indoor room environment
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# ---- Install RViz configuration ----
# Pre-configured RViz layout for mapping visualization
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# ---- Install executable scripts ----
# Helper scripts for teleoperation, etc.
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

# ---- Install Python scripts as executables ----
install(PROGRAMS
  scripts/teleop_helper.py
  scripts/autonomous_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Testing ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ---- Register the package ----
ament_package()
