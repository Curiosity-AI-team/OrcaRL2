cmake_minimum_required(VERSION 3.5)
project(boldbot_gazebo_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mx_joint_controller_msgs REQUIRED)

# Include Gazebo and Gazebo ROS's include directories
include_directories(${GAZEBO_INCLUDE_DIRS} ${gazebo_ros_INCLUDE_DIRS})

add_library(boldbot_gazebo_plugin SHARED src/boldbot_gazebo_plugin.cpp)

target_include_directories(boldbot_gazebo_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS})

ament_target_dependencies(boldbot_gazebo_plugin
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
  "mx_joint_controller_msgs")

install(TARGETS
  boldbot_gazebo_plugin
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
