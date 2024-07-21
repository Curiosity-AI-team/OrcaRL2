cmake_minimum_required(VERSION 3.5)
project(driver_odrive)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

# find_library(DBCPPP_LIB NAMES dbcppp PATHS /home/rover/gr_platform2/dbcppp/build)

# add_executable(test_odrive_control src/test_odrive_control.cpp)
# ament_target_dependencies(test_odrive_control rclcpp can_msgs geometry_msgs)
# target_link_libraries(test_odrive_control ${DBCPPP_LIB})

add_executable(test_socketcan src/test_socketcan.cpp)
ament_target_dependencies(test_socketcan rclcpp can_msgs)

add_library(driver_odrive SHARED src/diffbot_system.cpp)

target_include_directories(
  driver_odrive
  PRIVATE
  include
)
ament_target_dependencies(
  driver_odrive
  hardware_interface
  controller_manager
  can_msgs
  ros2_socketcan
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)


# install(TARGETS 
#     test_socketcan
#     test_odrive_control
#     DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS 
  driver_odrive 
  # fake_robot
  DESTINATION lib
)

install(PROGRAMS
  src/test_odrive_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  driver_odrive
)

ament_export_dependencies(
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

ament_package()


