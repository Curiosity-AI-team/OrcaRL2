cmake_minimum_required(VERSION 3.5)
project(points_concat_filter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add the executable
add_executable(points_concat_async_node
  src/points_concat_async_node.cpp
  src/main.cpp
)

ament_target_dependencies(points_concat_async_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  PCL
  pcl_conversions
)

target_link_libraries(points_concat_async_node
  ${PCL_LIBRARIES}
)

# Install targets
install(TARGETS points_concat_async_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()