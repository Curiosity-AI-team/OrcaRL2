cmake_minimum_required(VERSION 3.5)
project(ik_walk)

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
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(mx_joint_controller_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ik_walk_msgs REQUIRED)

add_library(ik_walks
    src/ik_walk.cpp
    external/IKWalk/IKWalk.cpp
    external/IKWalk/CubicSpline.cpp
    external/IKWalk/HumanoidModel.cpp
    external/IKWalk/IKWalk.cpp
    external/IKWalk/LegIK.cpp
    external/IKWalk/Polynom.cpp
    external/IKWalk/Spline.cpp)

target_include_directories(ik_walks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(ik_walks
  "rclcpp"
  "geometry_msgs"
  "tf2_eigen"
  "mx_joint_controller_msgs"
  "sensor_msgs"
  "ik_walk_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ik_walks PRIVATE "IK_WALK_BUILDING_LIBRARY")

install(
  DIRECTORY include/ DESTINATION include
)
install(
  DIRECTORY config DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ik_walks
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(ik_walk_node src/ik_walk_node.cpp)
target_include_directories(ik_walk_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ik_walk_node ik_walks)

install(TARGETS ik_walk_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  "sensor_msgs"
  "ik_walk_msgs"
)

ament_export_include_directories(
  include
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_libraries(
  ik_walks
)

ament_package()
