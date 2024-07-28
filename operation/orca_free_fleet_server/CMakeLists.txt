cmake_minimum_required(VERSION 3.5.0)

project(orca_free_fleet_server LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
  find_package(builtin_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rmf_fleet_msgs REQUIRED)
  find_package(free_fleet REQUIRED)
  find_package(Eigen3 REQUIRED)

  add_executable(orca_free_fleet_server
    src/main.cpp
    src/utilities.cpp
    src/ServerNode.cpp
    src/ServerNodeConfig.cpp
  )
  target_link_libraries(orca_free_fleet_server
    ${free_fleet_LIBRARIES}
    Eigen3::Eigen
  )
  target_include_directories(orca_free_fleet_server
    PRIVATE
      ${free_fleet_INCLUDE_DIRS}
  )
  ament_target_dependencies(orca_free_fleet_server
    rclcpp
    rmf_fleet_msgs
  )

  
  install(
    TARGETS orca_free_fleet_server
    RUNTIME DESTINATION lib/orca_free_fleet_server
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  ament_package()

else()
  message("ament_cmake not found so skipping this ROS2 package")
endif()

# -----------------------------------------------------------------------------


# -----------------------------------------------------------------------------


