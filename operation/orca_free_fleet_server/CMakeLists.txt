cmake_minimum_required(VERSION 3.5.0)

project(orca_free_fleet_server)

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)

  install(
    DIRECTORY
      launch
    DESTINATION share/${PROJECT_NAME}
  )

  install(
    PROGRAMS 
      scripts/send_destination_request.py
      scripts/send_path_request.py
      scripts/send_mode_request.py
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_package()

else()
  message("ament_cmake not found so skipping this ROS2 package")
endif()
