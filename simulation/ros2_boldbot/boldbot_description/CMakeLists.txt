cmake_minimum_required(VERSION 3.5)

project(boldbot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files("/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/ros2_boldbot/boldbot_description/urdf/boldbot.urdf.xacro" INSTALL DESTINATION /home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/ros2_boldbot/boldbot_description/urdf/)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME})

install(FILES model.config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
