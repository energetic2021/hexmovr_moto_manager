cmake_minimum_required(VERSION 3.8)
project(hexmovr_ros2_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config doc include src
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
