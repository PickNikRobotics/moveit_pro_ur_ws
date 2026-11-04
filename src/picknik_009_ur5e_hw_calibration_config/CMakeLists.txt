cmake_minimum_required(VERSION 3.22)
project(picknik_009_ur5e_hw_calibration_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
