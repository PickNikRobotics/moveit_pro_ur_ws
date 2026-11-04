cmake_minimum_required(VERSION 3.22)
project(picknik_009_ur5e_hw_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    objectives
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/rgb_image_relay.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
