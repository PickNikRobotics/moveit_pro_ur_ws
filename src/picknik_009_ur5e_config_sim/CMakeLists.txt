cmake_minimum_required(VERSION 3.22)
project(picknik_009_ur5e_config_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_009_ur5e_config REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# The MuJoCo shelving, tote and fixed cameras follow the calibration files.
# See design_docs/board_based_calibration.md.
set(CALIBRATION_DIR "${picknik_009_ur5e_config_DIR}/../objectives/calibration")
set(CALIBRATED_SCENE "${CMAKE_CURRENT_BINARY_DIR}/calibrated_scene.xml")
add_custom_command(
  OUTPUT ${CALIBRATED_SCENE}
  COMMAND
    ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_calibrated_scene_mjcf.py
    ${CALIBRATION_DIR}/cell_calibration.yaml
    ${CALIBRATION_DIR}/board_offsets.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/templates/calibrated_scene.xml.in
    ${CALIBRATED_SCENE}
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_calibrated_scene_mjcf.py
    ${CMAKE_CURRENT_SOURCE_DIR}/templates/calibrated_scene.xml.in
    ${CALIBRATION_DIR}/cell_calibration.yaml
    ${CALIBRATION_DIR}/board_offsets.yaml
  COMMENT "Generating the calibrated MuJoCo scene"
)
add_custom_target(calibrated_scene ALL DEPENDS ${CALIBRATED_SCENE})
install(
  FILES ${CALIBRATED_SCENE}
  DESTINATION share/${PROJECT_NAME}/mjcf/assets
)

install(
  DIRECTORY
    config
    launch
    mjcf
    objectives
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
