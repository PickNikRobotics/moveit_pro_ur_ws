cmake_minimum_required(VERSION 3.22)
project(picknik_009_ur5e_2f85_policy_config)

find_package(ament_cmake REQUIRED)
find_package(picknik_009_ur5e_config REQUIRED)

# The policy-session joint limits: the workspace's joint_limits.yaml with the six arm joints'
# max_acceleration raised, everything else (position and velocity limits, the gripper)
# unchanged. Generated rather than copied so the position limits tuned in the base file cannot
# drift. See design_docs/policy_session_config.md.
set(POLICY_MAX_ACCELERATION "2.5" CACHE STRING "Arm joint max_acceleration (rad/s^2) for policy rollouts")
set(base_limits "${picknik_009_ur5e_config_DIR}/../config/moveit/joint_limits.yaml")
# Re-run the generation when the base file changes.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${base_limits}")
file(READ "${base_limits}" limits)
# Only the arm: everything before the gripper's entry. The gripper keeps the base value.
string(FIND "${limits}" "  robotiq_85_left_knuckle_joint:" gripper_at)
if(gripper_at EQUAL -1)
  message(FATAL_ERROR "${base_limits}: no robotiq_85_left_knuckle_joint entry to stop the arm section at")
endif()
string(SUBSTRING "${limits}" 0 ${gripper_at} arm)
string(SUBSTRING "${limits}" ${gripper_at} -1 rest)
string(REGEX MATCHALL "\n    max_acceleration: [0-9.]+\n" found "${arm}")
list(LENGTH found count)
if(NOT count EQUAL 6)
  message(FATAL_ERROR "${base_limits}: expected 6 arm max_acceleration entries before the gripper, found ${count}")
endif()
string(REGEX REPLACE "\n    max_acceleration: [0-9.]+\n" "\n    max_acceleration: ${POLICY_MAX_ACCELERATION}\n" arm "${arm}")
set(header "# GENERATED at build time by picknik_009_ur5e_2f85_policy_config/CMakeLists.txt from\n")
string(APPEND header "# picknik_009_ur5e_config/config/moveit/joint_limits.yaml, with the arm joints'\n")
string(APPEND header "# max_acceleration set to ${POLICY_MAX_ACCELERATION}.\n")
string(APPEND header "# Do not edit; change the base file or POLICY_MAX_ACCELERATION.\n")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/joint_limits.yaml" "${header}${arm}${rest}")

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/joint_limits.yaml" DESTINATION share/${PROJECT_NAME}/config/moveit)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
