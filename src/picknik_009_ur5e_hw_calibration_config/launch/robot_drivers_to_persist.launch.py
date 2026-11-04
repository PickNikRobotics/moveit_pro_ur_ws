# Copyright 2026 PickNik Inc.
#
# Starts the normal hardware drivers with the cameras in calibration_mode.
# See design_docs/board_based_calibration.md.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("picknik_009_ur5e_hw_config"),
                    "launch",
                    "robot_drivers_to_persist.launch.py",
                ]
            )
        ),
        launch_arguments={"calibration_mode": "true"}.items(),
    )
    return LaunchDescription([hw_drivers_launch])
