"""Driver launch for the 2F-85 config's Orbbec wrist-camera setup."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


CALIBRATION_PACKAGE = "picknik_009_ur5e_config"
CALIBRATION_RELATIVE_PATH = "objectives/calibration/cell_calibration.yaml"


def orbbec_mount_transform():
    """Publish the current Orbbec root mount transform from calibration YAML."""
    calibration_path = os.path.join(
        get_package_share_directory(CALIBRATION_PACKAGE),
        CALIBRATION_RELATIVE_PATH,
    )
    if not os.path.exists(calibration_path):
        raise RuntimeError(
            f"Missing calibration file {calibration_path}; refusing to launch "
            "the 2F Orbbec without its wrist mount."
        )

    with open(calibration_path, encoding="utf-8") as calibration_file:
        calibration = yaml.safe_load(calibration_file) or {}

    entry = calibration.get("wrist_orbbec_mount")
    if not entry:
        raise RuntimeError(
            f"Missing 'wrist_orbbec_mount' in {calibration_path}; refusing to "
            "launch the 2F Orbbec without its wrist mount."
        )
    if entry.get("header", {}).get("frame_id") != "d415_mount_link":
        raise RuntimeError(
            "wrist_orbbec_mount must be expressed in d415_mount_link for the "
            "2F mount."
        )

    # The YAML saver replaces the whole section with a PoseStamped when a
    # candidate is applied, so the calibrated record has no `temporary` key.
    position = entry["pose"]["position"]
    orientation = entry["pose"]["orientation"]
    quaternion_norm = sum(orientation[axis] ** 2 for axis in ("x", "y", "z", "w")) ** 0.5
    if quaternion_norm == 0.0:
        raise RuntimeError("wrist_orbbec_mount has a zero-length orientation quaternion.")
    normalized_orientation = {
        axis: orientation[axis] / quaternion_norm for axis in ("x", "y", "z", "w")
    }
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="wrist_orbbec_mount_tf",
            arguments=[
                "--x", str(position["x"]),
                "--y", str(position["y"]),
                "--z", str(position["z"]),
                "--qx", str(normalized_orientation["x"]),
                "--qy", str(normalized_orientation["y"]),
                "--qz", str(normalized_orientation["z"]),
                "--qw", str(normalized_orientation["w"]),
                "--frame-id", entry["header"]["frame_id"],
                "--child-frame-id", "wrist_orbbec_link",
            ],
        )
    ]


def generate_launch_description():
    calibration_mode = LaunchConfiguration("calibration_mode")
    base_drivers_launch = PathJoinSubstitution(
        [
            FindPackageShare("picknik_009_ur5e_hw_config"),
            "launch",
            "robot_drivers_to_persist.launch.py",
        ]
    )
    cameras_launch = PathJoinSubstitution(
        [
            FindPackageShare("picknik_009_ur5e_hw_config"),
            "launch",
            "cameras.launch.xml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_mode",
                default_value="false",
                description="Run the cameras at their calibration-only resolutions.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_drivers_launch),
                launch_arguments={
                    "calibration_mode": calibration_mode,
                    "launch_cameras": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(cameras_launch),
                launch_arguments={
                    "calibration_mode": calibration_mode,
                    "launch_wrist_camera": "false",
                    "launch_wrist_orbbec": "true",
                    "wrist_orbbec_dual_color": "false",
                }.items(),
            ),
        ]
        + orbbec_mount_transform()
    )
