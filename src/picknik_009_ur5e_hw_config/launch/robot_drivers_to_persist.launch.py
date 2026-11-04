# Copyright 2023 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

# Fixed cameras whose world pose comes from the calibration file: (section, child frame).
CALIBRATED_CAMERAS = [
    ("scene_camera_link", "scene_camera_link"),
    ("shelf_cam_camera_link", "shelf_cam_camera_link"),
]


def calibrated_camera_transforms():
    """One static transform publisher per calibrated fixed camera, read once at stack start."""
    path = os.path.join(
        get_package_share_directory("picknik_009_ur5e_config"),
        "objectives",
        "calibration",
        "cell_calibration.yaml",
    )
    if not os.path.exists(path):
        return [LogInfo(msg=f"No {path}: fixed-camera transforms come only from Load Calibrated Scene.")]
    with open(path) as f:
        calibration = yaml.safe_load(f) or {}
    actions = []
    for section, child in CALIBRATED_CAMERAS:
        entry = calibration.get(section)
        if not entry:
            actions.append(LogInfo(msg=f"No '{section}' section in {path}: {child} is not published at start."))
            continue
        p = entry["pose"]["position"]
        q = entry["pose"]["orientation"]
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{child}_calibration_tf",
                arguments=[
                    "--x", str(p["x"]), "--y", str(p["y"]), "--z", str(p["z"]),
                    "--qx", str(q["x"]), "--qy", str(q["y"]), "--qz", str(q["z"]), "--qw", str(q["w"]),
                    "--frame-id", entry["header"]["frame_id"],
                    "--child-frame-id", child,
                ],
            )
        )
    return actions


def generate_launch_description():
    # Reuse ur_hw's robot drivers launch file (copied into picknik_009_ur5e_hw_config)
    base_robot_drivers_to_persist_launch_path = PathJoinSubstitution(
        [
            FindPackageShare("picknik_009_ur5e_hw_config"),
            "launch",
            "ur_hw.robot_drivers_to_persist.launch.py",
        ]
    )

    base_robot_drivers_to_persist_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_robot_drivers_to_persist_launch_path),
        launch_arguments={"robot_ip": "192.10.0.11"}.items(),
    )

    cameras_launch_path = PathJoinSubstitution(
        [
            FindPackageShare("picknik_009_ur5e_hw_config"),
            "launch",
            "cameras.launch.xml",
        ]
    )

    # The calibration config package sets this; normal runs keep the default.
    cameras_launch = IncludeLaunchDescription(
        cameras_launch_path,
        launch_arguments={"calibration_mode": LaunchConfiguration("calibration_mode")}.items(),
    )

    # The Trainer Training Config records this relay's topic: the policy joint states with
    # the EPick seal appended. Lives in the src/shelf_stocking submodule; see its
    # design_docs/policy_suction_channel.md.
    policy_state_seal_relay = Node(
        package="shelf_stocking_objectives",
        executable="policy_state_seal_relay.py",
        name="policy_state_seal_relay",
    )

    # Keeps the Shopping List pane's order in shelf_stocking_objectives'
    # config/shopping_list.yaml for the unshelve collections. See the submodule's
    # design_docs/shopping_list.md.
    shopping_list_server = Node(
        package="shelf_stocking_objectives",
        executable="shopping_list_server.py",
        name="shopping_list_server",
    )

    # The custom view panes are served same-origin by MoveIt Pro's REST
    # frontend_settings.yaml. The pane server IS launched, but from the base
    # package's agent_bridge.launch.xml rather than here, so there is no
    # per-host include to maintain and no port collision to silence.

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_mode",
                default_value="false",
                description="Run the cameras at the calibration-only resolutions.",
            ),
            base_robot_drivers_to_persist_launch,
            cameras_launch,
            policy_state_seal_relay,
            shopping_list_server,
        ]
        + calibrated_camera_transforms()
    )
