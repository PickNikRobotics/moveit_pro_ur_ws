# Copyright 2026 PickNik Inc.
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

"""EPick hardware drivers: the gripper-neutral hw drivers plus the EPick seal relay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
        launch_arguments={
            "calibration_mode": LaunchConfiguration("calibration_mode"),
            "launch_cameras": LaunchConfiguration("launch_cameras"),
        }.items(),
    )

    # The Trainer Training Config records this relay's topic: the policy joint states with
    # the EPick seal appended. Lives in the src/shelf_stocking submodule; see its
    # design_docs/policy_suction_channel.md.
    policy_state_seal_relay = Node(
        package="shelf_stocking_objectives",
        executable="policy_state_seal_relay.py",
        name="policy_state_seal_relay",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_mode",
                default_value="false",
                description="Run the cameras at the calibration-only resolutions.",
            ),
            DeclareLaunchArgument(
                "launch_cameras",
                default_value="true",
                description="Launch the camera drivers with this hardware config.",
            ),
            hw_drivers_launch,
            policy_state_seal_relay,
        ]
    )
