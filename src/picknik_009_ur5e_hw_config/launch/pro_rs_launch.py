import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction

# Add the realsense2_camera/launch directory to the user's path so we can import the realsense2_camera rs_launch module.
realsense2_camera_path = get_package_share_directory("realsense2_camera")
sys.path.append(f"{realsense2_camera_path}/launch")

import rs_launch


def generate_launch_description():
    rs_launch.configurable_parameters.extend(
        [
            {
                "name": "color_qos",
                "default": "SENSOR_DATA",
                "description": "Color stream QoS settings",
            },
            {
                "name": "depth_qos",
                "default": "SENSOR_DATA",
                "description": "Depth stream QoS settings",
            },
            # Upstream exposes spatial_filter.enable but not its hole-fill mode.
            # The default is the driver's own (0, no filling), so a camera that
            # does not set it is unchanged.
            {
                "name": "spatial_filter.holes_fill",
                "default": "0",
                "description": "Spatial filter hole filling: 0 none, 1-4 up to 2/4/8/16 px, 5 unlimited",
            },
        ]
    )
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(rs_launch.configurable_parameters)
        + [
            OpaqueFunction(
                function=rs_launch.launch_setup,
                kwargs={
                    "params": rs_launch.set_configurable_parameters(
                        rs_launch.configurable_parameters
                    )
                },
            )
        ]
    )
