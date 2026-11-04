# Copyright 2026 PickNik Inc.
#
# Launches a ZED camera through zed_wrapper's zed_camera.launch.py and maps its
# topics onto the /camera/<name>/... layout of design_docs/camera_naming.md.
# Counterpart to pro_rs_launch.py. See design_docs/zed_shelf_cam.md.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

# zed_wrapper publishes under /<camera_name>/<node_name>/.
ZED_NODE_NAME = "zed_node"

# zed_wrapper topic leaf -> workspace topic leaf. The color stream is the
# rectified left image; its camera_info describes that rectified image.
# The color image goes to an internal topic; rgb_image_relay.py republishes it as
# rgb8 on color/image_raw, because the ZED wrapper publishes BGR and the Trainer's
# converter does not reorder channels.
ZED_COLOR_TOPIC_LEAF = "color/image_bgr"
TOPIC_REMAPS = [
    ("rgb/color/rect/image", ZED_COLOR_TOPIC_LEAF),
    ("rgb/color/rect/camera_info", "color/camera_info"),
    ("depth/depth_registered", "depth/image_rect_raw"),
    ("depth/camera_info", "depth/camera_info"),
    ("point_cloud/cloud_registered", "depth/color/points"),
]


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration("camera_name").perform(context)

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": camera_name,
            "camera_model": LaunchConfiguration("camera_model"),
            "serial_number": LaunchConfiguration("serial_number"),
            "node_name": ZED_NODE_NAME,
            "ros_params_override_path": LaunchConfiguration(
                "ros_params_override_path"
            ),
            # Publish the camera's own link tree only. Its root link is tied
            # to world by the calibration YAML, not by the ZED odometry.
            "publish_urdf": "true",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "publish_imu_tf": "false",
        }.items(),
    )

    # SetRemap reaches the ZED composable node through the launch context.
    # If topics come back on their native names, check launch_ros first.
    topic_root = f"/{camera_name}/{ZED_NODE_NAME}/"
    remaps = [
        SetRemap(src=topic_root + src, dst=f"/camera/{camera_name}/{dst}")
        for src, dst in TOPIC_REMAPS
    ]
    rgb_relay = Node(
        package="picknik_009_ur5e_hw_config",
        executable="rgb_image_relay.py",
        name=f"{camera_name}_rgb_image_relay",
        remappings=[
            ("image_in", f"/camera/{camera_name}/{ZED_COLOR_TOPIC_LEAF}"),
            ("image_out", f"/camera/{camera_name}/color/image_raw"),
        ],
    )
    return [GroupAction(remaps + [zed_launch]), rgb_relay]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                description="Camera name: the ZED namespace and the <name> in "
                "/camera/<name>/...",
            ),
            DeclareLaunchArgument(
                "camera_model",
                default_value="zed2i",
                description="ZED model, passed to zed_camera.launch.py.",
            ),
            DeclareLaunchArgument(
                "serial_number",
                default_value="0",
                description="Camera serial number. 0 opens the first ZED found.",
            ),
            DeclareLaunchArgument(
                "ros_params_override_path",
                default_value="",
                description="One parameter YAML layered over the zed_wrapper "
                "defaults. Empty uses the defaults only.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
