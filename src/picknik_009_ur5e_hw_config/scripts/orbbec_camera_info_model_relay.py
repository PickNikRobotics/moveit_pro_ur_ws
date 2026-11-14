#!/usr/bin/env python3
"""Correct the Gemini 305 CameraInfo distortion-model label for consumers.

The installed Orbbec driver publishes eight D coefficients while declaring
plumb_bob. Eight coefficients are the ROS rational_polynomial model. This
relay preserves all camera calibration data and changes only the model label.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class OrbbecCameraInfoModelRelay(Node):
    def __init__(self):
        super().__init__("orbbec_camera_info_model_relay")
        self._input_topic = self.declare_parameter(
            "input_topic", "/camera/wrist_orbbec/color/camera_info_raw"
        ).value
        self._output_topic = self.declare_parameter(
            "output_topic", "/camera/wrist_orbbec/color/camera_info"
        ).value
        self._publisher = self.create_publisher(
            CameraInfo, self._output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            CameraInfo,
            self._input_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self._reported = False

    def _on_camera_info(self, message):
        if message.distortion_model == "plumb_bob" and len(message.d) == 8:
            message.distortion_model = "rational_polynomial"
            if not self._reported:
                self.get_logger().info(
                    "Correcting Gemini CameraInfo distortion_model from "
                    "plumb_bob to rational_polynomial for eight D coefficients."
                )
                self._reported = True
        elif len(message.d) not in (0, 4, 5, 8):
            self.get_logger().error(
                f"Passing CameraInfo unchanged with unsupported distortion "
                f"coefficient count {len(message.d)}; calibration consumers "
                "will reject it."
            )
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = OrbbecCameraInfoModelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
