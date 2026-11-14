#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
#
# Reboots the wrist Orbbec through its driver when it stops delivering frames.
#
# About one start in two or three, the Gemini 305 opens, logs every stream as
# enabled, and then sends nothing, with no error from the driver. It does not
# recover by itself, and relaunching the node does not reliably fix it. The
# driver's reboot_device service does, in place and without root. This node
# watches one camera_info topic (one small message per frame) and calls that
# service when it goes quiet. See design_docs/orbbec_wrist_camera.md.

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Empty


class OrbbecStreamWatchdog(Node):
    def __init__(self):
        super().__init__("orbbec_stream_watchdog")
        self._topic = self.declare_parameter("camera_info_topic", "").value
        service = self.declare_parameter("reboot_service", "").value
        # No frame for this long counts as a stall.
        self._stall_timeout = self.declare_parameter("stall_timeout_s", 10.0).value
        # Time a start or a reboot gets before it is judged. Doubles after each
        # reboot that brings no frame (an unplugged camera), up to the maximum.
        self._grace = self.declare_parameter("recovery_grace_s", 30.0).value
        self._max_grace = self.declare_parameter("max_recovery_grace_s", 300.0).value
        if not self._topic or not service:
            raise ValueError("camera_info_topic and reboot_service must be set")

        self._client = self.create_client(Empty, service)
        self.create_subscription(
            CameraInfo, self._topic, self._on_frame, qos_profile_sensor_data
        )
        now = time.monotonic()
        self._last_frame = now
        self._judge_after = now + self._grace
        self._current_grace = self._grace
        self._streaming = False
        self.create_timer(1.0, self._check)

    def _on_frame(self, _msg):
        self._last_frame = time.monotonic()
        if not self._streaming:
            self.get_logger().info(f"Frames arriving on {self._topic}")
        self._streaming = True
        self._current_grace = self._grace

    def _check(self):
        now = time.monotonic()
        if now < self._judge_after or now - self._last_frame < self._stall_timeout:
            return
        self._streaming = False
        if not self._client.service_is_ready():
            self.get_logger().warn(
                f"No frames on {self._topic} and {self._client.srv_name} is not "
                "available; is the camera node running?",
                throttle_duration_sec=60.0,
            )
            return
        self.get_logger().warn(
            f"No frames on {self._topic} for {now - self._last_frame:.0f} s; "
            f"calling {self._client.srv_name}"
        )
        self._client.call_async(Empty.Request())
        self._judge_after = now + self._current_grace
        self._current_grace = min(self._current_grace * 2.0, self._max_grace)


def main():
    rclpy.init()
    node = OrbbecStreamWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
