#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
#
# Republishes a BGR(A) color image as rgb8. The Trainer's LeRobot converter
# reshapes raw images by channel count only, so a bgr8 image is stored with red
# and blue swapped and a bgra8 image fails to convert. The ZED wrapper cannot
# publish rgb8; the RealSense driver does. See design_docs/zed_shelf_cam.md.

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

# Source encoding -> (channels, index order that yields RGB).
_TO_RGB = {
    "bgr8": (3, [2, 1, 0]),
    "bgra8": (4, [2, 1, 0]),
    "rgba8": (4, [0, 1, 2]),
}


class RgbImageRelay(Node):
    def __init__(self):
        super().__init__("rgb_image_relay")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(Image, "image_out", qos)
        self._sub = self.create_subscription(Image, "image_in", self._on_image, qos)
        self._warned = set()

    def _on_image(self, msg: Image) -> None:
        encoding = msg.encoding.lower()
        if encoding == "rgb8":
            self._pub.publish(msg)
            return
        if encoding not in _TO_RGB:
            if encoding not in self._warned:
                self._warned.add(encoding)
                self.get_logger().error(f"Cannot convert encoding '{msg.encoding}' to rgb8")
            return
        channels, order = _TO_RGB[encoding]
        pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.step // channels, channels
        )[:, : msg.width, order]
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = "rgb8"
        out.is_bigendian = 0
        out.step = msg.width * 3
        out.data = np.ascontiguousarray(pixels).tobytes()
        self._pub.publish(out)


def main():
    rclpy.init()
    node = RgbImageRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
