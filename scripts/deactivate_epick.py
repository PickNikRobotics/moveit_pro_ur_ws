#!/usr/bin/env python3
"""Send a GripperCommand goal to deactivate the EPick vacuum."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand


def main() -> int:
    rclpy.init()
    node = Node("deactivate_epick")
    client = ActionClient(node, GripperCommand, "/epick_controller/gripper_cmd")

    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("EPick action server not available")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    goal = GripperCommand.Goal()
    goal.command.position = 0.0
    goal.command.max_effort = 0.0

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        node.get_logger().error("Goal rejected")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    node.get_logger().info("Vacuum deactivated")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
