#!/usr/bin/env python3
"""Reset the UR arm after a protective stop or a stopped control program: the UI's Reset Arm.

The UI's Reset Arm button calls /recover_from_protective_stop but cannot reach it: the
Desktop App refuses to call any std_srvs/Trigger service (see "Recovering a latched fault
without the pendant" in design_docs/control_loop_realtime_priority.md). This calls the same
service. It unlocks the protective stop, stops the UR program, deactivates every controller,
re-sends the program, and reactivates the startup controllers. The script reports the arm's
state before and after, and refuses when the stop is one only a person at the robot can clear.

Run inside the drivers container as the host user:

    source ~/.moveit-pro-bashrc
    python3 ~/user_ws/scripts/reset_arm.py

or from the host with scripts/reset_arm.sh.
"""

import argparse
import sys
import time

import rclpy
from moveit_studio_agent_msgs.msg import FaultStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import RobotMode, SafetyMode

RECOVER_SERVICE = "/recover_from_protective_stop"
ROBOT_MODE_TOPIC = "/io_and_status_controller/robot_mode"
SAFETY_MODE_TOPIC = "/io_and_status_controller/safety_mode"
PROGRAM_TOPIC = "/io_and_status_controller/robot_program_running"
FAULT_TOPIC = "/protective_stop_manager_node/robot_fault_status"
# The service retries the unlock and waits on several driver services, so it can take a while.
RECOVER_TIMEOUT_S = 90.0

ROBOT_MODES = {v: k for k, v in vars(RobotMode).items() if k.isupper() and isinstance(v, int)}
SAFETY_MODES = {v: k for k, v in vars(SafetyMode).items() if k.isupper() and isinstance(v, int)}
FAULTS = {v: k for k, v in vars(FaultStatus).items() if k.isupper() and isinstance(v, int)}

# Stops software cannot clear: an e-stop or safeguard input, or a safety fault. The service's
# unlock just retries for a while and then fails, so say what to do at the robot instead.
NEEDS_A_PERSON = {
    SafetyMode.SAFEGUARD_STOP: "a safeguard stop input is active",
    SafetyMode.SYSTEM_EMERGENCY_STOP: "a system emergency stop is pressed",
    SafetyMode.ROBOT_EMERGENCY_STOP: "the robot's emergency stop is pressed",
    SafetyMode.VIOLATION: "a safety limit was violated; clear it on the teach pendant",
    SafetyMode.FAULT: "a safety fault; clear it on the teach pendant",
    SafetyMode.AUTOMATIC_MODE_SAFEGUARD_STOP: "an automatic-mode safeguard stop is active",
    SafetyMode.SYSTEM_THREE_POSITION_ENABLING_STOP: "the three-position enabling device is released",
}


class ArmState(Node):
    def __init__(self):
        super().__init__("reset_arm")
        # Every topic here publishes only on change. The io_and_status_controller ones are
        # latched, so they arrive on subscribe; the fault status is not, so it may never arrive.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.robot_mode = self.safety_mode = self.program_running = self.fault = None
        self.create_subscription(RobotMode, ROBOT_MODE_TOPIC, lambda m: setattr(self, "robot_mode", m.mode), latched)
        self.create_subscription(SafetyMode, SAFETY_MODE_TOPIC, lambda m: setattr(self, "safety_mode", m.mode), latched)
        self.create_subscription(Bool, PROGRAM_TOPIC, lambda m: setattr(self, "program_running", m.data), latched)
        self.create_subscription(FaultStatus, FAULT_TOPIC, lambda m: setattr(self, "fault", m.status), 10)
        self.recover = self.create_client(Trigger, RECOVER_SERVICE)

    def spin_for(self, seconds, until=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not until():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_state(self, seconds):
        """Wait for the latched values (the fault status is optional)."""
        self.spin_for(seconds, lambda: None not in (self.robot_mode, self.safety_mode, self.program_running))

    def describe(self):
        def name(table, value):
            return "no message" if value is None else table.get(value, str(value))

        program = "no message" if self.program_running is None else ("running" if self.program_running else "stopped")
        return (
            f"robot mode {name(ROBOT_MODES, self.robot_mode)}, safety {name(SAFETY_MODES, self.safety_mode)}, "
            f"control program {program}, Pro fault status {name(FAULTS, self.fault)}"
        )

    def healthy(self):
        return (
            self.robot_mode == RobotMode.RUNNING
            and self.safety_mode in (SafetyMode.NORMAL, SafetyMode.REDUCED)
            and self.program_running is True
            and self.fault in (None, FaultStatus.NORMAL)
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--force", action="store_true",
                        help="call the recovery even when the arm looks healthy or the stop needs a person")
    parser.add_argument("--status", action="store_true", help="only report the arm's state")
    args = parser.parse_args()

    rclpy.init()
    node = ArmState()
    try:
        node.wait_for_state(3.0)
        print(f"before: {node.describe()}")
        if node.robot_mode is None:
            print(f"no robot state on {ROBOT_MODE_TOPIC}; is the UR driver running?")
            return 1
        if args.status:
            return 0 if node.healthy() else 1
        if node.safety_mode in NEEDS_A_PERSON and not args.force:
            print(f"not resetting: {NEEDS_A_PERSON[node.safety_mode]}. Clear that at the robot, then run this again.")
            return 1
        if node.robot_mode is not None and node.robot_mode != RobotMode.RUNNING and not args.force:
            print(
                "not resetting: the arm is not powered on with its brakes released. Power it on and release "
                "the brakes on the teach pendant (the brake release moves the arm slightly), then run this again."
            )
            return 1
        if node.healthy() and not args.force:
            print("the arm already looks healthy; nothing to reset. If the UI still shows a fault, run with --force.")
            return 0

        if not node.recover.wait_for_service(timeout_sec=5.0):
            print(f"{RECOVER_SERVICE} is not available; is the drivers container's protective_stop_manager_node up?")
            return 1
        print(f"calling {RECOVER_SERVICE} (this can take up to a minute)...")
        future = node.recover.call_async(Trigger.Request())
        node.spin_for(RECOVER_TIMEOUT_S, future.done)
        if not future.done():
            print(f"{RECOVER_SERVICE} did not answer within {RECOVER_TIMEOUT_S:.0f}s; check the drivers container's log.")
            return 1
        result = future.result()
        print(f"recovery {'succeeded' if result.success else 'FAILED'}{': ' + result.message if result.message else ''}")

        # The program and the fault status take a moment to catch up with the re-sent program.
        node.spin_for(10.0, node.healthy)
        print(f"after:  {node.describe()}")
        return 0 if result.success and node.healthy() else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
