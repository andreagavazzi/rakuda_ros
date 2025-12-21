#!/usr/bin/env python3
import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class DxlTorqueAll(Node):
    def __init__(self, service_name: str):
        super().__init__("dxl_torque_all")
        self.cli = self.create_client(SetBool, service_name)
        self.service_name = service_name

    def set_torque(self, enable: bool, timeout_sec: float) -> int:
        if not self.cli.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"Service not available: {self.service_name} (timeout {timeout_sec}s)"
            )
            return 2

        req = SetBool.Request()
        req.data = enable

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error(f"Service call timed out after {timeout_sec}s")
            return 3

        resp = future.result()
        if resp is None:
            self.get_logger().error("Service call failed (no response).")
            return 4

        state = "ENABLED" if enable else "DISABLED"
        if resp.success:
            self.get_logger().info(f"Torque {state}. Message: {resp.message}")
            return 0
        else:
            self.get_logger().error(f"Torque request {state} FAILED. Message: {resp.message}")
            return 1


def parse_args():
    p = argparse.ArgumentParser(description="Enable/disable torque for all Dynamixel via dynamixel_hardware_interface.")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--on", action="store_true", help="Enable torque")
    g.add_argument("--off", action="store_true", help="Disable torque")
    p.add_argument(
        "--service",
        default="/dynamixel_hardware_interface/set_dxl_torque",
        help="Service name",
    )
    p.add_argument("--timeout", type=float, default=3.0, help="Timeout seconds")
    return p.parse_args()


def main():
    args = parse_args()
    enable = bool(args.on)

    rclpy.init()
    node = DxlTorqueAll(args.service)
    try:
        rc = node.set_torque(enable=enable, timeout_sec=args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(rc)


if __name__ == "__main__":
    main()

