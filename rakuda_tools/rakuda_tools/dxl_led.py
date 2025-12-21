#!/usr/bin/env python3
import argparse
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from dynamixel_interfaces.srv import SetDataToDxl


class DxlLed(Node):
    def __init__(self, service_name: str):
        super().__init__("dxl_led")
        self.cli = self.create_client(SetDataToDxl, service_name)
        self.service_name = service_name

    def set_led(self, dxl_id: int, on: bool, item_name: str, timeout_sec: float) -> int:
        if not self.cli.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"Service not available: {self.service_name} (timeout {timeout_sec}s)"
            )
            return 2

        req = SetDataToDxl.Request()
        req.header = Header()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = ""

        req.id = int(dxl_id)
        req.item_name = item_name
        req.item_data = 1 if on else 0

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error(f"Service call timed out after {timeout_sec}s")
            return 3

        resp = future.result()
        if resp is None:
            self.get_logger().error("Service call failed (no response).")
            return 4

        state = "ON" if on else "OFF"
        if resp.result:
            self.get_logger().info(f"LED {state} for ID={dxl_id} (item='{item_name}').")
            return 0
        else:
            self.get_logger().error(f"LED {state} FAILED for ID={dxl_id} (item='{item_name}').")
            return 1


def parse_args():
    p = argparse.ArgumentParser(description="Turn ON/OFF the LED of a single Dynamixel.")
    p.add_argument("--id", type=int, required=True, help="Dynamixel ID (0..252)")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--on", action="store_true", help="LED ON")
    g.add_argument("--off", action="store_true", help="LED OFF")

    p.add_argument(
        "--item",
        default="LED",
        help="Control table item name (default: LED)",
    )
    p.add_argument(
        "--service",
        default="/dynamixel_hardware_interface/set_dxl_data",
        help="Service name",
    )
    p.add_argument("--timeout", type=float, default=3.0, help="Timeout seconds")
    return p.parse_args()


def main():
    args = parse_args()
    on = bool(args.on)

    rclpy.init()
    node = DxlLed(args.service)
    try:
        rc = node.set_led(dxl_id=args.id, on=on, item_name=args.item, timeout_sec=args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(rc)


if __name__ == "__main__":
    main()

