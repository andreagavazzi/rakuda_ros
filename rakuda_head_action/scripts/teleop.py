#!/usr/bin/env python3
import math
import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import PointHead
from geometry_msgs.msg import PointStamped


class _RawTerminal:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get_key_nonblocking(self, timeout=0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return None


class HeadWASDContinuous(Node):
    def __init__(self):
        super().__init__("head_wasd_continuous")

        # --- Parameters
        self.action_name = self.declare_parameter("action_name", "/point_head").value
        self.frame_id = self.declare_parameter("frame_id", "torso_link").value
        self.pointing_frame = self.declare_parameter("pointing_frame", "camera_color_optical_frame").value

        self.min_duration_sec = float(self.declare_parameter("min_duration_sec", 0.25).value)
        self.max_velocity = float(self.declare_parameter("max_velocity", 2.0).value)

        self.r = float(self.declare_parameter("radius_m", 1.0).value)

        # "teleop rates" (deg/s)
        self.yaw_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate_deg = float(self.declare_parameter("yaw_rate_deg_s", 45.0).value)
        self.pitch_rate_deg = float(self.declare_parameter("pitch_rate_deg_s", 35.0).value)

        # loop rate
        self.hz = float(self.declare_parameter("hz", 15.0).value)
        self.dt = 1.0 / self.hz

        # timeout: if no keypress received for this long, stop
        self.stop_timeout = float(self.declare_parameter("stop_timeout_sec", 0.15).value)
        self.last_key_time = 0.0

        # angles
        self.yaw = 0.0
        self.pitch = 0.0

        # action client
        self.client = ActionClient(self, PointHead, self.action_name)
        self.get_logger().info(f"Connecting to action server: {self.action_name} ...")
        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("PointHead action server not available. Is head_action running?")
            raise RuntimeError("Action server not available")

        self.get_logger().info("Ready. Hold WASD to move continuously. Release to stop. Press q to quit.")
        self.get_logger().info("w=up, s=down, a=left, d=right")

        # send initial
        self.send_goal_from_angles()

    def _make_goal(self, x, y, z):
        goal = PointHead.Goal()

        ps = PointStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point.x = float(x)
        ps.point.y = float(y)
        ps.point.z = float(z)

        goal.target = ps
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.pointing_frame = self.pointing_frame

        sec = int(self.min_duration_sec)
        nsec = int((self.min_duration_sec - sec) * 1e9)
        goal.min_duration.sec = sec
        goal.min_duration.nanosec = nsec
        goal.max_velocity = float(self.max_velocity)
        return goal

    def send_goal_from_angles(self):
        # convert yaw/pitch to target point (x forward, y left, z up)
        x = self.r * math.cos(self.pitch) * math.cos(self.yaw)
        y = self.r * math.cos(self.pitch) * math.sin(self.yaw)
        z = self.r * math.sin(self.pitch)

        goal = self._make_goal(x, y, z)

        # fire-and-forget: we don't wait; head_action will accept and forward
        self.client.send_goal_async(goal)

    def update_rates_from_key(self, k: str):
        # This mimics turtlesim teleop: a keypress sets current "velocity".
        now = time.time()
        self.last_key_time = now

        yaw_rate = 0.0
        pitch_rate = 0.0

        if k in ("w", "W"):
            pitch_rate = +math.radians(self.pitch_rate_deg)
        elif k in ("s", "S"):
            pitch_rate = -math.radians(self.pitch_rate_deg)
        elif k in ("a", "A"):
            yaw_rate = +math.radians(self.yaw_rate_deg)   # +yaw = left
        elif k in ("d", "D"):
            yaw_rate = -math.radians(self.yaw_rate_deg)   # -yaw = right
        else:
            return

        self.yaw_rate = yaw_rate
        self.pitch_rate = pitch_rate

    def maybe_stop_if_no_key(self):
        if (time.time() - self.last_key_time) > self.stop_timeout:
            self.yaw_rate = 0.0
            self.pitch_rate = 0.0

    def tick(self):
        # called at fixed rate
        self.maybe_stop_if_no_key()

        if self.yaw_rate == 0.0 and self.pitch_rate == 0.0:
            return

        self.yaw += self.yaw_rate * self.dt
        self.pitch += self.pitch_rate * self.dt

        # keep bounded
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        self.pitch = max(min(self.pitch, math.radians(89.0)), math.radians(-89.0))

        self.send_goal_from_angles()


def main():
    rclpy.init()
    node = HeadWASDContinuous()

    try:
        with _RawTerminal() as rt:
            next_tick = time.time()
            while rclpy.ok():
                # pump callbacks
                rclpy.spin_once(node, timeout_sec=0.0)

                # read keys (non-blocking)
                k = rt.get_key_nonblocking(timeout=0.0)
                if k:
                    if k in ("q", "Q", "\x03"):
                        break
                    node.update_rates_from_key(k)

                # fixed-rate tick
                now = time.time()
                if now >= next_tick:
                    node.tick()
                    next_tick = now + node.dt

                # tiny sleep to avoid 100% CPU
                time.sleep(0.001)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
