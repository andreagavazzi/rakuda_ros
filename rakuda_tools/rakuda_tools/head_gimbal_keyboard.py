#!/usr/bin/env python3
import sys
import math
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class HeadGimbalKeyboard(Node):
    """
    WASD/ASWD keyboard teleop for head_gimbal.
    Publishes [yaw, pitch] as Float64MultiArray.

    Keys:
      W: pitch up
      S: pitch down
      A: yaw left
      D: yaw right
      SPACE: center (0,0)
      X: exit
      H: help
    """

    def __init__(self):
        super().__init__("head_gimbal_keyboard")

        # ---------------- CONFIG (cambia qui se serve) ----------------
        self.target_topic = self.declare_parameter("target_topic", "/head_target").value
        self.step = float(self.declare_parameter("step_rad", 0.03).value)          # ~2.9 deg
        self.yaw_min = float(self.declare_parameter("yaw_min", -math.radians(90)).value)
        self.yaw_max = float(self.declare_parameter("yaw_max",  math.radians(90)).value)
        self.pitch_min = float(self.declare_parameter("pitch_min", -math.radians(45)).value)
        self.pitch_max = float(self.declare_parameter("pitch_max",  math.radians(45)).value)

        # Direzioni (inverti a +1/-1 se ti muove “al contrario”)
        self.yaw_dir = int(self.declare_parameter("yaw_dir", +1).value)     # A -> +step * yaw_dir, D -> -step * yaw_dir
        self.pitch_dir = int(self.declare_parameter("pitch_dir", +1).value) # W -> +step * pitch_dir, S -> -step * pitch_dir
        # ----------------------------------------------------------------

        self.pub = self.create_publisher(Float64MultiArray, self.target_topic, 10)

        self.yaw = 0.0
        self.pitch = 0.0

        self._print_help()
        self._publish()

        # timer “veloce” per leggere tastiera senza bloccare ROS
        self.timer = self.create_timer(0.02, self._tick)

    def _print_help(self):
        self.get_logger().info(
            "\n--- Head gimbal keyboard ---\n"
            "W/S: pitch su/giu\n"
            "A/D: yaw sinistra/destra\n"
            "SPACE: centro (0,0)\n"
            "H: help\n"
            "X: esci\n"
            "Params: target_topic, step_rad, yaw_min/max, pitch_min/max, yaw_dir, pitch_dir\n"
        )

    def _publish(self):
        msg = Float64MultiArray()
        msg.data = [float(self.yaw), float(self.pitch)]
        self.pub.publish(msg)
        self.get_logger().info(f"Target -> yaw={self.yaw:.3f} rad | pitch={self.pitch:.3f} rad")

    def _read_key(self):
        # Non-blocking single char read
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    def _tick(self):
        k = self._read_key()
        if not k:
            return

        k = k.lower()

        changed = False

        if k == "w":
            self.pitch = clamp(self.pitch + self.step * self.pitch_dir, self.pitch_min, self.pitch_max)
            changed = True
        elif k == "s":
            self.pitch = clamp(self.pitch - self.step * self.pitch_dir, self.pitch_min, self.pitch_max)
            changed = True
        elif k == "a":
            self.yaw = clamp(self.yaw + self.step * self.yaw_dir, self.yaw_min, self.yaw_max)
            changed = True
        elif k == "d":
            self.yaw = clamp(self.yaw - self.step * self.yaw_dir, self.yaw_min, self.yaw_max)
            changed = True
        elif k == " ":
            self.yaw, self.pitch = 0.0, 0.0
            changed = True
        elif k == "h":
            self._print_help()
        elif k == "x":
            self.get_logger().info("Uscita richiesta (X).")
            rclpy.shutdown()
            return

        if changed:
            self._publish()


def main():
    # Metti terminale in raw mode e ripristina alla fine
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        rclpy.init()
        node = HeadGimbalKeyboard()
        rclpy.spin(node)
        node.destroy_node()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()
