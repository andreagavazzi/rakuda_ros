#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import PointHead
from geometry_msgs.msg import PointStamped

import tf2_ros


def quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    # Rotate vector v by quaternion q (x,y,z,w)
    # v' = q * v * q_conj
    # Implemented with quaternion-vector math (fast, no deps)
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + qw * t + cross(q.xyz, t)
    vpx = vx + qw * tx + (qy * tz - qz * ty)
    vpy = vy + qw * ty + (qz * tx - qx * tz)
    vpz = vz + qw * tz + (qx * ty - qy * tx)
    return vpx, vpy, vpz


class LookAtLink(Node):
    def __init__(self):
        super().__init__("look_at_link")

        # ---- Params
        self.action_name = self.declare_parameter("action_name", "/point_head").value
        self.reference_frame = self.declare_parameter("reference_frame", "torso_link").value
        self.target_link = self.declare_parameter("target_link", "r_gripper_a").value

        # Pivot (head rotation reference): IMPORTANT for correct pitch when target is high/low
        self.pivot_link = self.declare_parameter("pivot_link", "neck_pitch_link").value

        self.pointing_frame = self.declare_parameter("pointing_frame", "camera_color_optical_frame").value
        self.axis_z = self.declare_parameter("use_axis_z", True).value  # True => axis (0,0,1)

        self.hz = float(self.declare_parameter("hz", 20.0).value)
        self.min_duration = float(self.declare_parameter("min_duration_sec", 0.40).value)
        self.max_velocity = float(self.declare_parameter("max_velocity", 2.0).value)

        # deadband: only send if target moved more than this (meters)
        self.deadband_m = float(self.declare_parameter("deadband_m", 0.005).value)

        # Offset expressed in TARGET LINK frame (meters), then rotated into reference_frame
        self.offset_x = float(self.declare_parameter("offset_x", 0.0).value)
        self.offset_y = float(self.declare_parameter("offset_y", 0.012).value) #centra rispetto ai gripper
        self.offset_z = float(self.declare_parameter("offset_z", 0.0).value)

        # ---- TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Action client
        self.client = ActionClient(self, PointHead, self.action_name)
        self.get_logger().info(f"Waiting for action server {self.action_name} ...")
        if not self.client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f"Action server not available: {self.action_name}")

        self.get_logger().info(
            f"Tracking '{self.target_link}' from pivot '{self.pivot_link}' in '{self.reference_frame}' @ {self.hz} Hz "
            f"(deadband={self.deadband_m} m)"
        )

        self.prev_xyz = None
        self.timer = self.create_timer(1.0 / self.hz, self.on_timer)

    def build_goal(self, x, y, z):
        goal = PointHead.Goal()

        ps = PointStamped()
        ps.header.frame_id = self.reference_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point.x = float(x)
        ps.point.y = float(y)
        ps.point.z = float(z)
        goal.target = ps

        # Standard PointHead fields (your head_action may ignore these, but keep them coherent)
        if self.axis_z:
            goal.pointing_axis.x = 0.0
            goal.pointing_axis.y = 0.0
            goal.pointing_axis.z = 1.0
        else:
            goal.pointing_axis.x = 1.0
            goal.pointing_axis.y = 0.0
            goal.pointing_axis.z = 0.0
        goal.pointing_frame = self.pointing_frame

        sec = int(self.min_duration)
        nsec = int((self.min_duration - sec) * 1e9)
        goal.min_duration.sec = sec
        goal.min_duration.nanosec = nsec
        goal.max_velocity = float(self.max_velocity)
        return goal

    @staticmethod
    def dist(a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def on_timer(self):
        # We need:
        #  - T_ref_target : reference_frame <- target_link
        #  - T_ref_pivot  : reference_frame <- pivot_link
        try:
            tf_t = self.tf_buffer.lookup_transform(self.reference_frame, self.target_link, rclpy.time.Time())
            tf_p = self.tf_buffer.lookup_transform(self.reference_frame, self.pivot_link, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # target origin in reference
        gx = tf_t.transform.translation.x
        gy = tf_t.transform.translation.y
        gz = tf_t.transform.translation.z

        # pivot origin in reference
        px = tf_p.transform.translation.x
        py = tf_p.transform.translation.y
        pz = tf_p.transform.translation.z

        # Apply offset in target_link frame, rotated into reference_frame using target quaternion
        ox, oy, oz = self.offset_x, self.offset_y, self.offset_z
        if abs(ox) > 1e-12 or abs(oy) > 1e-12 or abs(oz) > 1e-12:
            q = tf_t.transform.rotation
            rox, roy, roz = quat_rotate(q.x, q.y, q.z, q.w, ox, oy, oz)
            gx += rox
            gy += roy
            gz += roz

        # Vector from pivot -> target, expressed in reference_frame
        x = gx - px
        y = gy - py
        z = gz - pz

        xyz = (x, y, z)
        if self.prev_xyz is not None and self.dist(xyz, self.prev_xyz) < self.deadband_m:
            return
        self.prev_xyz = xyz

        goal = self.build_goal(x, y, z)
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = LookAtLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
