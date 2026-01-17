#!/usr/bin/env python3

import math
import signal
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String as StringMsg

import tf2_ros


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    # Rotate vector v by quaternion q (x,y,z,w)  -> v' = q * v * q_conj
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    vpx = vx + qw * tx + (qy * tz - qz * ty)
    vpy = vy + qw * ty + (qz * tx - qx * tz)
    vpz = vz + qw * tz + (qx * ty - qy * tx)
    return vpx, vpy, vpz


def _strip_xml_namespace(tag: str) -> str:
    return tag.split("}", 1)[1] if "}" in tag else tag


def parse_joint_limits_from_urdf(urdf_xml: str, joint_name: str):
    """
    Returns (lower, upper) if present. If joint is 'continuous' or has no limits -> None.
    """
    root = ET.fromstring(urdf_xml)

    for joint in root.iter():
        if _strip_xml_namespace(joint.tag) != "joint":
            continue
        if joint.attrib.get("name") != joint_name:
            continue

        jtype = joint.attrib.get("type", "")
        if jtype == "continuous":
            return None

        limit_el = None
        for child in list(joint):
            if _strip_xml_namespace(child.tag) == "limit":
                limit_el = child
                break

        if limit_el is None:
            return None

        lower = limit_el.attrib.get("lower", None)
        upper = limit_el.attrib.get("upper", None)
        if lower is None or upper is None:
            return None

        return float(lower), float(upper)

    return None


class LookAtLinkTarget(Node):
    """
    TF tracking node that publishes yaw/pitch targets for a 2DOF head gimbal.

    Output:
      - Pub: /head_target (std_msgs/Float64MultiArray) [yaw, pitch] rad
        (this matches head_gimbal_stream input) :contentReference[oaicite:2]{index=2}
    """

    def __init__(self):
        super().__init__("look_at_link_target")

        # ---- Params (molto simili al tuo look_at_link.py) :contentReference[oaicite:3]{index=3}
        self.reference_frame = self.declare_parameter("reference_frame", "torso_link").value
        self.target_link = self.declare_parameter("target_link", "r_gripper_a").value
        self.pivot_link = self.declare_parameter("pivot_link", "neck_pitch_link").value

        self.hz = float(self.declare_parameter("hz", 20.0).value)
        self.deadband_m = float(self.declare_parameter("deadband_m", 0.005).value)

        # Offset expressed in TARGET LINK frame (meters), then rotated into reference_frame
        self.offset_x = float(self.declare_parameter("offset_x", 0.0).value)
        self.offset_y = float(self.declare_parameter("offset_y", 0.012).value)
        self.offset_z = float(self.declare_parameter("offset_z", 0.0).value)

        # Output topic (head_gimbal_stream subscribes /head_target) :contentReference[oaicite:4]{index=4}
        self.target_topic = self.declare_parameter("target_topic", "/head_target").value
        self.pub = self.create_publisher(Float64MultiArray, self.target_topic, 10)

        # Joint names for URDF limits (must match your robot)
        self.yaw_joint = self.declare_parameter("yaw_joint", "neck_yaw_joint").value
        self.pitch_joint = self.declare_parameter("pitch_joint", "neck_pitch_joint").value

        # Flip signs if needed
        self.yaw_dir = int(self.declare_parameter("yaw_dir", +1).value)
        self.pitch_dir = int(self.declare_parameter("pitch_dir", +1).value)

        # URDF sources
        self.robot_state_publisher_node = self.declare_parameter(
            "robot_state_publisher_node", "robot_state_publisher"
        ).value
        self.robot_description_param = self.declare_parameter("robot_description_param", "robot_description").value
        self.robot_description_topic = self.declare_parameter("robot_description_topic", "/robot_description").value

        # ---- Limits fallback (overwritten if URDF provides them)
        self.yaw_min, self.yaw_max = -math.pi, math.pi
        self.pitch_min, self.pitch_max = -math.pi / 2.0, math.pi / 2.0
        self._limits_loaded = False

        # ---- TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Load URDF limits (param -> fallback topic)
        if self._try_load_urdf_from_param():
            self._limits_loaded = True
        else:
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._urdf_sub = self.create_subscription(
                StringMsg, self.robot_description_topic, self._on_urdf_msg, qos
            )
            self.get_logger().info(
                f"URDF non letto da parametro. In attesa su topic: {self.robot_description_topic} ..."
            )

        self.get_logger().info(
            f"Tracking '{self.target_link}' from pivot '{self.pivot_link}' in '{self.reference_frame}' @ {self.hz} Hz "
            f"(deadband={self.deadband_m} m) -> publishing {self.target_topic}"
        )

        self.prev_xyz = None
        self.timer = self.create_timer(1.0 / max(1.0, self.hz), self.on_timer)

    def _try_load_urdf_from_param(self) -> bool:
        try:
            from rclpy.parameter_client import AsyncParametersClient

            client = AsyncParametersClient(self, self.robot_state_publisher_node)
            if not client.service_is_ready():
                client.wait_for_service(timeout_sec=0.5)

            fut = client.get_parameters([self.robot_description_param])
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
            if not fut.done() or fut.result() is None:
                return False

            vals = fut.result().values
            if not vals or vals[0].type_ == 0:  # NOT_SET
                return False

            urdf_xml = vals[0].string_value
            if not urdf_xml.strip():
                return False

            self._apply_limits_from_urdf(urdf_xml)
            self.get_logger().info(
                f"URDF letto da parametro: {self.robot_state_publisher_node}.{self.robot_description_param}"
            )
            return True

        except Exception as e:
            self.get_logger().warn(f"Impossibile leggere robot_description come parametro: {e}")
            return False

    def _on_urdf_msg(self, msg: StringMsg):
        if self._limits_loaded:
            return
        if not msg.data.strip():
            return
        try:
            self._apply_limits_from_urdf(msg.data)
            self._limits_loaded = True
            self.get_logger().info("URDF ricevuto da topic e limiti applicati.")
        except Exception as e:
            self.get_logger().error(f"Errore parsing URDF dal topic: {e}")

    def _apply_limits_from_urdf(self, urdf_xml: str):
        yaw_lim = parse_joint_limits_from_urdf(urdf_xml, self.yaw_joint)
        pitch_lim = parse_joint_limits_from_urdf(urdf_xml, self.pitch_joint)

        if yaw_lim is not None:
            self.yaw_min, self.yaw_max = yaw_lim
        else:
            self.get_logger().warn(f"Nessun limite URDF per {self.yaw_joint} (continuous o missing).")

        if pitch_lim is not None:
            self.pitch_min, self.pitch_max = pitch_lim
        else:
            self.get_logger().warn(f"Nessun limite URDF per {self.pitch_joint} (continuous o missing).")

        self.get_logger().info(
            f"Limiti URDF:\n"
            f"  {self.yaw_joint}:   [{self.yaw_min:.3f}, {self.yaw_max:.3f}]\n"
            f"  {self.pitch_joint}: [{self.pitch_min:.3f}, {self.pitch_max:.3f}]"
        )

    @staticmethod
    def dist(a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    def publish_target(self, yaw, pitch):
        msg = Float64MultiArray()
        msg.data = [float(yaw), float(pitch)]
        self.pub.publish(msg)

    def on_timer(self):
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

        # Vector from pivot -> target (reference frame)
        x = gx - px
        y = gy - py
        z = gz - pz

        xyz = (x, y, z)
        if self.prev_xyz is not None and self.dist(xyz, self.prev_xyz) < self.deadband_m:
            return
        self.prev_xyz = xyz

        # --- 2DOF gimbal IK
        yaw = math.atan2(y, x) * self.yaw_dir
        horiz = math.sqrt(x * x + y * y)
        pitch = math.atan2(z, horiz) * self.pitch_dir

        # Clamp to URDF limits (or fallback)
        yaw = clamp(yaw, self.yaw_min, self.yaw_max)
        pitch = clamp(pitch, self.pitch_min, self.pitch_max)

        self.publish_target(yaw, pitch)


def main():
    rclpy.init()
    node = LookAtLinkTarget()

    shutting_down = {"flag": False}

    def _shutdown(signum=None, frame=None):
        if shutting_down["flag"]:
            return
        shutting_down["flag"] = True
        try:
            node.get_logger().info("Ctrl+C: shutdown pulito...")
        except Exception:
            pass
        try:
            if hasattr(node, "timer") and node.timer is not None:
                node.timer.cancel()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    finally:
        try:
            if hasattr(node, "timer") and node.timer is not None:
                node.timer.cancel()
                node.destroy_timer(node.timer)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
