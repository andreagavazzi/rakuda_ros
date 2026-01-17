#!/usr/bin/env python3
import math
import signal
import re
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String as StringMsg

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

import tf2_ros


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    # Rotate vector v by quaternion q (x,y,z,w)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    vpx = vx + qw * tx + (qy * tz - qz * ty)
    vpy = vy + qw * ty + (qz * tx - qx * tz)
    vpz = vz + qw * tz + (qx * ty - qy * tx)
    return vpx, vpy, vpz


def _strip_xml_namespace(tag: str) -> str:
    return tag.split("}", 1)[1] if "}" in tag else tag


def _parse_float_expr(s: str) -> float:
    """
    Parse numeric values from URDF/xacro.
    Supports:
      - "0.660"
      - "${0.660}"
      - "${radians(30)}"
      - "${pi/6}"
    """
    s = s.strip()

    # Fast path: plain float
    try:
        return float(s)
    except Exception:
        pass

    # Strip ${...}
    if s.startswith("${") and s.endswith("}"):
        s = s[2:-1].strip()

    import math as _math
    allowed = {
        "pi": _math.pi,
        "radians": _math.radians,
        "deg2rad": lambda d: d * _math.pi / 180.0,
        "abs": abs,
        "min": min,
        "max": max,
    }

    # Safety filter: allow only basic arithmetic and identifiers
    if not re.match(r"^[0-9\.\+\-\*\/\(\)\s,a-zA-Z_]+$", s):
        raise ValueError(f"Unsupported numeric expression: {s}")

    return float(eval(s, {"__builtins__": {}}, allowed))


def parse_joint_limits_from_urdf(urdf_xml: str, joint_name: str, *, debug_log=None):
    """
    Returns (lower, upper) if found. Otherwise None.
    debug_log: callable(str) for extra debug.
    """
    root = ET.fromstring(urdf_xml)

    found_joint = None
    for el in root.iter():
        if _strip_xml_namespace(el.tag) != "joint":
            continue
        if el.attrib.get("name") == joint_name:
            found_joint = el
            break

    if found_joint is None:
        if debug_log:
            names = []
            for el in root.iter():
                if _strip_xml_namespace(el.tag) == "joint":
                    n = el.attrib.get("name", "")
                    if n:
                        names.append(n)
            similar = [n for n in names if joint_name in n or n in joint_name or "neck" in n]
            debug_log(f"Joint '{joint_name}' not found. Similar joints: {similar[:30]}")
        return None

    jtype = found_joint.attrib.get("type", "")
    if jtype == "continuous":
        if debug_log:
            debug_log(f"Joint '{joint_name}' is continuous (no lower/upper limits).")
        return None

    limit_el = None
    for child in list(found_joint):
        if _strip_xml_namespace(child.tag) == "limit":
            limit_el = child
            break

    if limit_el is None:
        if debug_log:
            debug_log(f"Joint '{joint_name}' found but has no <limit> tag.")
        return None

    lower_s = limit_el.attrib.get("lower", None)
    upper_s = limit_el.attrib.get("upper", None)
    if lower_s is None or upper_s is None:
        if debug_log:
            debug_log(f"Joint '{joint_name}' <limit> missing lower/upper: {limit_el.attrib}")
        return None

    try:
        lower = _parse_float_expr(lower_s)
        upper = _parse_float_expr(upper_s)
        return lower, upper
    except Exception as e:
        if debug_log:
            debug_log(f"Joint '{joint_name}' limit parse failed: lower='{lower_s}' upper='{upper_s}' err={e}")
        return None


class LookAtLinkTarget(Node):
    """
    TF tracking node that publishes yaw/pitch targets for a 2DOF head gimbal.

    Output:
      - Pub: /head_target (std_msgs/Float64MultiArray) [yaw, pitch] rad
        (consumed by head_gimbal_motion_filter / head_gimbal_stream)
    """

    def __init__(self):
        super().__init__("look_at_link_target")

        # ---- Params (simili al tuo vecchio look_at_link.py)
        self.reference_frame = self.declare_parameter("reference_frame", "torso_link").value
        self.target_link = self.declare_parameter("target_link", "r_gripper_a").value
        self.pivot_link = self.declare_parameter("pivot_link", "neck_pitch_link").value

        self.hz = float(self.declare_parameter("hz", 20.0).value)
        self.deadband_m = float(self.declare_parameter("deadband_m", 0.005).value)

        # Offset expressed in TARGET LINK frame (meters), then rotated into reference_frame
        self.offset_x = float(self.declare_parameter("offset_x", 0.0).value)
        self.offset_y = float(self.declare_parameter("offset_y", 0.012).value)
        self.offset_z = float(self.declare_parameter("offset_z", 0.0).value)

        # Output topic
        self.target_topic = self.declare_parameter("target_topic", "/head_target").value
        self.pub = self.create_publisher(Float64MultiArray, self.target_topic, 10)

        # Joint names for URDF limits
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

        # Limits fallback (overwritten if URDF provides them)
        self.yaw_min, self.yaw_max = -math.pi, math.pi
        self.pitch_min, self.pitch_max = -math.pi / 2.0, math.pi / 2.0
        self._limits_loaded = False

        # ---- TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Load URDF limits (param service -> fallback topic)
        if self._try_load_urdf_from_param_service():
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
                f"URDF not read from param service. Waiting on topic: {self.robot_description_topic} ..."
            )

        self.get_logger().info(
            f"Tracking '{self.target_link}' from pivot '{self.pivot_link}' in '{self.reference_frame}' "
            f"@ {self.hz} Hz (deadband={self.deadband_m} m) -> publishing {self.target_topic}"
        )

        self.prev_xyz = None
        self.timer = self.create_timer(1.0 / max(1.0, self.hz), self.on_timer)

    def _try_load_urdf_from_param_service(self) -> bool:
        """
        Read robot_description from a remote node using the standard GetParameters service.
        Works even if rclpy.parameter_client is missing.
        """
        service_name = f"/{self.robot_state_publisher_node}/get_parameters"
        client = self.create_client(GetParameters, service_name)

        if not client.wait_for_service(timeout_sec=0.8):
            return False

        req = GetParameters.Request()
        req.names = [self.robot_description_param]
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

        res = fut.result()
        if res is None or not res.values:
            return False

        val = res.values[0]
        if val.type != ParameterType.PARAMETER_STRING:
            return False

        urdf_xml = val.string_value
        if not urdf_xml.strip():
            return False

        self._apply_limits_from_urdf(urdf_xml)
        self.get_logger().info(
            f"URDF read from param service: {self.robot_state_publisher_node}.{self.robot_description_param}"
        )
        return True

    def _on_urdf_msg(self, msg: StringMsg):
        if self._limits_loaded:
            return
        if not msg.data.strip():
            return
        try:
            self._apply_limits_from_urdf(msg.data)
            self._limits_loaded = True
            self.get_logger().info("URDF received from topic; limits applied.")
        except Exception as e:
            self.get_logger().error(f"URDF topic parse failed: {e}")

    def _apply_limits_from_urdf(self, urdf_xml: str):
        dbg = lambda s: self.get_logger().warn("[URDF DEBUG] " + s)

        yaw_lim = parse_joint_limits_from_urdf(urdf_xml, self.yaw_joint, debug_log=dbg)
        pitch_lim = parse_joint_limits_from_urdf(urdf_xml, self.pitch_joint, debug_log=dbg)

        if yaw_lim is not None:
            self.yaw_min, self.yaw_max = yaw_lim
        else:
            self.get_logger().warn(f"No URDF limits for {self.yaw_joint} (continuous/missing/parse).")

        if pitch_lim is not None:
            self.pitch_min, self.pitch_max = pitch_lim
        else:
            self.get_logger().warn(f"No URDF limits for {self.pitch_joint} (continuous/missing/parse).")

        self.get_logger().info(
            "URDF limits:\n"
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
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    node = LookAtLinkTarget()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    stop = {"flag": False}

    def _request_stop(signum=None, frame=None):
        if stop["flag"]:
            return
        stop["flag"] = True
        try:
            node.get_logger().info("Shutdown requested (Ctrl+C).")
        except Exception:
            pass
        try:
            if hasattr(node, "timer") and node.timer is not None:
                node.timer.cancel()
        except Exception:
            pass
        try:
            executor.wake()
        except Exception:
            pass

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        while rclpy.ok() and not stop["flag"]:
            executor.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        _request_stop()
    finally:
        try:
            if hasattr(node, "timer") and node.timer is not None:
                node.timer.cancel()
                node.destroy_timer(node.timer)
        except Exception:
            pass
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
