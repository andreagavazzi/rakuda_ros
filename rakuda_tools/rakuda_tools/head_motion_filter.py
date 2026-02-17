#!/usr/bin/env python3
"""
head_motion_filter.py

A motion shaper / filter for a 2-DOF head gimbal commanded via
position_controllers/JointGroupPositionController.

- Subscribes:  /head_target  (Float64MultiArray)  [yaw, pitch]  rad
- Publishes:   /head_position_controller/commands (Float64MultiArray) [yaw, pitch] rad

Startup controller switch (optional, enabled by default):
  - Deactivate: head_controller
  - Activate:   head_position_controller
  This replicates the CLI behavior:
    ros2 control switch_controllers --deactivate head_controller --activate head_position_controller

Core features:
- Startup state machine: WAIT_JS -> HOMING -> TRACKING (optional homing)
- 1st order smoothing (tau) + velocity clamp + acceleration clamp
- Optional clamp to URDF kinematic joint limits (NOT ros2_control joints)
- Optional target timeout behavior (hold/home)
- Graceful shutdown (Ctrl+C) without rclpy waitset errors
"""

import math
import re
import signal
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import JointState

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

from builtin_interfaces.msg import Duration

try:
    # ros2_control
    from controller_manager_msgs.srv import SwitchController
except Exception:
    SwitchController = None


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _duration_from_seconds(t: float) -> Duration:
    """Convert float seconds to builtin_interfaces/Duration."""
    d = Duration()
    if t is None or t <= 0.0:
        d.sec = 0
        d.nanosec = 0
        return d
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    # Normalize
    if nanosec >= 1_000_000_000:
        sec += nanosec // 1_000_000_000
        nanosec = nanosec % 1_000_000_000
    d.sec = sec
    d.nanosec = nanosec
    return d


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

    # Fast path
    try:
        return float(s)
    except Exception:
        pass

    # Strip ${...}
    if s.startswith("${") and s.endswith("}"):
        s = s[2:-1].strip()

    allowed = {
        "pi": math.pi,
        "radians": math.radians,
        "deg2rad": lambda d: d * math.pi / 180.0,
        "abs": abs,
        "min": min,
        "max": max,
    }

    # safety filter
    if not re.match(r"^[0-9\.\+\-\*\/\(\)\s,a-zA-Z_]+$", s):
        raise ValueError(f"Unsupported numeric expression: {s}")

    return float(eval(s, {"__builtins__": {}}, allowed))


def _top_level_urdf_joints(root: ET.Element):
    """
    Return ONLY URDF kinematic joints: direct children of <robot>.
    This avoids ros2_control joints inside <ros2_control> blocks.
    """
    return [c for c in list(root) if _strip_xml_namespace(c.tag) == "joint"]


def parse_joint_limits_from_urdf(urdf_xml: str, joint_name: str, *, debug_log=None):
    """
    Returns (lower, upper) for the URDF kinematic joint (top-level).
    Ignores ros2_control joints.
    """
    root = ET.fromstring(urdf_xml)
    joints = _top_level_urdf_joints(root)

    matches = [j for j in joints if j.attrib.get("name") == joint_name]
    if not matches:
        if debug_log:
            names = [j.attrib.get("name", "") for j in joints if j.attrib.get("name", "")]
            similar = [n for n in names if joint_name in n or n in joint_name or "neck" in n]
            debug_log(f"URDF joint '{joint_name}' not found among top-level joints. Similar: {similar[:30]}")
        return None

    # If duplicates exist, pick first with usable limit
    for idx, j in enumerate(matches):
        jtype = j.attrib.get("type", "")
        if jtype == "continuous":
            continue

        limit_el = None
        for child in list(j):
            if _strip_xml_namespace(child.tag) == "limit":
                limit_el = child
                break
        if limit_el is None:
            continue

        lower_s = limit_el.attrib.get("lower", None)
        upper_s = limit_el.attrib.get("upper", None)
        if lower_s is None or upper_s is None:
            continue

        try:
            lower = _parse_float_expr(lower_s)
            upper = _parse_float_expr(upper_s)
            if debug_log and len(matches) > 1:
                debug_log(f"URDF joint '{joint_name}': using occurrence #{idx+1}/{len(matches)} with <limit>.")
            return lower, upper
        except Exception as e:
            if debug_log:
                debug_log(
                    f"URDF joint '{joint_name}' limit parse failed: lower='{lower_s}' upper='{upper_s}' err={e}"
                )
            continue

    if debug_log:
        debug_log(f"URDF joint '{joint_name}' found {len(matches)} time(s) but none had a usable <limit>.")
        for idx, j in enumerate(matches[:5]):
            child_tags = [_strip_xml_namespace(c.tag) for c in list(j)]
            debug_log(f"  occurrence #{idx+1}: type='{j.attrib.get('type','')}', children={child_tags}")

    return None


class HeadGimbalMotionFilter(Node):
    """
    Motion filter / shaper for a 2DOF head gimbal.

    State machine:
      - WAIT_JS: wait for /joint_states so internal state starts from real pose
      - HOMING: (optional) move to home pose
      - TRACKING: follow /head_target (filtered + limited)

    Optional URDF clamp:
      - Reads robot_description from /robot_state_publisher/get_parameters
        OR falls back to /robot_description topic.
      - Extracts limits ONLY from URDF top-level joints (ignores ros2_control joints).
    """

    def __init__(self):
        super().__init__("head_gimbal_motion_filter")

        # ---------------- Topics ----------------
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("target_topic", "/head_target")
        # Default changed to the position controller command topic, as we now
        # auto-switch to head_position_controller at startup (see parameters below).
        self.declare_parameter("command_topic", "/head_position_controller/commands")

        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)

        # ---------------- Controller switch at startup ----------------
        self.declare_parameter("auto_switch_controllers", True)
        self.declare_parameter("controller_manager_ns", "/controller_manager")
        self.declare_parameter("deactivate_controllers", ["head_controller"])
        self.declare_parameter("activate_controllers", ["head_position_controller"])
        self.declare_parameter("switch_strictness", "best_effort")  # best_effort|strict
        self.declare_parameter("activate_asap", True)
        self.declare_parameter("switch_timeout_sec", 5.0)
        self.declare_parameter("wait_for_switch_service_sec", 6.0)
        self.declare_parameter("switch_retry_period_sec", 1.0)

        self.auto_switch_controllers = bool(self.get_parameter("auto_switch_controllers").value)
        self.controller_manager_ns = str(self.get_parameter("controller_manager_ns").value).rstrip("/")
        self.deactivate_controllers = list(self.get_parameter("deactivate_controllers").value)
        self.activate_controllers = list(self.get_parameter("activate_controllers").value)
        self.switch_strictness = str(self.get_parameter("switch_strictness").value).lower().strip()
        self.activate_asap = bool(self.get_parameter("activate_asap").value)
        self.switch_timeout_sec = float(self.get_parameter("switch_timeout_sec").value)
        self.wait_for_switch_service_sec = float(self.get_parameter("wait_for_switch_service_sec").value)
        self.switch_retry_period_sec = float(self.get_parameter("switch_retry_period_sec").value)

        self._switch_done = (not self.auto_switch_controllers)
        self._switch_future = None
        self._switch_deadline = None
        self._next_switch_attempt_time = None
        self._switch_service_name = f"{self.controller_manager_ns}/switch_controller"
        self._switch_client = None

        # ---------------- Motion params ----------------
        self.declare_parameter("rate_hz", 60.0)
        self.declare_parameter("tau", 0.28)       # seconds
        self.declare_parameter("max_vel", 1.5)    # rad/s
        self.declare_parameter("max_acc", 6.0)    # rad/s^2

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.tau = float(self.get_parameter("tau").value)
        self.max_vel = float(self.get_parameter("max_vel").value)
        self.max_acc = float(self.get_parameter("max_acc").value)

        # ignore tiny changes in target
        self.declare_parameter("target_deadband_rad", 0.0)
        self.target_deadband = float(self.get_parameter("target_deadband_rad").value)

        # ---------------- Joints ----------------
        self.declare_parameter("yaw_joint", "neck_yaw_joint")
        self.declare_parameter("pitch_joint", "neck_pitch_joint")
        self.yaw_joint = str(self.get_parameter("yaw_joint").value)
        self.pitch_joint = str(self.get_parameter("pitch_joint").value)

        # ---------------- Startup / homing ----------------
        self.declare_parameter("home_on_start", True)
        self.declare_parameter("home_positions", [0.0, 0.0])   # [yaw, pitch]
        self.declare_parameter("home_tolerance", 0.02)         # rad
        self.declare_parameter("home_timeout_sec", 4.0)
        self.declare_parameter("wait_joint_states_timeout_sec", 3.0)
        self.declare_parameter("home_max_vel", 0.3)            # rad/s (<=0 -> max_vel)
        self.declare_parameter("hold_after_homing_until_new_target", True)

        self.home_on_start = bool(self.get_parameter("home_on_start").value)
        self.home_positions = [float(x) for x in self.get_parameter("home_positions").value]
        self.home_tol = float(self.get_parameter("home_tolerance").value)
        self.home_timeout = float(self.get_parameter("home_timeout_sec").value)
        self.wait_js_timeout = float(self.get_parameter("wait_joint_states_timeout_sec").value)

        home_max_vel_param = float(self.get_parameter("home_max_vel").value)
        self.home_max_vel = self.max_vel if home_max_vel_param <= 0.0 else home_max_vel_param

        self.hold_after_homing_until_new_target = bool(
            self.get_parameter("hold_after_homing_until_new_target").value
        )

        if len(self.home_positions) != 2:
            raise ValueError(f"home_positions must be [yaw, pitch], got {self.home_positions}")

        # ---------------- Target timeout behavior ----------------
        self.declare_parameter("target_timeout_sec", 0.0)     # 0 = disabled
        self.declare_parameter("on_target_timeout", "hold")   # hold|home

        self.target_timeout = float(self.get_parameter("target_timeout_sec").value)
        self.on_target_timeout = str(self.get_parameter("on_target_timeout").value).lower().strip()
        if self.on_target_timeout not in ("hold", "home"):
            self.on_target_timeout = "hold"

        # ---------------- Optional URDF limits clamp ----------------
        self.declare_parameter("clamp_to_urdf_limits", False)
        self.declare_parameter("robot_state_publisher_node", "robot_state_publisher")
        self.declare_parameter("robot_description_param", "robot_description")
        self.declare_parameter("robot_description_topic", "/robot_description")

        self.clamp_to_urdf_limits = bool(self.get_parameter("clamp_to_urdf_limits").value)
        self.robot_state_publisher_node = str(self.get_parameter("robot_state_publisher_node").value)
        self.robot_description_param = str(self.get_parameter("robot_description_param").value)
        self.robot_description_topic = str(self.get_parameter("robot_description_topic").value)

        # ---------------- Shutdown behavior ----------------
        self.declare_parameter("publish_hold_on_shutdown", True)
        self.publish_hold_on_shutdown = bool(self.get_parameter("publish_hold_on_shutdown").value)

        # ---------------- Internal state ----------------
        self.dt_nominal = 1.0 / max(1.0, self.rate_hz)

        # joint_states cache (avoid msg.name.index every time)
        self._have_js = False
        self._js_yaw = 0.0
        self._js_pitch = 0.0
        self._iy = None
        self._ip = None

        # commanded pose and velocities
        self.yaw = 0.0
        self.pitch = 0.0
        self.v_yaw = 0.0
        self.v_pitch = 0.0

        # target bookkeeping
        self._target_seq = 0
        self._target_seq_at_tracking_enable = 0
        self._last_target_yaw = self.home_positions[0]
        self._last_target_pitch = self.home_positions[1]
        self._last_target_time = self.get_clock().now()

        # phase
        self._t_start = self.get_clock().now()
        self._t_homing_start = None
        # We optionally gate startup behind a ros2_control controller switch.
        if self.auto_switch_controllers:
            self._phase = "WAIT_SWITCH"
        else:
            self._phase = "WAIT_JS" if self.home_on_start else "TRACKING"
            if self._phase == "TRACKING":
                self._target_seq_at_tracking_enable = self._target_seq

        # timing for real dt
        self._last_tick_time = None

        # URDF limits (default: no clamp)
        self.yaw_min = -math.inf
        self.yaw_max = +math.inf
        self.pitch_min = -math.inf
        self.pitch_max = +math.inf
        self._limits_loaded = False

        # ---------------- ROS interfaces ----------------
        self.sub_js = self.create_subscription(JointState, self.joint_states_topic, self.on_joint_states, 10)
        self.sub_target = self.create_subscription(Float64MultiArray, self.target_topic, self.on_target, 10)
        self.pub_cmd = self.create_publisher(Float64MultiArray, self.command_topic, 10)

        # URDF limits (optional)
        if self.clamp_to_urdf_limits:
            self._init_urdf_limits()

        # timer
        self.timer = self.create_timer(self.dt_nominal, self.tick)

        self.get_logger().info(
            f"[head_gimbal_motion_filter] rate={self.rate_hz}Hz tau={self.tau}s "
            f"max_vel={self.max_vel}rad/s max_acc={self.max_acc}rad/s^2 "
            f"| home_on_start={self.home_on_start} home={self.home_positions} tol={self.home_tol} "
            f"(home_max_vel={self.home_max_vel}) "
            f"| target_timeout={self.target_timeout}s on_timeout={self.on_target_timeout} "
            f"| clamp_to_urdf_limits={self.clamp_to_urdf_limits} "
            f"| auto_switch_controllers={self.auto_switch_controllers}"
        )

        # Kick off controller switch (non-blocking). We gate the state machine in tick()
        # until the switch has completed (or a timeout is reached).
        if self.auto_switch_controllers:
            if SwitchController is None:
                self.get_logger().error(
                    "auto_switch_controllers=True but controller_manager_msgs is not available. "
                    "Continuing WITHOUT switching controllers."
                )
                self._switch_done = True
                self._enter_post_switch(self.get_clock().now())
            else:
                self._switch_client = self.create_client(SwitchController, self._switch_service_name)
                now = self.get_clock().now()
                self._switch_deadline = now.nanoseconds / 1e9 + self.wait_for_switch_service_sec
                self._next_switch_attempt_time = now.nanoseconds / 1e9
                self.get_logger().info(
                    "Startup controller switch enabled: "
                    f"deactivate={self.deactivate_controllers} activate={self.activate_controllers} "
                    f"via {self._switch_service_name}"
                )

    # -------- URDF LIMITS --------
    def _init_urdf_limits(self):
        # Try remote param via standard GetParameters service; fallback to /robot_description topic.
        if self._try_load_urdf_from_param_service():
            self._limits_loaded = True
            return

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._urdf_sub = self.create_subscription(
            StringMsg, self.robot_description_topic, self._on_urdf_msg, qos
        )
        self.get_logger().info(f"URDF limits: param service not available, waiting on {self.robot_description_topic} ...")

    def _try_load_urdf_from_param_service(self) -> bool:
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
            f"URDF limits loaded from param service: {self.robot_state_publisher_node}.{self.robot_description_param}"
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
            self.get_logger().info("URDF limits loaded from topic.")
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

    # -------- Callbacks --------
    def on_joint_states(self, msg: JointState):
        # Cache indices the first time we can
        if self._iy is None or self._ip is None:
            try:
                self._iy = msg.name.index(self.yaw_joint)
                self._ip = msg.name.index(self.pitch_joint)
            except ValueError:
                return

        if self._iy < len(msg.position) and self._ip < len(msg.position):
            self._js_yaw = float(msg.position[self._iy])
            self._js_pitch = float(msg.position[self._ip])
            first = not self._have_js
            self._have_js = True

            # If we're still waiting for the controller switch, just cache joint states.
            if self._phase == "WAIT_SWITCH":
                return

            if first and self._phase == "WAIT_JS":
                # init internal state from real robot pose (avoid startup jump)
                self.yaw = self._js_yaw
                self.pitch = self._js_pitch
                self.v_yaw = 0.0
                self.v_pitch = 0.0
                self._t_homing_start = self.get_clock().now()
                self._phase = "HOMING"
                self.get_logger().info(
                    f"Got joint_states. Init yaw={self.yaw:.3f} pitch={self.pitch:.3f}. Entering HOMING."
                )

    def on_target(self, msg: Float64MultiArray):
        if len(msg.data) < 2:
            return
        yaw = float(msg.data[0])
        pitch = float(msg.data[1])

        # target deadband: ignore tiny changes
        if self.target_deadband > 0.0:
            if (abs(yaw - self._last_target_yaw) < self.target_deadband and
                    abs(pitch - self._last_target_pitch) < self.target_deadband):
                return

        # optional clamp to URDF limits
        if self.clamp_to_urdf_limits and self._limits_loaded:
            yaw = clamp(yaw, self.yaw_min, self.yaw_max)
            pitch = clamp(pitch, self.pitch_min, self.pitch_max)

        self._target_seq += 1
        self._last_target_yaw = yaw
        self._last_target_pitch = pitch
        self._last_target_time = self.get_clock().now()

    # -------- State machine helpers --------
    def _homing_done(self) -> bool:
        if self._have_js:
            ey = abs(self.home_positions[0] - self._js_yaw)
            ep = abs(self.home_positions[1] - self._js_pitch)
        else:
            ey = abs(self.home_positions[0] - self.yaw)
            ep = abs(self.home_positions[1] - self.pitch)
        return (ey <= self.home_tol) and (ep <= self.home_tol)

    def _enable_tracking(self):
        self._phase = "TRACKING"
        self._target_seq_at_tracking_enable = self._target_seq
        self.get_logger().info("Tracking enabled.")

    def _compute_dt(self, now):
        if self._last_tick_time is None:
            self._last_tick_time = now
            return self.dt_nominal
        dt = (now - self._last_tick_time).nanoseconds / 1e9
        self._last_tick_time = now
        return clamp(dt, 1e-4, 0.2)

    # -------- Main loop --------
    def tick(self):
        now = self.get_clock().now()
        dt = self._compute_dt(now)

        # 0) Controller switch gate: do this before anything else.
        if self._phase == "WAIT_SWITCH":
            self._tick_controller_switch(now)
            return

        # defaults: hold current pose
        target_yaw = self.yaw
        target_pitch = self.pitch
        vel_limit = self.max_vel

        # WAIT_JS fallback
        if self._phase == "WAIT_JS":
            elapsed = (now - self._t_start).nanoseconds / 1e9
            if elapsed >= self.wait_js_timeout:
                # Fallback init
                self.yaw = 0.0
                self.pitch = 0.0
                self.v_yaw = 0.0
                self.v_pitch = 0.0
                self._t_homing_start = now
                self._phase = "HOMING"
                self.get_logger().warn(
                    f"No joint_states after {self.wait_js_timeout}s. Fallback init yaw=0 pitch=0. Entering HOMING."
                )

        # HOMING
        if self._phase == "HOMING":
            target_yaw = self.home_positions[0]
            target_pitch = self.home_positions[1]
            vel_limit = self.home_max_vel

            homing_elapsed = (now - self._t_homing_start).nanoseconds / 1e9 if self._t_homing_start else 0.0
            if self._homing_done():
                self.get_logger().info("Homing complete.")
                self._enable_tracking()
            elif homing_elapsed >= self.home_timeout:
                self.get_logger().warn("Homing timeout. Enabling tracking anyway.")
                self._enable_tracking()

        # TRACKING
        if self._phase == "TRACKING":
            vel_limit = self.max_vel

            # optional timeout behavior
            timed_out = False
            if self.target_timeout > 0.0:
                t_since = (now - self._last_target_time).nanoseconds / 1e9
                if t_since >= self.target_timeout:
                    timed_out = True
                    if self.on_target_timeout == "home":
                        target_yaw = self.home_positions[0]
                        target_pitch = self.home_positions[1]
                    else:
                        target_yaw = self.yaw
                        target_pitch = self.pitch

            # hold-after-homing: ignore any old target until a new one arrives
            if self.hold_after_homing_until_new_target and self._target_seq <= self._target_seq_at_tracking_enable:
                target_yaw = self.home_positions[0]
                target_pitch = self.home_positions[1]
            else:
                if not timed_out:
                    target_yaw = self._last_target_yaw
                    target_pitch = self._last_target_pitch

        # optional clamp to URDF limits (even for homing)
        if self.clamp_to_urdf_limits and self._limits_loaded:
            target_yaw = clamp(target_yaw, self.yaw_min, self.yaw_max)
            target_pitch = clamp(target_pitch, self.pitch_min, self.pitch_max)

        # motion shaping: smoothing + vel clamp + acc clamp
        alpha = 1.0 if self.tau <= 1e-6 else (1.0 - math.exp(-dt / self.tau))
        yaw_f = self.yaw + alpha * (target_yaw - self.yaw)
        pitch_f = self.pitch + alpha * (target_pitch - self.pitch)

        v_yaw_des = (yaw_f - self.yaw) / dt
        v_pitch_des = (pitch_f - self.pitch) / dt

        v_yaw_des = clamp(v_yaw_des, -vel_limit, +vel_limit)
        v_pitch_des = clamp(v_pitch_des, -vel_limit, +vel_limit)

        max_dv = self.max_acc * dt
        self.v_yaw += clamp(v_yaw_des - self.v_yaw, -max_dv, +max_dv)
        self.v_pitch += clamp(v_pitch_des - self.v_pitch, -max_dv, +max_dv)

        self.yaw += self.v_yaw * dt
        self.pitch += self.v_pitch * dt

        # final safety clamp
        if self.clamp_to_urdf_limits and self._limits_loaded:
            self.yaw = clamp(self.yaw, self.yaw_min, self.yaw_max)
            self.pitch = clamp(self.pitch, self.pitch_min, self.pitch_max)

        # publish
        cmd = Float64MultiArray()
        cmd.data = [float(self.yaw), float(self.pitch)]
        self.pub_cmd.publish(cmd)

    # -------- Controller switching --------
    def _enter_post_switch(self, now):
        """Called once after switching controllers (or when switching is skipped)."""
        # Reset timers so WAIT_JS timeout is relative to *after* the switch.
        self._t_start = now
        self._last_tick_time = None

        # Initialize internal state from joint_states if we already have them.
        if self._have_js:
            self.yaw = float(self._js_yaw)
            self.pitch = float(self._js_pitch)
            self.v_yaw = 0.0
            self.v_pitch = 0.0

        if self.home_on_start:
            if self._have_js:
                self._t_homing_start = now
                self._phase = "HOMING"
                self.get_logger().info(
                    f"Controller switch done. Init from joint_states yaw={self.yaw:.3f} pitch={self.pitch:.3f}. "
                    "Entering HOMING."
                )
            else:
                self._phase = "WAIT_JS"
                self.get_logger().info("Controller switch done. Waiting for joint_states (WAIT_JS).")
        else:
            self._phase = "TRACKING"
            self._target_seq_at_tracking_enable = self._target_seq
            self.get_logger().info("Controller switch done. Entering TRACKING.")

    def _tick_controller_switch(self, now):
        """Non-blocking controller switch using /controller_manager/switch_controller."""
        if self._switch_done:
            # Shouldn't happen, but be safe.
            self._enter_post_switch(now)
            return

        now_s = now.nanoseconds / 1e9

        # If a request is in flight, wait for completion.
        if self._switch_future is not None:
            if self._switch_future.done():
                try:
                    res = self._switch_future.result()
                    ok = bool(getattr(res, "ok", False))
                except Exception as e:
                    ok = False
                    self.get_logger().warn(f"Controller switch call failed: {e}")

                self._switch_future = None
                if ok:
                    self._switch_done = True
                    self.get_logger().info(
                        f"Controllers switched (ok={ok}): deactivate={self.deactivate_controllers} "
                        f"activate={self.activate_controllers}"
                    )
                    self._enter_post_switch(now)
                else:
                    self.get_logger().warn("Controller switch returned ok=False (or failed). Will retry.")
                    self._next_switch_attempt_time = now_s + self.switch_retry_period_sec
            return

        # No in-flight request: check retry window.
        if self._next_switch_attempt_time is not None and now_s < self._next_switch_attempt_time:
            return

        if self._switch_client is None:
            self.get_logger().error("SwitchController client not initialized. Skipping controller switch.")
            self._switch_done = True
            self._enter_post_switch(now)
            return

        if not self._switch_client.service_is_ready():
            # Give it a small non-blocking poke.
            try:
                self._switch_client.wait_for_service(timeout_sec=0.0)
            except Exception:
                pass

        if not self._switch_client.service_is_ready():
            if self._switch_deadline is not None and now_s >= self._switch_deadline:
                self.get_logger().warn(
                    f"SwitchController service not available after {self.wait_for_switch_service_sec:.1f}s. "
                    "Continuing WITHOUT switching controllers."
                )
                self._switch_done = True
                self._enter_post_switch(now)
            return

        # Build and send request.
        req = SwitchController.Request()
        req.activate_controllers = list(self.activate_controllers)
        req.deactivate_controllers = list(self.deactivate_controllers)
        # Deprecated fields kept for compatibility
        req.start_controllers = list(self.activate_controllers)
        req.stop_controllers = list(self.deactivate_controllers)

        # strictness
        strict = 1  # BEST_EFFORT
        try:
            strict = int(getattr(SwitchController.Request, "BEST_EFFORT", 1))
            if self.switch_strictness == "strict":
                strict = int(getattr(SwitchController.Request, "STRICT", 2))
        except Exception:
            strict = 2 if self.switch_strictness == "strict" else 1
        req.strictness = strict

        # asap + timeout
        req.activate_asap = bool(self.activate_asap)
        req.start_asap = bool(self.activate_asap)  # deprecated
        req.timeout = _duration_from_seconds(self.switch_timeout_sec)

        self.get_logger().info(
            "Switching controllers... "
            f"deactivate={req.deactivate_controllers} activate={req.activate_controllers} "
            f"strictness={'STRICT' if strict == 2 else 'BEST_EFFORT'} "
            f"activate_asap={req.activate_asap} timeout={self.switch_timeout_sec:.1f}s"
        )
        try:
            self._switch_future = self._switch_client.call_async(req)
        except Exception as e:
            self.get_logger().warn(f"Failed to call SwitchController: {e}")
            self._next_switch_attempt_time = now_s + self.switch_retry_period_sec

    # -------- Graceful stop --------
    def stop_cleanly(self):
        try:
            if hasattr(self, "timer") and self.timer is not None:
                self.timer.cancel()
        except Exception:
            pass

        if self.publish_hold_on_shutdown:
            try:
                cmd = Float64MultiArray()
                cmd.data = [float(self.yaw), float(self.pitch)]
                self.pub_cmd.publish(cmd)
            except Exception:
                pass


def main():
    rclpy.init()
    node = HeadGimbalMotionFilter()
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
            node.stop_cleanly()
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
            node.stop_cleanly()
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

