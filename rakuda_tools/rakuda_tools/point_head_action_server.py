#!/usr/bin/env python3
import math
import threading
from typing import Dict, Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse

from control_msgs.action import PointHead, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import tf2_ros
from tf2_geometry_msgs import do_transform_point  # sudo apt install ros-humble-tf2-geometry-msgs

from dynamixel_interfaces.srv import SetDataToDxl


def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def rotz_xy(x: float, y: float, yaw_rad: float) -> Tuple[float, float]:
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return (c * x - s * y, s * x + c * y)


class PointHeadActionServer(Node):
    """
    /point_head (PointHead) -> /head_controller/follow_joint_trajectory (FJT)

    - Preemption: newest goal wins (abort older)
    - Cancel: cancels the underlying FJT goal
    - Torque: set_dxl_data 'Torque Enable' on IDs [2,3] (only head)
    - Shutdown: torque OFF best-effort on exit

    Humble note:
    - PointHead.Result is empty
    - pointing_angle_error is in PointHead.Feedback
    """

    def __init__(self):
        super().__init__("point_head_action_server")

        # ---- params
        self.declare_parameter("point_head_action_name", "point_head")
        self.declare_parameter("follow_action_name", "/head_controller/follow_joint_trajectory")

        self.declare_parameter("yaw_joint", "neck_yaw_joint")
        self.declare_parameter("pitch_joint", "neck_pitch_joint")

        self.declare_parameter("yaw_parent_frame", "torso_link")
        self.declare_parameter("yaw_child_frame", "neck_yaw_link")
        self.declare_parameter("default_pointing_frame", "camera_color_optical_frame")

        self.declare_parameter("yaw_origin_xyz", [0.0, 0.0, 0.12555])      # torso_link
        self.declare_parameter("pitch_origin_xyz", [0.0, 0.0001, 0.02655]) # neck_yaw_link

        self.declare_parameter("yaw_min", -1.22173)
        self.declare_parameter("yaw_max",  1.22173)
        self.declare_parameter("pitch_min", -0.872665)
        self.declare_parameter("pitch_max",  0.610865)
        self.declare_parameter("pitch_sign", 1.0)

        # torque
        self.declare_parameter("auto_enable_torque", True)
        self.declare_parameter("auto_disable_torque_on_exit", True)
        self.declare_parameter("set_data_service", "/dynamixel_hardware_interface/set_dxl_data")
        self.declare_parameter("torque_item_name", "Torque Enable")
        self.declare_parameter("torque_ids", [2, 3])
        self.declare_parameter("torque_timeout_sec", 1.0)

        self.declare_parameter("torque_retry_period_sec", 1.0)
        self.declare_parameter("torque_retry_max_attempts", 10)

        self.declare_parameter("shutdown_wait_service_sec", 0.3)
        self.declare_parameter("shutdown_wait_each_call_sec", 0.5)

        self.declare_parameter("wait_loop_hz", 20.0)

        # ---- state
        self._js: Dict[str, float] = {}
        self._torque_ensured_once = False
        self._torque_attempts = 0
        self._torque_timer = None

        # preemption token
        self._seq_lock = threading.Lock()
        self._current_goal_seq = 0

        # ---- TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- subscribers
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)

        # ---- FJT client
        follow_name = self.get_parameter("follow_action_name").value
        self.fjt_client = ActionClient(self, FollowJointTrajectory, follow_name)

        # ---- set_dxl_data client
        set_data_srv = self.get_parameter("set_data_service").value
        self._dxl_cli = self.create_client(SetDataToDxl, set_data_srv)
        self._setdata_fields = self._detect_setdata_request_fields()
        self.get_logger().info(f"SetDataToDxl field map: {self._setdata_fields}")

        if bool(self.get_parameter("auto_enable_torque").value):
            period = float(self.get_parameter("torque_retry_period_sec").value)
            self._torque_timer = self.create_timer(period, self._try_enable_torque_startup)
            self.get_logger().info(
                f"Auto torque enable ON (IDs={self.get_parameter('torque_ids').value}) "
                f"via {set_data_srv} item='{self.get_parameter('torque_item_name').value}'"
            )

        # ---- Action server
        action_name = self.get_parameter("point_head_action_name").value
        self.server = ActionServer(
            self,
            PointHead,
            action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            handle_accepted_callback=self.handle_accepted_cb,
        )
        self.get_logger().info(f"PointHead server: /{action_name}  ->  {follow_name}")

    # ---------------------------
    # action server callbacks
    # ---------------------------
    def goal_cb(self, goal_request: PointHead.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel requested for /point_head")
        return CancelResponse.ACCEPT

    def handle_accepted_cb(self, goal_handle):
        with self._seq_lock:
            self._current_goal_seq += 1
            seq = self._current_goal_seq
        setattr(goal_handle, "_rakuda_seq", seq)
        goal_handle.execute()

    def _is_preempted(self, my_seq: int) -> bool:
        with self._seq_lock:
            return my_seq != self._current_goal_seq

    # ---------------------------
    # SetDataToDxl field autodetect
    # ---------------------------
    def _detect_setdata_request_fields(self) -> Dict[str, str]:
        req = SetDataToDxl.Request()
        fields = req.get_fields_and_field_types()

        header_field = "header" if "header" in fields else ""

        string_fields = [n for n, t in fields.items() if t == "string"]
        if not string_fields:
            raise RuntimeError("SetDataToDxl.Request has no string field")

        item_field = ""
        for cand in ["item_name", "item", "name", "address_name", "addr_name"]:
            if cand in string_fields:
                item_field = cand
                break
        if not item_field:
            item_field = string_fields[0]

        def is_int_type(t: str) -> bool:
            return any(k in t for k in ["int", "uint"]) and "Header" not in t

        def is_float_type(t: str) -> bool:
            return any(k in t for k in ["float", "double"])

        int_fields = [n for n, t in fields.items() if is_int_type(t)]
        float_fields = [n for n, t in fields.items() if is_float_type(t)]

        if not int_fields:
            raise RuntimeError("SetDataToDxl.Request has no integer fields")

        id_field = ""
        for n in int_fields:
            if "id" in n.lower():
                id_field = n
                break
        if not id_field:
            id_field = int_fields[0]

        value_field = ""
        for n in int_fields:
            if n == id_field:
                continue
            low = n.lower()
            if "value" in low or "data" in low:
                value_field = n
                break
        if not value_field:
            for n in int_fields:
                if n != id_field:
                    value_field = n
                    break
        if not value_field:
            raise RuntimeError("Cannot detect value field")

        timeout_field = ""
        for cand in ["timeout_sec", "timeout", "timeout_s", "timeoutSec"]:
            if cand in float_fields:
                timeout_field = cand
                break
        if not timeout_field and float_fields:
            timeout_field = float_fields[0]

        return {"header": header_field, "id": id_field, "item": item_field, "value": value_field, "timeout": timeout_field}

    # ---------------------------
    # joint states
    # ---------------------------
    def _on_joint_state(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self._js[n] = p

    def _get_joint_pos(self, joint: str) -> Optional[float]:
        return self._js.get(joint, None)

    # ---------------------------
    # torque helpers
    # ---------------------------
    def _torque_ids(self) -> List[int]:
        return [int(x) for x in self.get_parameter("torque_ids").value]

    def _build_setdata_req(self, dxl_id: int, item_name: str, value_int: int, timeout_sec: float) -> SetDataToDxl.Request:
        req = SetDataToDxl.Request()
        f = self._setdata_fields

        if f["header"]:
            h = getattr(req, f["header"])
            try:
                h.stamp.sec = 0
                h.stamp.nanosec = 0
                h.frame_id = ""
            except Exception:
                pass

        setattr(req, f["id"], int(dxl_id))
        setattr(req, f["item"], str(item_name))
        setattr(req, f["value"], int(value_int))
        if f["timeout"]:
            setattr(req, f["timeout"], float(timeout_sec))
        return req

    def _call_torque_enable_async(self, dxl_id: int, enable: bool):
        item = self.get_parameter("torque_item_name").value
        timeout = float(self.get_parameter("torque_timeout_sec").value)
        req = self._build_setdata_req(dxl_id, item, 1 if enable else 0, timeout)
        return self._dxl_cli.call_async(req)

    def _log_torque_resp(self, future, dxl_id: int, enable: bool):
        try:
            resp = future.result()
            ok = getattr(resp, "success", True)
            msg = getattr(resp, "message", "")
            if ok:
                self.get_logger().info(f"Torque {'ON' if enable else 'OFF'} OK for ID {dxl_id}. {msg}".strip())
            else:
                self.get_logger().warn(f"Torque {'ON' if enable else 'OFF'} FAILED for ID {dxl_id}. {msg}".strip())
        except Exception as e:
            self.get_logger().error(f"Torque {'ON' if enable else 'OFF'} error for ID {dxl_id}: {e}")

    def enable_head_torque_best_effort(self):
        if not self._dxl_cli.service_is_ready():
            return
        for dxl_id in self._torque_ids():
            fut = self._call_torque_enable_async(dxl_id, True)
            fut.add_done_callback(lambda f, i=dxl_id: self._log_torque_resp(f, i, True))

    def disable_head_torque_best_effort_sync(self) -> bool:
        if not bool(self.get_parameter("auto_disable_torque_on_exit").value):
            return True

        if self._torque_timer is not None:
            try:
                self._torque_timer.cancel()
            except Exception:
                pass

        wait_srv = float(self.get_parameter("shutdown_wait_service_sec").value)
        wait_each = float(self.get_parameter("shutdown_wait_each_call_sec").value)

        if not self._dxl_cli.wait_for_service(timeout_sec=wait_srv):
            self.get_logger().warn("Shutdown: set_dxl_data not available -> skipping torque OFF.")
            return False

        ok_all = True
        futures = []
        for dxl_id in self._torque_ids():
            futures.append((dxl_id, self._call_torque_enable_async(dxl_id, False)))

        for dxl_id, fut in futures:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=wait_each)
            if not fut.done():
                self.get_logger().warn(f"Shutdown: torque OFF timeout for ID {dxl_id}.")
                ok_all = False
            else:
                self._log_torque_resp(fut, dxl_id, False)

        return ok_all

    def _try_enable_torque_startup(self):
        max_attempts = int(self.get_parameter("torque_retry_max_attempts").value)
        self._torque_attempts += 1

        if self._torque_attempts > max_attempts:
            self.get_logger().warn("Torque startup retry exhausted; continuing.")
            if self._torque_timer is not None:
                self._torque_timer.cancel()
            return

        if not self._dxl_cli.service_is_ready():
            if self._torque_attempts in (1, 3, 5, 10):
                self.get_logger().warn("set_dxl_data not ready yet; retrying torque enable...")
            return

        self.enable_head_torque_best_effort()
        self._torque_ensured_once = True
        if self._torque_timer is not None:
            self._torque_timer.cancel()

    def _ensure_torque_before_motion(self):
        if not bool(self.get_parameter("auto_enable_torque").value):
            return
        if self._torque_ensured_once:
            return
        if not self._dxl_cli.service_is_ready():
            self.get_logger().warn("set_dxl_data not ready at first goal; cannot force torque yet.")
            return

        self.enable_head_torque_best_effort()
        self._torque_ensured_once = True

    # ---------------------------
    # feedback
    # ---------------------------
    def _publish_pointing_error_feedback(self, goal_handle, ang_err: float):
        fb = PointHead.Feedback()
        fb.pointing_angle_error = float(ang_err)
        goal_handle.publish_feedback(fb)

    # ---------------------------
    # execute
    # ---------------------------
    async def execute_cb(self, goal_handle):
        goal: PointHead.Goal = goal_handle.request
        my_seq = int(getattr(goal_handle, "_rakuda_seq", 0))

        if self._is_preempted(my_seq):
            goal_handle.abort()
            return PointHead.Result()

        self._ensure_torque_before_motion()

        if not self.fjt_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("FollowJointTrajectory action server not available.")
            self._publish_pointing_error_feedback(goal_handle, float("nan"))
            goal_handle.abort()
            return PointHead.Result()

        yaw_parent = self.get_parameter("yaw_parent_frame").value
        yaw_origin = self.get_parameter("yaw_origin_xyz").value
        pitch_origin = self.get_parameter("pitch_origin_xyz").value
        pitch_sign = float(self.get_parameter("pitch_sign").value)

        t = rclpy.time.Time()

        try:
            tf = self.tf_buffer.lookup_transform(
                yaw_parent,
                goal.target.header.frame_id,
                t,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            target_in_parent = do_transform_point(goal.target, tf)
        except Exception as e:
            self.get_logger().error(f"TF failed: target -> {yaw_parent}: {e}")
            self._publish_pointing_error_feedback(goal_handle, float("nan"))
            goal_handle.abort()
            return PointHead.Result()

        vx = target_in_parent.point.x - float(yaw_origin[0])
        vy = target_in_parent.point.y - float(yaw_origin[1])
        vz = target_in_parent.point.z - float(yaw_origin[2])

        yaw = math.atan2(vy, vx)
        vxp, vyp = rotz_xy(vx, vy, -yaw)
        vzp = vz

        rx = vxp - float(pitch_origin[0])
        ry = vyp - float(pitch_origin[1])
        rz = vzp - float(pitch_origin[2])

        pitch = pitch_sign * math.atan2(rz, rx)

        yaw = clamp(yaw, float(self.get_parameter("yaw_min").value), float(self.get_parameter("yaw_max").value))
        pitch = clamp(pitch, float(self.get_parameter("pitch_min").value), float(self.get_parameter("pitch_max").value))

        yaw_joint = self.get_parameter("yaw_joint").value
        pitch_joint = self.get_parameter("pitch_joint").value

        cur_yaw = self._get_joint_pos(yaw_joint)
        cur_pitch = self._get_joint_pos(pitch_joint)

        dy = abs(yaw - cur_yaw) if cur_yaw is not None else abs(yaw)
        dp = abs(pitch - cur_pitch) if cur_pitch is not None else abs(pitch)

        max_vel = float(goal.max_velocity) if goal.max_velocity > 0.0 else 1.0
        min_dur = float(goal.min_duration.sec) + float(goal.min_duration.nanosec) * 1e-9

        est = max(dy, dp) / max_vel
        dur = max(est, min_dur, 0.5)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return PointHead.Result()
        if self._is_preempted(my_seq):
            goal_handle.abort()
            return PointHead.Result()

        traj = JointTrajectory()
        traj.joint_names = [yaw_joint, pitch_joint]

        pt = JointTrajectoryPoint()
        pt.positions = [yaw, pitch]
        pt.time_from_start = rclpy.duration.Duration(seconds=dur).to_msg()
        traj.points = [pt]

        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.trajectory = traj

        fjt_goal_handle = await self.fjt_client.send_goal_async(fjt_goal)
        if not fjt_goal_handle.accepted:
            self.get_logger().error("FollowJointTrajectory goal rejected.")
            self._publish_pointing_error_feedback(goal_handle, float("nan"))
            goal_handle.abort()
            return PointHead.Result()

        result_future = fjt_goal_handle.get_result_async()

        hz = float(self.get_parameter("wait_loop_hz").value)
        dt = 1.0 / max(hz, 1.0)

        # Wait with rclpy spinning (no asyncio needed)
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                try:
                    await fjt_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                goal_handle.canceled()
                return PointHead.Result()

            if self._is_preempted(my_seq):
                try:
                    await fjt_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                goal_handle.abort()
                return PointHead.Result()

            rclpy.spin_until_future_complete(self, result_future, timeout_sec=dt)

        _ = await result_future

        pointing_frame = goal.pointing_frame if goal.pointing_frame else self.get_parameter("default_pointing_frame").value

        ax = float(goal.pointing_axis.x)
        ay = float(goal.pointing_axis.y)
        az = float(goal.pointing_axis.z)
        an = norm3(ax, ay, az)
        if an < 1e-9:
            ax, ay, az = 0.0, 0.0, 1.0
            an = 1.0

        try:
            tfp = self.tf_buffer.lookup_transform(
                pointing_frame,
                goal.target.header.frame_id,
                t,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            target_in_pointing = do_transform_point(goal.target, tfp)
            tx, ty, tz = target_in_pointing.point.x, target_in_pointing.point.y, target_in_pointing.point.z
            tn = norm3(tx, ty, tz)

            if tn < 1e-9:
                ang_err = 0.0
            else:
                dot = (ax * tx + ay * ty + az * tz) / (an * tn)
                dot = clamp(dot, -1.0, 1.0)
                ang_err = math.acos(dot)
        except Exception as e:
            self.get_logger().warn(f"TF failed for pointing_angle_error: {e}")
            ang_err = float("nan")

        self._publish_pointing_error_feedback(goal_handle, ang_err)

        if self._is_preempted(my_seq):
            goal_handle.abort()
            return PointHead.Result()

        goal_handle.succeed()
        return PointHead.Result()

    # ---------------------------
    # cleanup
    # ---------------------------
    def cleanup_on_exit(self):
        self.get_logger().info("Cleanup: disabling head torque (best effort)...")
        self.disable_head_torque_best_effort_sync()


def main():
    rclpy.init()
    node = PointHeadActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received.")
    finally:
        try:
            node.cleanup_on_exit()
        except Exception as e:
            try:
                node.get_logger().warn(f"Cleanup failed: {e}")
            except Exception:
                pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

