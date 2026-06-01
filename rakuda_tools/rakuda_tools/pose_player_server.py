#!/usr/bin/env python3
import os
import socket
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

SOCK_PATH = "/tmp/pose_player.sock"


class PosePlayerServer(Node):
    def __init__(self):
        super().__init__("pose_player_server")

        yaml_path = os.path.join(
            get_package_share_directory("rakuda_tools"),
            "config",
            "initial_position.yaml",
        )
        with open(yaml_path) as f:
            self._data = yaml.safe_load(f)

        poses = list(self._data["poses"].keys())
        self.get_logger().info(f"Available poses: {', '.join(poses)}")

        # Collect all unique controllers across all poses
        all_controllers = {}
        for pose_data in self._data["poses"].values():
            for ctrl_name in pose_data["controllers"]:
                all_controllers[ctrl_name] = ctrl_name.endswith("_gripper_controller")

        # Create all action clients upfront, then wait for all simultaneously
        self._jtc_clients = {}
        self._gripper_clients = {}
        all_clients = {}

        for ctrl_name, is_gripper in all_controllers.items():
            if is_gripper:
                action_name = f"/{ctrl_name}/gripper_cmd"
                client = ActionClient(self, GripperCommand, action_name)
            else:
                action_name = f"/{ctrl_name}/follow_joint_trajectory"
                client = ActionClient(self, FollowJointTrajectory, action_name)
            all_clients[ctrl_name] = (client, is_gripper, action_name)

        for ctrl_name, (client, is_gripper, action_name) in all_clients.items():
            self.get_logger().info(f"Waiting for: {action_name}")
            if not client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f"Action server not available: {action_name}")
            if is_gripper:
                self._gripper_clients[ctrl_name] = client
            else:
                self._jtc_clients[ctrl_name] = client

        self.get_logger().info("All action servers ready")
        self._busy = False
        self._srv_socket = None

        # Start UNIX socket listener
        threading.Thread(target=self._socket_server, daemon=True).start()
        self.get_logger().info(f"Listening on {SOCK_PATH}")

    def _socket_server(self):
        if os.path.exists(SOCK_PATH):
            os.unlink(SOCK_PATH)
        self._srv_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._srv_socket.bind(SOCK_PATH)
        self._srv_socket.listen(4)
        while True:
            try:
                conn, _ = self._srv_socket.accept()
                threading.Thread(target=self._handle_conn, args=(conn,), daemon=True).start()
            except Exception:
                break

    def shutdown(self):
        print("[pose_player_server] Shutting down...")
        if self._srv_socket:
            self._srv_socket.close()
        if os.path.exists(SOCK_PATH):
            os.unlink(SOCK_PATH)

    def _handle_conn(self, conn):
        with conn:
            pose_name = conn.recv(256).decode().strip()
            if pose_name not in self._data["poses"]:
                available = ", ".join(self._data["poses"].keys())
                conn.send(f"ERR unknown pose. Available: {available}".encode())
                return
            if self._busy:
                conn.send(b"ERR busy")
                return
            conn.send(b"OK")
            threading.Thread(target=self._execute, args=(pose_name,), daemon=True).start()

    def _on_jtc_goal(self, future, event, ctrl_name):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(f"Goal rejected by {ctrl_name}")
            event.set()
            return
        gh.get_result_async().add_done_callback(
            lambda f: (self.get_logger().info(f"{ctrl_name} done"), event.set())
        )

    def _on_gripper_goal(self, future, event, ctrl_name):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(f"Gripper goal rejected by {ctrl_name}")
            event.set()
            return
        gh.get_result_async().add_done_callback(
            lambda f: (self.get_logger().info(f"{ctrl_name} done"), event.set())
        )

    def _execute(self, pose_name):
        self._busy = True
        try:
            pose = self._data["poses"][pose_name]
            move_time = float(pose["move_time"])
            controllers = pose["controllers"]
            self.get_logger().info(f"Executing '{pose_name}'")

            jtc = {k: v for k, v in self._jtc_clients.items() if k in controllers}
            gripper = {k: v for k, v in self._gripper_clients.items() if k in controllers}

            done_events = {}
            for ctrl_name, client in jtc.items():
                cfg = controllers[ctrl_name]
                traj = JointTrajectory()
                traj.joint_names = cfg["joints"]
                pt = JointTrajectoryPoint()
                pt.positions = cfg["positions"]
                pt.velocities = [0.0] * len(cfg["positions"])
                pt.time_from_start.sec = int(move_time)
                pt.time_from_start.nanosec = int((move_time - int(move_time)) * 1e9)
                traj.points = [pt]

                goal = FollowJointTrajectory.Goal()
                goal.trajectory = traj

                ev = threading.Event()
                done_events[ctrl_name] = ev
                client.send_goal_async(goal).add_done_callback(
                    lambda f, e=ev, c=ctrl_name: self._on_jtc_goal(f, e, c)
                )

            for ctrl_name, ev in done_events.items():
                if not ev.wait(timeout=move_time + 5.0):
                    self.get_logger().warn(f"{ctrl_name} timed out")

            for ctrl_name, client in gripper.items():
                cfg = controllers[ctrl_name]
                goal = GripperCommand.Goal()
                goal.command.position = float(cfg["positions"][0])
                goal.command.max_effort = float(cfg.get("max_effort", 1.5))

                ev = threading.Event()
                client.send_goal_async(goal).add_done_callback(
                    lambda f, e=ev, c=ctrl_name: self._on_gripper_goal(f, e, c)
                )
                if not ev.wait(timeout=move_time + 5.0):
                    self.get_logger().warn(f"{ctrl_name} timed out")

            self.get_logger().info(f"Pose '{pose_name}' complete")
        finally:
            self._busy = False


def main():
    rclpy.init()
    node = PosePlayerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
