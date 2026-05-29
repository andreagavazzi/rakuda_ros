#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PosePlayer(Node):
    def __init__(self):
        super().__init__("pose_player")
        self.declare_parameter("pose", "home")
        self.pose_name = self.get_parameter("pose").value

        self.yaml_path = os.path.join(
            get_package_share_directory("rakuda_tools"),
            "config",
            "initial_position.yaml",
        )

    def play(self):
        with open(self.yaml_path, "r") as f:
            data = yaml.safe_load(f)

        pose = data["poses"][self.pose_name]
        move_time = float(pose["move_time"])
        controllers = pose["controllers"]

        # Crea e attendi tutti gli action client
        jtc_clients = {}
        gripper_clients = {}

        for ctrl_name in controllers.keys():
            is_gripper = ctrl_name.endswith("_gripper_controller")
            if is_gripper:
                action_name = f"/{ctrl_name}/gripper_cmd"
                client = ActionClient(self, GripperCommand, action_name)
            else:
                action_name = f"/{ctrl_name}/follow_joint_trajectory"
                client = ActionClient(self, FollowJointTrajectory, action_name)

            self.get_logger().info(f"Waiting action server: {action_name}")
            if not client.wait_for_server(timeout_sec=3.0):
                raise RuntimeError(f"Action server not available: {action_name}")

            if is_gripper:
                gripper_clients[ctrl_name] = client
            else:
                jtc_clients[ctrl_name] = client

        # ── JTC: invia tutti i goal in parallelo ─────────────────────────────
        send_futures = {}
        for ctrl_name, client in jtc_clients.items():
            cfg = controllers[ctrl_name]
            joints = cfg["joints"]
            positions = cfg["positions"]

            traj = JointTrajectory()
            traj.joint_names = joints

            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.velocities = [0.0] * len(positions)
            pt.time_from_start.sec = int(move_time)
            pt.time_from_start.nanosec = int((move_time - int(move_time)) * 1e9)
            traj.points = [pt]

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            send_futures[ctrl_name] = client.send_goal_async(goal)

        result_futures = {}
        for ctrl_name, fut in send_futures.items():
            rclpy.spin_until_future_complete(self, fut)
            goal_handle = fut.result()
            if not goal_handle.accepted:
                raise RuntimeError(f"Goal rejected by {ctrl_name}")
            result_futures[ctrl_name] = goal_handle.get_result_async()

        for ctrl_name, fut in result_futures.items():
            rclpy.spin_until_future_complete(self, fut)
            status = fut.result().status
            self.get_logger().info(f"{ctrl_name} completed (status={status})")

        # ── Gripper: invia tutti i goal in parallelo ──────────────────────────
        gripper_send_futures = {}
        for ctrl_name, client in gripper_clients.items():
            cfg = controllers[ctrl_name]
            goal = GripperCommand.Goal()
            goal.command.position = float(cfg["positions"][0])
            goal.command.max_effort = float(cfg.get("max_effort", 1.5))
            gripper_send_futures[ctrl_name] = client.send_goal_async(goal)

        for ctrl_name, fut in gripper_send_futures.items():
            rclpy.spin_until_future_complete(self, fut)
            goal_handle = fut.result()
            if not goal_handle.accepted:
                raise RuntimeError(f"Gripper goal rejected by {ctrl_name}")
            result_fut = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_fut)
            self.get_logger().info(f"{ctrl_name} completed")


def main():
    import sys
    from rclpy.utilities import remove_ros_args

    # Supporta: ros2 run rakuda_tools pose_player <pose>
    # e anche:  ros2 run rakuda_tools pose_player --ros-args -p pose:=<pose>
    non_ros = remove_ros_args(sys.argv)
    pose_override = non_ros[1] if len(non_ros) > 1 else None

    rclpy.init()
    node = PosePlayer()
    if pose_override:
        node.pose_name = pose_override
    node.play()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
