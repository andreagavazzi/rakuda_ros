#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray


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

        # Publisher fisso per head_controller (JointGroupPositionController)
        self.head_pub = self.create_publisher(Float64MultiArray, "/head_controller/commands", 10)

    def play(self):
        with open(self.yaml_path, "r") as f:
            data = yaml.safe_load(f)

        pose = data["poses"][self.pose_name]
        move_time = float(pose["move_time"])
        controllers = pose["controllers"]

        # 1) invia subito head (topic) se presente
        if "head_controller" in controllers:
            cfg = controllers["head_controller"]
            positions = cfg["positions"]  # ordine = come da controller config
            self.head_pub.publish(Float64MultiArray(data=positions))
            self.get_logger().info("Head command published on /head_controller/commands")

        # 2) prepara action client SOLO per i controller JTC (tutti tranne head_controller)
        clients = {}
        for ctrl_name in controllers.keys():
            if ctrl_name == "head_controller":
                continue

            action_name = f"/{ctrl_name}/follow_joint_trajectory"
            client = ActionClient(self, FollowJointTrajectory, action_name)
            self.get_logger().info(f"Waiting action server: {action_name}")

            if not client.wait_for_server(timeout_sec=3.0):
                raise RuntimeError(f"Action server not available: {action_name}")

            clients[ctrl_name] = client

        # 3) invia i goal action
        send_futures = {}
        for ctrl_name, client in clients.items():
            cfg = controllers[ctrl_name]
            joints = cfg["joints"]
            positions = cfg["positions"]

            traj = JointTrajectory()
            traj.joint_names = joints

            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.time_from_start.sec = int(move_time)
            pt.time_from_start.nanosec = int((move_time - int(move_time)) * 1e9)
            traj.points = [pt]

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj

            send_futures[ctrl_name] = client.send_goal_async(goal)

        # 4) aspetta accettazione + risultato
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


def main():
    rclpy.init()
    node = PosePlayer()
    node.play()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
