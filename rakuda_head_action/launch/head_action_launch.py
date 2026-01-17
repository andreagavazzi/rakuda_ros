from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("rakuda_head_action")
    params = os.path.join(pkg, "config", "rakuda_head_action.yaml")

    return LaunchDescription([
        Node(
            package="rakuda_head_action",
            executable="head_action_node",
            name="head_action",
            output="screen",
            parameters=[params],
        )
    ])