import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rakuda_head_action'),
        'config',
        'rakuda_head_action_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='rakuda_head_action',
            executable='rakuda_head_action_node',
            name='rakuda_head_action_node',
            output='screen',
            parameters=[config],
        ),
    ])
