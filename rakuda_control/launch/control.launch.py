from launch import LaunchDescription
from launch_ros.actions import Node

import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_share = get_package_share_directory('rakuda_description')
    control_share = get_package_share_directory('rakuda_control')

    xacro_file = os.path.join(description_share, 'urdf', 'rakuda.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    controllers_file = os.path.join(control_share, 'config', 'rakuda_controllers.yaml')

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_urdf},
            controllers_file
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    torso_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['torso_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    head_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        torso_controller_spawner,
        head_controller_spawner
    ])

