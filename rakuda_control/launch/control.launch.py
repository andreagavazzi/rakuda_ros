from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    control_share = get_package_share_directory('rakuda_control')
    controllers_file = os.path.join(control_share, 'config', 'rakuda_controllers.yaml')
    
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        remappings=[('~/robot_description', '/robot_description')],
        output='log',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        output='log'
    )

    torso_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_torso_controller',
        arguments=['torso_controller', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        output='log'
    )

    head_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_head_controller',
        arguments=['head_controller', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        output='log'
    )

    # Disabilita torque dai Dynamixel delle braccia
    torque_except_node = Node(
        package="rakuda_tools",
        executable="torque_except",
        name="torque_except",
        output="log",
    )

    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        torso_controller_spawner,
        head_controller_spawner,
        torque_except_node,
    ])

