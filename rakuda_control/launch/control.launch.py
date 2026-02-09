from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 1. Recupera i path
    control_share = get_package_share_directory('rakuda_control')
    description_share = get_package_share_directory('rakuda_description') # Necessario per lo xacro
    
    # 2. Processa lo Xacro (uguale a description.launch.py)
    xacro_file = os.path.join(description_share, 'urdf', 'rakuda.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    controllers_file = os.path.join(control_share, 'config', 'rakuda_controllers.yaml')
    
    # 3. Passa robot_description come PARAMETRO, non solo remapping
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,    # <--- FONDAMENTALE
            controllers_file
        ],
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

