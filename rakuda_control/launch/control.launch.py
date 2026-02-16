from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    # Recupera path e xacro
    control_share = get_package_share_directory('rakuda_control')
    description_share = get_package_share_directory('rakuda_description')
    xacro_file = os.path.join(description_share, 'urdf', 'rakuda.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}
    controllers_file = os.path.join(control_share, 'config', 'rakuda_controllers.yaml')

    # 1. Controller Manager 
    # Il nodo principale, deve partire prima di tutto
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='log',
    )

    # 2. Joint State Broadcaster
    # Parte subito insieme al manager dato che ha un timeout interno per gestisce l'attesa
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='log'
    )

    # 3. Spawner Torso
    torso_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['torso_controller'],
        output='log'
    )

    # 4. Spawner Head
    head_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_controller'],
        output='log'
    )


    # 4a. Spawner Head Position parte inattivo
    head_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_position_controller', "--inactive"],
        output='log'
    )


    # 5. Spawner Left Arm
    left_arm_controller_spawner = Node(
       package='controller_manager',
       executable='spawner',
       arguments=['left_arm_controller'],
       output='log'
    )
    
    # 6. Spawner Right Arm
    right_arm_controller_spawner = Node(
       package='controller_manager',
       executable='spawner',
       arguments=['right_arm_controller'],
       output='log'
    )


    # kick head
    kick_head = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/head_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            "{joint_names: ['neck_yaw_joint','neck_pitch_joint'], "
            "points: [{positions: [0.0, -0.35], time_from_start: {sec: 2}}]}",
        ],
        output="screen",
    )

    kick_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=head_controller_spawner,
            on_exit=[
                TimerAction(period=0.5, actions=[kick_head]),
            ],
        )
    )



    # 7. Catena eventi
    # Quando joint_state_broadcaster finisce (esce), allora avvia i controller
    delay_controllers_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                torso_controller_spawner,
                head_controller_spawner,
                head_position_controller_spawner,
                left_arm_controller_spawner,
                right_arm_controller_spawner
              ]
        )
    )





    # 6. Return
    return LaunchDescription([
        ros2_control_node, # Parte subito
        joint_state_broadcaster_spawner, # Parte subito (ha già un timeout interno che aspetta il manager)
        delay_controllers_after_joint_state, # Contiene gli spawner dei controller che partono dopo il joint_state_broadcaster
        kick_after_spawn, # Dopo aver avviato i controller, esegue il kick alla testa
    ])