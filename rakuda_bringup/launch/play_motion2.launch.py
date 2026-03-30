import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = FindPackageShare("rakuda_bringup")

    xacro_file = os.path.join(
        get_package_share_directory("rakuda_description"),
        "urdf", "rakuda.urdf.xacro",
    )
    robot_description = xacro.process_file(xacro_file).toxml()

    srdf_file = os.path.join(
        get_package_share_directory("rakuda_moveit_config"),
        "config", "rakuda.srdf",
    )
    with open(srdf_file, "r") as f:
        robot_description_semantic = f.read()

    motions_file_arg = DeclareLaunchArgument(
        "motions_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "motions", "rakuda_motions.yaml"]
        ),
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    play_motion2_node = Node(
        package="play_motion2",
        executable="play_motion2_node",
        name="play_motion2",
        output="screen",
        parameters=[
            LaunchConfiguration("motions_file"),
            {
                "robot_description": robot_description,
                "robot_description_semantic": robot_description_semantic,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    return LaunchDescription([
        motions_file_arg,
        use_sim_time_arg,
        play_motion2_node,
    ])
