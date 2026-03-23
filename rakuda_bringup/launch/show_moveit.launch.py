#!/usr/bin/env python3

# show_moveit.launch.py
# Avvio sequenziale con IncludeLaunchDescription per shutdown pulito via Ctrl+C:
#   t=0.0s  rakuda_description
#   t=1.5s  rakuda_control
#   t=3.5s  move_group
#   t=7.0s  moveit_rviz

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("rakuda_description"),
            "launch", "description.launch.py",
        ))
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("rakuda_control"),
            "launch", "control.launch.py",
        ))
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("rakuda_moveit_config"),
            "launch", "move_group.launch.py",
        ))
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("rakuda_moveit_config"),
            "launch", "moveit_rviz.launch.py",
        ))
    )

    return LaunchDescription([
        LogInfo(msg="[show_moveit] avvio: description -> control -> move_group -> moveit_rviz"),

        desc_launch,

        TimerAction(period=1.5, actions=[
            LogInfo(msg="[show_moveit] avvio control..."),
            control_launch,
        ]),

        TimerAction(period=3.5, actions=[
            LogInfo(msg="[show_moveit] avvio move_group..."),
            move_group_launch,
        ]),

        TimerAction(period=7.0, actions=[
            LogInfo(msg="[show_moveit] avvio moveit_rviz..."),
            rviz_launch,
        ]),
    ])
