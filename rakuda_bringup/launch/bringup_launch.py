from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_share = get_package_share_directory('rakuda_description')
    control_share = get_package_share_directory('rakuda_control')

    display_launch = os.path.join(description_share, 'launch', 'display.launch.py')
    control_launch = os.path.join(control_share, 'launch', 'control.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(display_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch)),
    ])

