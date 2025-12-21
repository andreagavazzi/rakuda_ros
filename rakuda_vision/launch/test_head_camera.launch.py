# Copyright 2023 Andrea Gavazzi
#
# Simplified version of my head_camera.launch.py 


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    head_camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py']),
            launch_arguments={
                'camera_namespace': '',
                'camera_name': 'head_camera',
                'device_type': 'd415',
                'rgb_camera.color_profile': '640x360x30',
                'depth_module.depth_profile': '640x360x30',
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
                
                # 'base_frame_id': 'head_camera_link',
                # 'color_frame_id': 'head_camera_color_frame',
                # 'color_optical_frame_id': 'head_camera_color_optical_frame',
                # 'publish_tf': 'false'   # tf are published by robot_state_publisher
                
            }.items()
        )

    return LaunchDescription([
        head_camera_node
    ])
