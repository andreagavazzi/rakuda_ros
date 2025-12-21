# Copyright 2025 Andrea Gavazzi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf', default_value='false', description='Whether to publish TFs from camera'
    )

    head_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='head_camera',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'camera_namespace': '',
            'camera_name': 'head_camera',
            'device_type': 'd435',

            # Profili corretti
            'rgb_camera.color_profile': '640x360x30',
            'depth_module.depth_profile': '640x360x30',
            'infra_module.infra_profile': '640x360x30',
            'depth_module.infra_profile': '640x360x30',

            # Abilitazioni
            'infra1.enable': False,
            'infra2.enable': False,
            'depth_module.visual_preset': 0,
            
            # Disabilita IMU
            'gyro_fps': '0',
            'accel_fps': '0',
            'unite_imu_method': 'none',
            

            # Extra
            'pointcloud.enable': True,
            'align_depth.enable': True,
            'publish_tf': LaunchConfiguration('publish_tf'),

            'color_frame_id': 'head_camera_color_frame',
            'color_optical_frame_id': 'head_camera_color_optical_frame'
        }]
    )

    return LaunchDescription([
        declare_publish_tf,
        head_camera_node
    ])

