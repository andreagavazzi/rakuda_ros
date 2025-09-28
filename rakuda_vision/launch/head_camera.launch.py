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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf', default_value='false', description='Whether to publish TFs from camera'
    )

    head_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'camera_namespace': '',
            'camera_name': 'head_camera',
            'device_type': 'd415',

            # Profili corretti all’avvio
            'rgb_camera.color_profile': '640x360x30',
            'depth_module.depth_profile': '640x360x30',
            'infra_module.infra_profile': '640x360x30',
            'depth_module.infra_profile': '640x360x30',

            # Abilitazioni
            'infra1.enable': 'false',
            'infra2.enable': 'false',
            'depth_module.visual_preset': '0',  # default preset, evita modifiche automatiche
            

            # Parametri opzionali
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'publish_tf': LaunchConfiguration('publish_tf'),

            'color_frame_id': 'head_camera_color_frame',
            'color_optical_frame_id': 'head_camera_color_optical_frame'
            
            'respawn': 'true'
            
        }.items()
    )

    return LaunchDescription([
        declare_publish_tf,
        head_camera_node
    ])
