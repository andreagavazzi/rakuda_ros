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

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    orbbec_launch_dir = os.path.join(
        get_package_share_directory('orbbec_camera'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('device_preset', default_value='G33X Close Range High Accuracy'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(orbbec_launch_dir, 'gemini_330_series.launch.py')
            ),
            launch_arguments={
                'color_width': '1280',
                'color_height': '720',
                'color_fps': '30',
                'depth_width': '1280',
                'depth_height': '720',
                'depth_fps': '30',
                'device_preset': LaunchConfiguration('device_preset'),
                'align_depth': 'true',
                'spatial_filter': 'true',
                'temporal_filter': 'true',
                'hole_filling_filter': 'true',
            }.items()
        )
    ])


