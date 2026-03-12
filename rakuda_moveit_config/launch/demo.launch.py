from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('rakuda_description'),
        'urdf', 'rakuda.urdf.xacro'
    )
    moveit_config = (
        MoveItConfigsBuilder("rakuda", package_name="rakuda_moveit_config")
        .robot_description(
            file_path=xacro_file,
            mappings={"use_mock_hardware": "true"}
        )
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
