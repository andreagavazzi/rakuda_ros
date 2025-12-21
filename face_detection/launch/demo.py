# Lancia il nodo face detection e la orbbec camera con parametri di default

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Percorsi share ---
    pkg_share = get_package_share_directory('face_detection')
    orbbec_share = get_package_share_directory('orbbec_camera')

    default_model = os.path.join(pkg_share, 'models', 'yolov8n-face.pt')
    orbbec_launch_path = os.path.join(orbbec_share, 'launch', 'gemini_330_series.launch.py')

    # --- Argomenti lanciabili ---
    input_topic      = LaunchConfiguration('input_topic')
    model_path       = LaunchConfiguration('model_path')
    imgsz            = LaunchConfiguration('imgsz')
    conf             = LaunchConfiguration('conf')
    visualize        = LaunchConfiguration('visualize')
    max_det          = LaunchConfiguration('max_det')
    process_every_n  = LaunchConfiguration('process_every_n')

    decl_args = [
        DeclareLaunchArgument('input_topic',      default_value='/camera/color/image_raw',
                              description='Topic immagine colore della Orbbec'),
        DeclareLaunchArgument('model_path',       default_value=default_model,
                              description='Percorso al file .pt del modello face (nel share del package)'),
        DeclareLaunchArgument('imgsz',            default_value='640',
                              description='Dimensione lato immagine per YOLO'),
        DeclareLaunchArgument('conf',             default_value='0.35',
                              description='Soglia confidenza detections'),
        DeclareLaunchArgument('visualize',        default_value='true',
                              description='Disegna bbox sull’immagine annotata'),
        DeclareLaunchArgument('max_det',          default_value='10',
                              description='Numero massimo di facce per frame'),
        DeclareLaunchArgument('process_every_n',  default_value='1',
                              description='Elabora 1 frame ogni N'),
    ]

    # --- Include launch Orbbec (camera) ---
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch_path)
        # Se vuoi passare argomenti alla camera, aggiungi "launch_arguments={...}.items()"
    )

    # --- Nodo YOLO face ---
    face_node = Node(
        package='face_detection',
        executable='yolo_face_node',
        name='yolo_face_node',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'model_path': model_path,
            'imgsz': imgsz,
            'conf': conf,
            'visualize': visualize,
            'max_det': max_det,
            'process_every_n': process_every_n,
        }]
    )

    return LaunchDescription(decl_args + [orbbec_launch, face_node])
