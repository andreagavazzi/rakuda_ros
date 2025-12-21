from setuptools import setup
from glob import glob
import os

package_name = 'face_detection'

def existing(pattern):
    """Ritorna lista file esistenti per evitare liste vuote in data_files."""
    files = glob(pattern)
    return files if files else []

data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    ('share/' + package_name, ['package.xml']),
]

# Installa modelli/launch/config solo se esistono
model_files  = existing('models/*')
launch_files = existing('launch/*.py')
config_files = existing('config/*.yaml')

if model_files:
    data_files.append(('share/' + package_name + '/models', model_files))
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))
if config_files:
    data_files.append(('share/' + package_name + '/config', config_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea@example.com',
    description='YOLOv8 face detector subscribing to camera image topics',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_face_node = face_detection.yolo_face_node:main',
        ],
    },
)

