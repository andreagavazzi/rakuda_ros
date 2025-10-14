from setuptools import setup

package_name = 'face_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrea Gavazzi',
    maintainer_email='andrea.gavazzi@me.com',
    description='Face detection node for ROS 2 Humble (OpenCV DNN)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'face_detector = face_detection.face_detector_node:main',
        ],
    },
)
