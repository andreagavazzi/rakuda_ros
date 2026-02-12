from setuptools import setup

package_name = 'rakuda_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/initial_position.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea@todo.todo',
    description='Small tools for Rakuda',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_motion_filter = rakuda_tools.head_motion_filter:main',
            'torque_except = rakuda_tools.torque_except:main',
            'look_at_link_target = rakuda_tools.look_at_link_target:main',
            'head_gimbal_keyboard = rakuda_tools.head_gimbal_keyboard:main',
            "pose_player = rakuda_tools.pose_player:main",
        ],
    },
)

