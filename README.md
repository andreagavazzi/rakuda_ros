# rakuda_ros

This repository groups multiple ROS 2 packages that cover the full stack: control (ros2_control), tools/utilities, and higher-level behaviors.
Each package can be built and launched independently, but they are designed to work together.


---


### `rakuda_control`

ROS 2 control package for **Rakuda**.  
It provides the runtime configuration and launch files to bring up `ros2_control`:
- Controller Manager (`ros2_control_node`) setup
- Controllers YAML (joints, interfaces, constraints)
- Launch files to load/activate controllers and broadcasters

### Usage
```bash
ros2 launch rakuda_control rakuda_control_launch.py
ros2 control list_controllers
ros2 control set_controller_state head_controller active
```

> [!IMPORTANT]  
> Works together with the rakuda_description package (URDF/Xacro) and the Dynamixel hardware interface.


---

TODO....