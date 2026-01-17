# rakuda_ros





### `rakuda_control`

ROS 2 control package for **Rakuda**.  
It provides the runtime configuration and launch files to bring up `ros2_control`:
- Controller Manager (`ros2_control_node`) setup
- Controllers YAML (joints, interfaces, constraints)
- Launch files to load/activate controllers and broadcasters

## Usage
```bash
ros2 launch rakuda_control rakuda_control_launch.py
ros2 control list_controllers
ros2 control set_controller_state head_controller active
```

> [!IMPORTANT]  
> Works together with the rakuda_description package (URDF/Xacro) and the Dynamixel hardware interface.


---

### `rakuda_tools`
Small utilities and helper nodes/scripts used during development and debugging.

Typical responsibilities:
- Enable/disable torque for all (or selected) Dynamixel motors
- Set LEDs, read/write Dynamixel registers
- Quick diagnostics and smoke-test scripts

**When to use it:** when you need quick CLI tools to interact with hardware and verify the robot state.

---

## Build

From your ROS 2 workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Build a single package:

```bash
colcon build --packages-select rakuda_tools --symlink-install
source install/setup.bash
```

---

## Run (examples)

> Replace launch filenames with the ones actually present in your packages.

List available launch files:
```bash
ros2 launch --show-args rakuda_bringup <launch_file>.launch.py
```

Start control stack:
```bash
ros2 launch rakuda_control rakuda_control.launch.py
```

Run a tool:
```bash
ros2 run rakuda_tools <tool_name>
```



