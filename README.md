# rakuda_ros


> [!WARNING]  
> The repository is currently in heavy development.



## Repository structure

This repository contains the following packages:

### `rakuda_control`
Control stack based on **ros2_control**.

Typical responsibilities:
- Controller manager configuration
- Controller YAMLs (head/torso controllers, joint state broadcaster, etc.)
- Launch files that start `ros2_control_node` and spawn controllers

```bash
ros2 launch rakuda_control rakuda_control.launch.py
```


### `rakuda_head_action`
A ROS 2 node exposing a **PointHead interface** for the robot head (useful for “look at target” behaviors).

Typical responsibilities:
- Provide an Action server to command head orientation towards a 3D target
- Convert target point + frames into commands compatible with the head controller
- Integrate with TF to interpret target frames

**When to use it:** when you want a standard-ish “point the head at X” behavior (e.g., look at a hand, look at an object).

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



