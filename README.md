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

# `rakuda_tools`

## mouth_publisher

`mouth_publisher` is a ROS 2 node that receives audio from the ReSpeaker microphone array and uses it to drive the robot’s mouth animation in real time. Its purpose is to generate speech-synchronized mouth movements, making the robot’s interaction more natural and expressive.

### Subscribed Topics

- Audio input from the ReSpeaker microphone array

### Published Topics

- Mouth animation command topic used by the robot to animate the mouth in sync with speech

### Features

- Real-time audio acquisition from the ReSpeaker
- Speech-synchronized mouth animation
- Improved expressiveness during human-robot interaction

### Usage

```bash
ros2 run rakuda_tools mouth_publisher


## head_motion_filter

`head_motion_filter` is a ROS 2 node that acts as a motion shaper and filter for a 2-DOF head gimbal controlled through a `position_controllers/JointGroupPositionController`. It receives target yaw and pitch commands, applies smoothing and motion constraints, and publishes filtered commands to achieve stable, natural, and safe head movements.

The node can optionally perform a controller switch at startup by deactivating `head_controller` and activating `head_position_controller`, replicating the equivalent CLI behavior. It also includes a startup state machine with optional homing, timeout handling, optional URDF joint-limit clamping, and graceful shutdown behavior.

### Subscribed Topics

- `/head_target` (`std_msgs/Float64MultiArray`)  
  Target head command in the form `[yaw, pitch]` in radians.

### Published Topics

- `/head_position_controller/commands` (`std_msgs/Float64MultiArray`)  
  Filtered head command in the form `[yaw, pitch]` in radians.

### Features

- Startup controller switch from `head_controller` to `head_position_controller`
- Startup state machine: `WAIT_JS -> HOMING -> TRACKING`
- First-order smoothing (`tau`)
- Velocity limiting
- Acceleration limiting
- Optional clamping to URDF joint limits
- Optional target timeout behavior (`hold` or `home`)
- Graceful shutdown without `rclpy` waitset errors

### Usage

```bash
ros2 run rakuda_tools head_motion_filter


