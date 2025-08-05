# rakuda_control

This package is based on [ros2_control](https://github.com/ros-controls/ros2_control) controller package

## Related Files

- [source/rakuda_hardware.cpp](./source/rakuda_hardware.cpp)
  - It realized the comunication to the [Hardware Components](https://control.ros.org/master/doc/getting_started/getting_started.html#hardware-components) loading them from rakuda_description.
- [launch/rakuda_control.launch.py](./launch/rakuda_control.launch.py)
  - This is the launch file that start the [Controller Manager](https://control.ros.org/master/doc/getting_started/getting_started.html#controller-manager) and the controllers.
- [config/rakuda_controllers.yaml](./config/rakuda_controllers.yaml)
  - This is the parameter file for the Controller Manager.


## Configuring the USB Communication Port
The following method fixes the USB serial conversion device name used to communicate with the actual machine.

```
ros2 run rakuda_tools create_udev_rules
```
After running, rebooting and connecting rakuda will create /dev/rakudaspine


## Starting a Node

`rakuda_control.launch.py`
 The node starts and The following controllers are loaded:

- right_arm_controller (`joint_trajectory_controller/JointTrajectoryController`)
- right_gripper_controller (`position_controllers/GripperActionController`)
- left_arm_controller (`joint_trajectory_controller/JointTrajectoryController`)
- left_gripper_controller (`position_controllers/GripperActionController`)
- neck_controller (`joint_trajectory_controller/JointTrajectoryController`)
- waist_yaw_controller (`joint_trajectory_controller/JointTrajectoryController`)
- joint_state_broadcaster (`joint_state_broadcaster/JointStateBroadcaster`)

After the node starts, You can view the joint angle information with the following command

```
ros2 topic echo /joint_states
```



## Controller Manager Parameters

The Controller Manager parameters are stored in
[config/sciurus17_controllers.yaml](./config/sciurus17_controllers.yaml)


```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    right_gripper_controller:
      type: position_controllers/GripperActionController
```

### control cycle

`update_rate` sets the control period


### controller

You can set up a controller to control each joint on Rakuda. The names and roles of controllers correspond to each other as follows:

- right_arm_controller
  - Right-arm control controller
- right_gripper_controller
  - Controller for right gripper control
- left_arm_controller
  - Left-arm control controller
- left_gripper_controller
  - Controller for left gripper control
- neck_controller
  - Neck control controller
- waist_yaw_controller
  - Waist control controller

## rakuda_hardware Parameters
`rakuda_hardware`is set in `rakuda_description/urdf/rakuda.urdf.xacro`


```xml
  <xacro:arg name="port_name" default="/dev/sciurus17spine" />
  <xacro:arg name="baudrate" default="3000000" />
  <xacro:arg name="timeout_seconds" default="1.0" />
  <xacro:arg name="manipulator_config_file_path" default="" />
```

### USB communication port

`port_name` Sets the USB communication port to be used to communicate with the Rakuda.

### Baud rate

`baudrate` sets the communication baud rate with the Dynamixel installed in Rakuda.

The default value is set to (3 Mbps).

### Communication Timeout

`timeout_seconds` sets the communication timeout time in seconds.

If communication continues to fail for a certain period of time (default 1 second), Stop read/write from working. This is effective when the USB cable or power cable is disconnected.

### Configuration file path for RT manipulator C++ libraries

`rakuda_hardware` uses the 
[RT manipulator C++ library](https://github.com/rt-net/rt_manipulators_cpp) to communicate with Rakuda.

`manipulatcor_config_file_path` is Sets the path to the servo configuration file that the library reads.

---

[back to top](#rakuda_control)
