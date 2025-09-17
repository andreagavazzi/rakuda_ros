# rakuda_vision

The **`rakuda_vision`** package provides vision capabilities for the Rakuda robot platform using the **Intel RealSense D415** depth camera.  
It integrates the [realsense2_camera](https://github.com/IntelRealSense/realsense-ros) driver into the ROS 2 ecosystem, making RGB-D data available for perception, manipulation, and AI tasks.

![](https://github.com/andreagavazzi/rakuda_ros/blob/main/rakuda_vision/depthcamera.png)

---

## Features

- Publishes **color and depth images** from the RealSense D415.  
- Provides **point clouds** for 3D perception and environment mapping.  
- Includes **launch files** to easily configure and start the camera.  
- Supports multiple streams: color, depth, infrared, and aligned depth.  
- Designed as the **vision entry point** for Rakuda, enabling modules such as navigation, object detection, and manipulation.

---

## Installation


1. Install dependencies:
   ```bash
   sudo apt update
   sudo apt install ros-${ROS_DISTRO}-realsense2-camera \
                    ros-${ROS_DISTRO}-image-common \
                    ros-${ROS_DISTRO}-image-transport
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select rakuda_vision
   source install/setup.bash
   ```

---

## Usage

To launch the RealSense D415 with default parameters:
```bash
ros2 launch rakuda_vision head_camera.launch.py
```

Optional parameters:
- `enable_pointcloud:=true` — publish point cloud data.  
- `color_width:=640 color_height:=480` — set resolution for the color stream.  
- `depth_width:=640 depth_height:=480` — set resolution for the depth stream.  
- `fps:=30` — set frame rate.  

Example:
```bash
ros2 launch rakuda_vision head_camera.launch.py enable_pointcloud:=true fps:=30
```

---

## Published Topics

The package publishes the following topics (when enabled):

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Raw color image |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Depth image (rectified) |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Depth image aligned to color |
| `/camera/infra1/image_raw` | `sensor_msgs/Image` | Infrared image (left) |
| `/camera/infra2/image_raw` | `sensor_msgs/Image` | Infrared image (right) |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | Point cloud in the color frame |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Intrinsic/extrinsic parameters |

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_pointcloud` | `false` | Enable point cloud publishing |
| `color_width` | `640` | Width of the color image |
| `color_height` | `480` | Height of the color image |
| `depth_width` | `640` | Width of the depth image |
| `depth_height` | `480` | Height of the depth image |
| `fps` | `30` | Frames per second |


## License

This package is released under the [Apache 2.0 License](LICENSE).

---

## Maintainer

- Andrea Gavazzi (agRobotics)  
