# rakuda_vision

The **`rakuda_vision`** package provides vision capabilities for the Rakuda robot platform using the **Orbbec Gemini 335** depth camera.  
It integrates the [OrbbecSDK_ROS2]([https://github.com/IntelRealSense/realsense-ros](https://github.com/orbbec/OrbbecSDK_ROS2)) driver into the Rakuda ecosystem, making RGB-D data available for perception, manipulation, and AI tasks.

![](https://github.com/andreagavazzi/rakuda_ros/blob/main/rakuda_vision/depthcamera.png)

---

## Features

- Publishes **color and depth images** from the Gemini 335.  
- Provides **point clouds** for 3D perception and environment mapping.  
- Includes **launch files** to easily configure and start the camera.  
- Supports multiple streams: color, depth, infrared, and aligned depth.  
- Designed as the **vision entry point** for Rakuda, enabling modules such as navigation, object detection, and manipulation.

---


## Usage

To launch the Gemini 335 with default parameters:
```bash
ros2 launch rakuda_vision head_camera.launch.py
```

Optional parameters:
- `publish_tf:=true` — publish tf data.
- `enable_pointcloud:=true` — publish point cloud data.  
- `color_width:=640 color_height:=480` — set resolution for the color stream.  
- `depth_width:=640 depth_height:=480` — set resolution for the depth stream.  
- `fps:=30` — set frame rate.  

Example:
```bash
ros2 launch rakuda_vision head_camera.launch.py publish_tf:=true fps:=30
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
