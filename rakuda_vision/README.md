# rakuda_vision

The **`rakuda_vision`** package provides vision capabilities for the Rakuda robot platform using the **Intel RealSense D415** depth camera.  
It integrates the [realsense2_camera](https://github.com/IntelRealSense/realsense-ros) driver into the ROS 2 ecosystem, making RGB-D data available for perception, manipulation, and AI tasks.

---

## Features

- Publishes **color and depth images** from the RealSense D415.  
- Provides **point clouds** for 3D perception and environment mapping.  
- Includes **launch files** to easily configure and start the camera.  
- Supports multiple streams: color, depth, infrared, and aligned depth.  
- Designed as the **vision entry point** for Rakuda, enabling modules such as navigation, object detection, and manipulation.

---

## Installation

1. Clone this package into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/<your-org>/rakuda_vision.git
