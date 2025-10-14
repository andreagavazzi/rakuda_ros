![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)
![Jetson Orin Nano](https://img.shields.io/badge/Jetson-Orin%20Nano-green)

## Face Detection Package for ROS 2 Humble (Jetson Orin Nano Optimized)

This ROS 2 package (`face_detection`) provides a real-time face detection node implemented in Python using OpenCV's Deep Neural Network (DNN) module.  
It is optimized for NVIDIA Jetson Orin Nano and designed to run efficiently with GPU acceleration (CUDA and cuDNN).

The node subscribes to a camera topic (e.g., `/camera/image_raw`), performs face detection using the Res10 SSD model, and publishes the processed image with annotated bounding boxes to `/face_detector/image_faces`.

### Features
- Real-time face detection at up to 30 FPS on Jetson Orin Nano  
- GPU-accelerated inference via OpenCV DNN (CUDA backend)  
- ROS 2 image I/O integration with `cv_bridge` and `image_transport`  
- Easily extendable for face recognition and distance estimation  

### Topics
- **Subscribed:** `/camera/image_raw` (`sensor_msgs/Image`)  
- **Published:** `/face_detector/image_faces` (`sensor_msgs/Image`)  

### Requirements
- ROS 2 Humble
- Python 3.8+
- OpenCV 4.x compiled with CUDA (`WITH_CUDA=ON`, `OPENCV_DNN_CUDA=ON`)
- `cv_bridge`, `image_transport`, `rclpy`, `numpy`

This package serves as the foundation for building more advanced perception modules, such as face recognition and person tracking in robotics applications.


### Run the Face Detection Node
```bash
ros2 run face_detection face_detector



