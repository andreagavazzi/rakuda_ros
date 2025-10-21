![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)
![Jetson Orin Nano](https://img.shields.io/badge/Jetson-Orin%20Nano-green)
![YOLOv8](https://img.shields.io/badge/YOLOv8-blue)

# face_detection

ROS 2 (Humble) node for **real-time face detection from RGB camera**. It subscribes to a color image, runs a YOLO face model, and publishes:
1) an **annotated image** with rectangles and labels 
2) a **`vision_msgs/Detection2DArray`** with ROIs (`face`, score, bbox).

---

## Key features
- **YOLO-based face detector** (Ultralytics). Model loaded once at startup from `model_path`.  
- **GPU-aware**: uses CUDA automatically when available and attempts FP16 on supported models; otherwise falls back to CPU.  
- **Annotated RGB output** (optional) with boxes and confidence text.  
- **Lightweight throttling**: process every Nth frame (`process_every_n`).  
- **ROS 2 friendly I/O** with explicit QoS (BEST_EFFORT, KEEP_LAST depth 10).

---

## Topics

**Subscribe**
- `/camera/color/image_raw` — `sensor_msgs/Image` (parameter `input_topic`)

**Publish**
- `/face_detection/image_annotated` — `sensor_msgs/Image` (parameter `output_topic`, only if `visualize=true`)  
- `/face_detection/rois` — `vision_msgs/Detection2DArray` with:  
  - `results[].hypothesis.class_id = "face"`  
  - `results[].hypothesis.score ∈ [0,1]`  
  - `bbox.center (cx, cy)`, `bbox.size (w, h)`  
  (parameter `roi_topic`)

---

## Parameters

| Name | Type (default) | Description |
|---|---|---|
| `input_topic` | string (`/camera/color/image_raw`) | RGB input image topic |
| `output_topic` | string (`/face_detection/image_annotated`) | Annotated image output |
| `roi_topic` | string (`/face_detection/rois`) | Detections array output |
| `model_path` | string (`<pkg>/models/yolov8n-face.pt`) | Face model path (must exist) |
| `conf` | double (0.35) | Confidence threshold for detections |
| `imgsz` | int (640) | Inference image size |
| `visualize` | bool (true) | Draw and publish annotated image |
| `max_det` | int (10) | Max detections per frame |
| `process_every_n` | int (1) | Process 1 frame every *N* frames (throttling) |

---

## Dependencies

- ROS 2 Humble: `rclpy`, `vision_msgs`, `sensor_msgs`, `ament_index_python`
- Python: `ultralytics`, `torch`, `opencv-python`, `cv_bridge`
- Optional: CUDA (Jetson/desktop) for GPU acceleration

:exclamation:Jetson note: install the JetPack-matching PyTorch wheel from NVIDIA


## Build

```bash
colcon build --packages-select face_detection
source install/setup.bash
```

### Run
```bash
ros2 launch face_detection face.launch.py
```

## License

(C) 2025 Andrea Gavazzi
Unless attributed otherwise, everything in this repository is under the Apache License, Version 2.0.

