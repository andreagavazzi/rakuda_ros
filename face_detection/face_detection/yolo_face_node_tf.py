#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
)

import cv2
from cv_bridge import CvBridge
import numpy as np
import torch
from ultralytics import YOLO

import tf2_ros
from geometry_msgs.msg import TransformStamped


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class YoloFaceNode(Node):
    def __init__(self):
        super().__init__("yolo_face_node")

        # -------- Parameters --------
        share_dir = get_package_share_directory("face_detection")
        default_model = os.path.join(share_dir, "models", "yolov8n-face.pt")

        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter("output_topic", "/face_detection/image_annotated")
        self.declare_parameter("roi_topic", "/face_detection/rois")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("visualize", True)
        self.declare_parameter("max_det", 10)
        self.declare_parameter("process_every_n", 1)  # 1=ogni frame, 2=1 sì/1 no...

        # --- TF publishing (for look_at_link_target) ---
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("face_frame_id", "face_target")  # child frame name (best face)
        self.declare_parameter("parent_frame", "")  # default: use msg.header.frame_id
        self.declare_parameter("publish_best_only", True)  # True => only one TF (face_frame_id)
        self.declare_parameter("face_frame_prefix", "face_")  # used if publish_best_only=False
        self.declare_parameter("depth_topic", "/camera/depth/image_raw/compressed")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("use_depth", True)
        self.declare_parameter("depth_sync_tolerance_s", 0.05)
        self.declare_parameter("depth_window", 5)  # odd number, e.g. 3/5/7
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 3.0)
        self.declare_parameter("assumed_depth_m", 1.2)  # fallback if depth not available
        self.declare_parameter("no_face_timeout_s", 10.0)  # after this, publish (0,0,0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        roi_topic = self.get_parameter("roi_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.visualize = bool(self.get_parameter("visualize").value)
        self.max_det = int(self.get_parameter("max_det").value)
        self.process_every_n = int(self.get_parameter("process_every_n").value)

        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.face_frame_id = self.get_parameter("face_frame_id").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.publish_best_only = bool(self.get_parameter("publish_best_only").value)
        self.face_frame_prefix = self.get_parameter("face_frame_prefix").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.use_depth = bool(self.get_parameter("use_depth").value)
        self.depth_sync_tolerance_s = float(self.get_parameter("depth_sync_tolerance_s").value)
        self.depth_window = int(self.get_parameter("depth_window").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.assumed_depth_m = float(self.get_parameter("assumed_depth_m").value)
        self.no_face_timeout_s = float(self.get_parameter("no_face_timeout_s").value)

        # -------- Load model once --------
        if not os.path.isfile(self.model_path):
            self.get_logger().error(f"Model file not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)

        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        # Prefer CUDA + FP16 su Jetson se possibile
        self.use_cuda = torch.cuda.is_available()
        if self.use_cuda:
            self.model.to("cuda")
            try:
                if hasattr(self.model, "model"):
                    self.model.model.half()
            except Exception:
                pass
        else:
            self.get_logger().warn("CUDA non disponibile: esecuzione su CPU.")

        # Warmup (best effort)
        try:
            dummy = torch.zeros((1, 3, self.imgsz, self.imgsz))
            if self.use_cuda:
                dummy = dummy.cuda().half()
            _ = self.model.predict(dummy, imgsz=self.imgsz, conf=self.conf, verbose=False)
        except Exception:
            pass

        # -------- ROS I/O --------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.bridge = CvBridge()
        self.sub_rgb = self.create_subscription(Image, input_topic, self.image_cb, qos)

        # Publisher immagine annotata
        self.pub_img = self.create_publisher(Image, output_topic, 10)
        # Publisher detections/ROI
        self.pub_rois = self.create_publisher(Detection2DArray, roi_topic, 10)

        # --- Depth + CameraInfo cache (for 3D TF) ---
        self._cam_info = None
        self._last_depth = None
        self._last_depth_stamp_s = None
        self._last_depth_frame_id = None
        self._depth_encoding = None

        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

            # We need intrinsics; subscribe always when TF publish is enabled
            self.sub_info = self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)

            if self.use_depth:
                self.sub_depth = self.create_subscription(Image, self.depth_topic, self._depth_cb, qos)

        self.get_logger().info(
            "YOLO Face node avviato.\n"
            f"- input_topic:   {input_topic}\n"
            f"- output_topic:  {output_topic}\n"
            f"- roi_topic:     {roi_topic}\n"
            f"- model:         {self.model_path}\n"
            f"- conf:          {self.conf}\n"
            f"- imgsz:         {self.imgsz}\n"
            f"- visualize:     {self.visualize}\n"
            f"- max_det:       {self.max_det}\n"
            f"- process_every: {self.process_every_n}\n"
            f"- cuda:          {self.use_cuda}\n"
            f"- publish_tf:    {self.publish_tf} (use_depth={self.use_depth})"
        )

        self._frame_count = 0

        # Keep-alive for TF: remember last valid face position so 'face_target' never disappears
        self._last_face_xyz = None  # (x,y,z) in parent_frame
        self._last_face_parent = None
        self._start_time = self.get_clock().now()
        self._last_face_seen_time = None  # rclpy.time.Time (node clock)

    # ----------------- Depth/Info callbacks -----------------
    def _camera_info_cb(self, msg: CameraInfo):
        self._cam_info = msg

    def _depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge depth error: {e}")
            return

        self._last_depth = depth
        self._last_depth_stamp_s = _stamp_to_sec(msg.header.stamp)
        self._last_depth_frame_id = msg.header.frame_id
        self._depth_encoding = msg.encoding

    def _get_depth_m(self, u: int, v: int, stamp_s: float):
        # If no depth available (or too old), return None
        if self._last_depth is None or self._last_depth_stamp_s is None:
            return None

        if abs(stamp_s - self._last_depth_stamp_s) > self.depth_sync_tolerance_s:
            return None

        depth = self._last_depth
        h, w = depth.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return None

        # Windowed median to reduce holes/noise
        win = max(1, int(self.depth_window))
        if win % 2 == 0:
            win += 1
        r = win // 2

        u0 = max(0, u - r)
        u1 = min(w, u + r + 1)
        v0 = max(0, v - r)
        v1 = min(h, v + r + 1)

        patch = depth[v0:v1, u0:u1].astype(np.float32).reshape(-1)

        # Convert to meters depending on dtype
        # - uint16: millimeters (common for depth cameras)
        # - float32/float64: meters
        if depth.dtype == np.uint16:
            patch_m = patch * 0.001
        else:
            patch_m = patch

        # Filter invalids
        patch_m = patch_m[np.isfinite(patch_m)]
        patch_m = patch_m[(patch_m > self.min_depth_m) & (patch_m < self.max_depth_m)]
        if patch_m.size == 0:
            return None

        return float(np.median(patch_m))

    def _pixel_to_3d(self, u: int, v: int, stamp_s: float):
        """
        Returns (x, y, z, used_depth) in CAMERA frame (typically the optical frame).
        """
        if self._cam_info is None:
            return None

        K = self._cam_info.k  # row-major 3x3
        fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
        if fx <= 1e-6 or fy <= 1e-6:
            return None

        depth_m = None
        used_depth = False
        if self.use_depth:
            depth_m = self._get_depth_m(u, v, stamp_s)
            used_depth = depth_m is not None

        if depth_m is None:
            depth_m = float(self.assumed_depth_m)
            used_depth = False

        z = depth_m
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        return x, y, z, used_depth

    def _broadcast_face_tf(self, parent_frame: str, child_frame: str, stamp, x: float, y: float, z: float):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        # Identity rotation (we only care about position for look-at)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _resolve_parent_frame(self, msg: Image) -> str:
        if self.parent_frame.strip():
            return self.parent_frame.strip()
        if self._cam_info is not None and getattr(self._cam_info.header, 'frame_id', ''):
            return self._cam_info.header.frame_id
        return msg.header.frame_id

    def _no_face_elapsed_s(self) -> float:
        now = self.get_clock().now()
        ref = self._last_face_seen_time if self._last_face_seen_time is not None else self._start_time
        return (now - ref).nanoseconds / 1e9

    # ----------------- Main RGB callback -----------------
    def image_cb(self, msg: Image):
        # Throttle: elabora 1 frame ogni N (ma continuiamo a ripubblicare TF per keep-alive)
        self._frame_count += 1
        if self.process_every_n > 1 and (self._frame_count % self.process_every_n) != 0:
            # Keep TF alive even when skipping inference
            if self.publish_tf:
                if self._no_face_elapsed_s() >= self.no_face_timeout_s:
                    parent = self._resolve_parent_frame(msg)
                    self._broadcast_face_tf(parent, self.face_frame_id, msg.header.stamp, 0.0, 0.0, 0.0)
                elif self._last_face_xyz is not None and self._last_face_parent is not None:
                    x, y, z = self._last_face_xyz
                    self._broadcast_face_tf(self._last_face_parent, self.face_frame_id, msg.header.stamp, x, y, z)
            return

        # Converti a OpenCV (BGR)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        # Inference
        try:
            results = self.model.predict(
                source=frame,
                imgsz=self.imgsz,
                conf=self.conf,
                max_det=self.max_det,
                verbose=False,
                device=0 if self.use_cuda else "cpu",
            )
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return

        annotated = frame if not self.visualize else frame.copy()

        # Detection2DArray
        det_array = Detection2DArray()
        det_array.header = Header()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = msg.header.frame_id

        # Collect candidates for TF
        candidates = []  # (score, area, cx, cy)

        if results and len(results) > 0:
            res = results[0]
            if res.boxes is not None and res.boxes.xyxy is not None:
                boxes = res.boxes.xyxy
                confs = res.boxes.conf
                for i in range(len(boxes)):
                    x1, y1, x2, y2 = [float(v) for v in boxes[i].tolist()]
                    w = max(0.0, x2 - x1)
                    h = max(0.0, y2 - y1)
                    cx = x1 + w / 2.0
                    cy = y1 + h / 2.0
                    area = w * h
                    score = float(confs[i].item()) if confs is not None else 0.0

                    candidates.append((score, area, cx, cy))

                    # Draw
                    if self.visualize:
                        cv2.rectangle(
                            annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1
                        )
                        cv2.putText(
                            annotated,
                            f"face {score:.2f}",
                            (int(x1), max(0, int(y1) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1,
                            cv2.LINE_AA,
                        )

                    # Detection2D
                    det = Detection2D()
                    det.header = det_array.header

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = "face"
                    hyp.hypothesis.score = score
                    det.results.append(hyp)

                    # bbox center in pixel space
                    center = det.bbox.center
                    if hasattr(center, "x"):  # Pose2D
                        center.x = cx
                        center.y = cy
                        if hasattr(center, "theta"):
                            center.theta = 0.0
                    elif hasattr(center, "position"):  # Pose
                        center.position.x = cx
                        center.position.y = cy
                        if hasattr(center.position, "z"):
                            center.position.z = 0.0
                        if hasattr(center, "orientation"):
                            if hasattr(center.orientation, "x"):
                                center.orientation.x = 0.0
                            if hasattr(center.orientation, "y"):
                                center.orientation.y = 0.0
                            if hasattr(center.orientation, "z"):
                                center.orientation.z = 0.0
                            if hasattr(center.orientation, "w"):
                                center.orientation.w = 1.0

                    det.bbox.size_x = w
                    det.bbox.size_y = h

                    det_array.detections.append(det)

        # Publish annotated image
        try:
            if self.visualize:
                img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                img_msg.header = Header()
                img_msg.header.stamp = msg.header.stamp
                img_msg.header.frame_id = msg.header.frame_id
                self.pub_img.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f"Publish image error: {e}")

        # Publish ROI array (even empty)
        try:
            self.pub_rois.publish(det_array)
        except Exception as e:
            self.get_logger().warn(f"Publish ROI error: {e}")

        # Publish TF(s)
        if self.publish_tf and candidates:
            if self.parent_frame.strip():
                parent = self.parent_frame.strip()
            elif self._cam_info is not None and getattr(self._cam_info.header, 'frame_id', ''):
                parent = self._cam_info.header.frame_id
            else:
                parent = msg.header.frame_id
            stamp_s = _stamp_to_sec(msg.header.stamp)

            # Sort by score, then area (bigger face wins on tie)
            candidates.sort(key=lambda t: (t[0], t[1]), reverse=True)

            if self.publish_best_only:
                score, area, cx, cy = candidates[0]
                u, v = int(round(cx)), int(round(cy))

                p = self._pixel_to_3d(u, v, stamp_s)
                if p is not None:
                    x, y, z, used_depth = p
                    self._broadcast_face_tf(parent, self.face_frame_id, msg.header.stamp, x, y, z)
                    self._last_face_xyz = (x, y, z)
                    self._last_face_parent = parent
                    self._last_face_seen_time = self.get_clock().now()
            else:
                # publish all faces: face_0, face_1, ...
                for idx, (score, area, cx, cy) in enumerate(candidates):
                    u, v = int(round(cx)), int(round(cy))
                    p = self._pixel_to_3d(u, v, stamp_s)
                    if p is None:
                        continue
                    x, y, z, used_depth = p
                    child = f"{self.face_frame_prefix}{idx}"
                    self._broadcast_face_tf(parent, child, msg.header.stamp, x, y, z)
                    if idx == 0:
                        self._last_face_xyz = (x, y, z)
                        self._last_face_parent = parent
                        self._last_face_seen_time = self.get_clock().now()
        # Keep-alive when no face detected
        if self.publish_tf and (not candidates):
            if self._no_face_elapsed_s() >= self.no_face_timeout_s:
                parent = self._resolve_parent_frame(msg)
                self._broadcast_face_tf(parent, self.face_frame_id, msg.header.stamp, 0.0, 0.0, 0.0)
            elif self._last_face_xyz is not None and self._last_face_parent is not None:
                x, y, z = self._last_face_xyz
                self._broadcast_face_tf(self._last_face_parent, self.face_frame_id, msg.header.stamp, x, y, z)



def main():
    rclpy.init()
    node = None
    try:
        node = YoloFaceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()