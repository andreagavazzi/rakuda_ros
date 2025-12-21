#!/usr/bin/env python3
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from std_msgs.msg import Header

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)

import cv2
from cv_bridge import CvBridge
import torch
from ultralytics import YOLO


class YoloFaceNode(Node):
    def __init__(self):
        super().__init__('yolo_face_node')

        # -------- Parameters --------
        share_dir = get_package_share_directory('face_detection')
        default_model = os.path.join(share_dir, 'models', 'yolov8n-face.pt')

        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/face_detection/image_annotated')
        self.declare_parameter('roi_topic', '/face_detection/rois')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('visualize', True)
        self.declare_parameter('max_det', 10)
        self.declare_parameter('process_every_n', 1)  # 1=ogni frame, 2=1 sì/1 no...

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        roi_topic = self.get_parameter('roi_topic').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf = float(self.get_parameter('conf').get_parameter_value().double_value)
        self.imgsz = int(self.get_parameter('imgsz').get_parameter_value().integer_value)
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.max_det = int(self.get_parameter('max_det').get_parameter_value().integer_value)
        self.process_every_n = int(self.get_parameter('process_every_n').get_parameter_value().integer_value)

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
            self.model.to('cuda')
            try:
                # alcuni modelli supportano half dalla property .model
                if hasattr(self.model, 'model'):
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
            depth=10
        )

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, input_topic, self.image_cb, qos)

        # Publisher immagine annotata
        self.pub_img = self.create_publisher(Image, output_topic, 10)
        # Publisher detections/ROI
        self.pub_rois = self.create_publisher(Detection2DArray, roi_topic, 10)

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
            f"- cuda:          {self.use_cuda}"
        )

        self._frame_count = 0
        self.last_log = time.time()

    def image_cb(self, msg: Image):
        # Throttle: elabora 1 frame ogni N
        self._frame_count += 1
        if self.process_every_n > 1 and (self._frame_count % self.process_every_n) != 0:
            return

        # Converti a OpenCV (BGR)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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
                device=0 if self.use_cuda else 'cpu'
            )
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return

        # Parsing risultati
        faces = 0
        annotated = frame if not self.visualize else frame.copy()

        # Prepara messaggio ROI (Detection2DArray)
        det_array = Detection2DArray()
        det_array.header = Header()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = msg.header.frame_id

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
                    score = float(confs[i].item()) if confs is not None else 0.0
                    faces += 1

                    # Disegno opzionale
                    if self.visualize:
                        cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1)
                        cv2.putText(annotated, f"face {score:.2f}",
                                    (int(x1), max(0, int(y1) - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                    # Detection2D
                    det = Detection2D()
                    det.header = det_array.header

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = 'face'
                    hyp.hypothesis.score = score
                    det.results.append(hyp)

                    # Supporta sia Pose2D (x,y,theta) sia Pose (position/orientation)
                    center = det.bbox.center
                    if hasattr(center, 'x'):  # Pose2D
                        center.x = cx
                        center.y = cy
                        if hasattr(center, 'theta'):
                            center.theta = 0.0
                    elif hasattr(center, 'position'):  # Pose
                        center.position.x = cx
                        center.position.y = cy
                        # opzionale: z=0 e orientazione identità
                        if hasattr(center, 'position') and hasattr(center.position, 'z'):
                            center.position.z = 0.0
                        if hasattr(center, 'orientation'):
                            # identità (nessuna rotazione)
                            if hasattr(center.orientation, 'x'): center.orientation.x = 0.0
                            if hasattr(center.orientation, 'y'): center.orientation.y = 0.0
                            if hasattr(center.orientation, 'z'): center.orientation.z = 0.0
                            if hasattr(center.orientation, 'w'): center.orientation.w = 1.0

                    det.bbox.size_x = w
                    det.bbox.size_y = h


                    det_array.detections.append(det)

        # Log (rate-limited)
        # now = time.time()
        # if now - self.last_log > 1.5:
        #    self.get_logger().info(f"Detections: {faces}")
        #    self.last_log = now

        # Pubblica immagine annotata
        try:
            if self.visualize:
                img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                img_msg.header = Header()
                img_msg.header.stamp = msg.header.stamp
                img_msg.header.frame_id = msg.header.frame_id
                self.pub_img.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f"Publish image error: {e}")

        # Pubblica ROI array (anche se vuoto)
        try:
            self.pub_rois.publish(det_array)
        except Exception as e:
            self.get_logger().warn(f"Publish ROI error: {e}")


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


if __name__ == '__main__':
    main()

