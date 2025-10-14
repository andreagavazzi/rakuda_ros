# file: face_detection/face_detector_node.py
# ROS 2 Humble + Python — RetinaFace (MobileNet-0.25 ONNX) with OpenCV DNN
# Publishes:
#  - /face_detector/image_faces (sensor_msgs/Image) : image with bboxes
#  - /face_detector/detections  (vision_msgs/Detection2DArray) : boxes + scores
#
# Params:
#  - model_path: full path to retinaface_mnet0.25.onnx
#  - conf_threshold: float (default 0.6)
#  - nms_threshold: float (default 0.4)
#  - top_k: int (default 5000)

import os
import math
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D

# -------------------------
# RetinaFace utilities
# -------------------------

def generate_anchors(img_h, img_w, strides=(8, 16, 32), ratios=(1.0,), scales=(1.0,)):
    """Generate anchors for three feature levels following RetinaFace defaults."""
    anchors_all = []
    for s in strides:
        feat_h = math.ceil(img_h / s)
        feat_w = math.ceil(img_w / s)
        # per-level base sizes (per paper: 2 scales per level: 2,3; here we approximate with fixed sizes)
        # Common practical variant for mnet0.25: three sizes per level
        if s == 8:
            sizes = [16, 32]
        elif s == 16:
            sizes = [64, 128]
        else:
            sizes = [256, 512]
        for y in range(feat_h):
            for x in range(feat_w):
                cx = (x + 0.5) * s
                cy = (y + 0.5) * s
                for size in sizes:
                    w = size
                    h = size
                    anchors_all.append([cx, cy, w, h, s])
    return np.array(anchors_all, dtype=np.float32)

def decode_bboxes(deltas, anchors, variance=(0.1, 0.2)):
    """Decode predicted offsets back to absolute boxes [x1,y1,x2,y2]."""
    cxcy = anchors[:, :2]
    wh = anchors[:, 2:4]
    dxdy = deltas[:, :2]
    dwdh = deltas[:, 2:4]

    decoded_cxcy = dxdy * variance[0] * wh + cxcy
    decoded_wh   = np.exp(dwdh * variance[1]) * wh

    x1y1 = decoded_cxcy - decoded_wh * 0.5
    x2y2 = decoded_cxcy + decoded_wh * 0.5
    return np.hstack([x1y1, x2y2])

def decode_landmarks(dlm, anchors, variance=(0.1, 0.2)):
    """Decode 5 landmarks (x,y)*5 from deltas."""
    cxcy = anchors[:, :2]
    wh = anchors[:, 2:4]
    l = dlm.reshape((-1, 5, 2))
    l = l * variance[0] * wh[:, None, :] + cxcy[:, None, :]
    return l.reshape((-1, 10))

def nms(boxes, scores, iou_thresh=0.4, top_k=2000):
    """Non-maximum suppression (numpy)."""
    if len(boxes) == 0:
        return []
    x1 = boxes[:,0]; y1 = boxes[:,1]; x2 = boxes[:,2]; y2 = boxes[:,3]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]
    if top_k is not None:
        order = order[:top_k]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(iou <= iou_thresh)[0]
        order = order[inds + 1]
    return keep

class RetinaFaceONNX:
    def __init__(self, onnx_path, use_cuda=True):
        self.net = cv2.dnn.readNet(onnx_path)
        # Try CUDA FP16, fallback to CPU
        try:
            if use_cuda:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
            else:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        except Exception:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def infer(self, img_bgr, conf_thresh=0.6, nms_thresh=0.4, top_k=5000):
        """
        Returns: boxes (N,4), scores (N,), landmarks (N,10)
        Boxes in absolute pixel coords [x1,y1,x2,y2].
        """
        h, w = img_bgr.shape[:2]
        # RetinaFace default preprocessing: resize keep-aspect? Here feed original; model expects BGR mean subtraction if needed.
        # Common ONNX export for mnet0.25 expects [B,3,H,W], scale 1.0, mean subtraction (104,117,123) optional.
        blob = cv2.dnn.blobFromImage(img_bgr, 1.0, (w, h), (104, 117, 123), swapRB=False, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())

        # Expect three outputs: bbox regression, classification, landmarks
        # Try to map by shapes
        loc, conf, landms = None, None, None
        for out in outs:
            s = out.shape
            if len(s) == 4:
                out = out.reshape(s[0], -1, s[-1])
            if out.shape[-1] == 4:
                loc = out[0]
            elif out.shape[-1] == 2:
                conf = out[0]
            elif out.shape[-1] == 10:
                landms = out[0]

        if loc is None or conf is None or landms is None:
            # Some exports provide named outputs; try fixed order
            if len(outs) == 3:
                loc, conf, landms = outs[0][0], outs[1][0], outs[2][0]
            else:
                raise RuntimeError("Unexpected ONNX outputs. Check your model export.")

        scores = conf[:,1]  # foreground class score
        # Build anchors for current image size
        anchors = generate_anchors(h, w)
        if loc.shape[0] != anchors.shape[0]:
            # If model expects padded/strided resolution, fallback: resize to divisible input
            # Resize to nearest multiple of 32
            new_w = int(math.ceil(w / 32) * 32)
            new_h = int(math.ceil(h / 32) * 32)
            blob2 = cv2.dnn.blobFromImage(cv2.resize(img_bgr, (new_w, new_h)), 1.0, (new_w, new_h), (104,117,123), swapRB=False, crop=False)
            self.net.setInput(blob2)
            outs = self.net.forward(self.net.getUnconnectedOutLayersNames())
            loc, conf, landms = outs[0][0], outs[1][0], outs[2][0]
            scores = conf[:,1]
            anchors = generate_anchors(new_h, new_w)

            # We'll decode in resized space then scale back to original
            ratio_w = w / new_w
            ratio_h = h / new_h
            boxes_abs = decode_bboxes(loc, anchors)
            lmk_abs   = decode_landmarks(landms, anchors)
            # Filter by confidence
            keep = np.where(scores >= conf_thresh)[0]
            boxes_abs = boxes_abs[keep]
            lmk_abs   = lmk_abs[keep]
            scores_kept = scores[keep]
            # NMS
            keep2 = nms(boxes_abs, scores_kept, nms_thresh, top_k)
            boxes_abs = boxes_abs[keep2]; lmk_abs = lmk_abs[keep2]; scores_kept = scores_kept[keep2]
            # Scale back
            boxes_abs[:, [0,2]] *= ratio_w
            boxes_abs[:, [1,3]] *= ratio_h
            lmk_abs[:, 0::2] *= ratio_w
            lmk_abs[:, 1::2] *= ratio_h
            return boxes_abs, scores_kept, lmk_abs

        # Normal path (no resize fallback)
        boxes_abs = decode_bboxes(loc, anchors)
        lmk_abs   = decode_landmarks(landms, anchors)
        keep = np.where(scores >= conf_thresh)[0]
        boxes_abs = boxes_abs[keep]
        lmk_abs   = lmk_abs[keep]
        scores_kept = scores[keep]
        keep2 = nms(boxes_abs, scores_kept, nms_thresh, top_k)
        return boxes_abs[keep2], scores_kept[keep2], lmk_abs[keep2]

# -------------------------
# ROS 2 Node
# -------------------------

class RetinaFaceDetectorNode(Node):
    def __init__(self):
        super().__init__('retinaface_detector')

        # QoS optimized for video
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.bridge = CvBridge()

        # Params
        self.declare_parameter('model_path', '/home/ubuntu/models/retinaface/retinaface_mnet0.25.onnx')
        self.declare_parameter('conf_threshold', 0.6)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('top_k', 5000)
        self.declare_parameter('subscribe_topic', '/camera/image_raw')
        self.declare_parameter('publish_image_topic', '/face_detector/image_faces')
        self.declare_parameter('publish_dets_topic', '/face_detector/detections')
        self.declare_parameter('use_cuda', True)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        conf_th = float(self.get_parameter('conf_threshold').value)
        nms_th = float(self.get_parameter('nms_threshold').value)
        top_k = int(self.get_parameter('top_k').value)
        use_cuda = bool(self.get_parameter('use_cuda').value)

        self.conf_th = conf_th
        self.nms_th = nms_th
        self.top_k = top_k

        if not os.path.isfile(model_path):
            self.get_logger().error(f"Model not found at: {model_path}")
            raise FileNotFoundError(model_path)

        self.rf = RetinaFaceONNX(model_path, use_cuda=use_cuda)
        self.get_logger().info("✅ RetinaFace ONNX loaded (CUDA FP16 if available).")

        sub_topic = self.get_parameter('subscribe_topic').value
        self.img_sub = self.create_subscription(Image, sub_topic, self.image_cb, qos)

        pub_img_topic = self.get_parameter('publish_image_topic').value
        pub_det_topic = self.get_parameter('publish_dets_topic').value
        self.img_pub = self.create_publisher(Image, pub_img_topic, qos)
        self.det_pub = self.create_publisher(Detection2DArray, pub_det_topic, qos)

        self.get_logger().info(f"🔎 Subscribing: {sub_topic}")
        self.get_logger().info(f"🖼️ Publishing annotated: {pub_img_topic}")
        self.get_logger().info(f"📦 Publishing detections: {pub_det_topic}")

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        t0 = time.time()
        boxes, scores, landmarks = self.rf.infer(frame, self.conf_th, self.nms_th, self.top_k)
        t1 = time.time()
        fps = 1.0 / max(1e-6, (t1 - t0))

        # Build Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header

        for i in range(len(scores)):
            x1, y1, x2, y2 = boxes[i]
            # Clamp
            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w - 1, x2))
            y2 = max(0, min(h - 1, y2))
            bw = max(1.0, x2 - x1)
            bh = max(1.0, y2 - y1)

            det = Detection2D()
            det.header = msg.header
            det.bbox = BoundingBox2D()
            det.bbox.center.x = float(x1 + bw / 2.0)
            det.bbox.center.y = float(y1 + bh / 2.0)
            det.bbox.size_x = float(bw)
            det.bbox.size_y = float(bh)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "face"
            hyp.hypothesis.score = float(scores[i])
            det.results.append(hyp)

            det_array.detections.append(det)

            # Draw
            pt1 = (int(x1), int(y1)); pt2 = (int(x2), int(y2))
            cv2.rectangle(frame, pt1, pt2, (0, 255, 0), 2)
            cv2.putText(frame, f"{scores[i]*100:.1f}%", (int(x1), int(y1)-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

            # (Optional) draw landmarks (5 points)
            lm = landmarks[i].reshape(5,2).astype(np.int32)
            for (lx, ly) in lm:
                cv2.circle(frame, (lx, ly), 1, (0, 255, 0), 2)

        # FPS overlay (optional)
        cv2.putText(frame, f"RetinaFace | FPS: {fps:.1f}", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publish
        self.det_pub.publish(det_array)
        out_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_img.header = msg.header
        self.img_pub.publish(out_img)

def main(args=None):
    rclpy.init(args=args)
    node = RetinaFaceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

