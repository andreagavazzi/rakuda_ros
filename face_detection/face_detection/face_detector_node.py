import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FaceDetectorDNN(Node):
    def __init__(self):
        super().__init__('face_detector_dnn')

        # Bridge ROS ↔ OpenCV
        self.bridge = CvBridge()

        # Subscriber: RGB camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Publisher: image with detections
        self.publisher = self.create_publisher(Image, '/face_detector/image_faces', 10)

        # Load DNN model
        model_path = "/home/ubuntu/models/face_detector/"
        prototxt = model_path + "deploy.prototxt"
        weights = model_path + "res10_300x300_ssd_iter_140000.caffemodel"

        self.net = cv2.dnn.readNetFromCaffe(prototxt, weights)

        # ✅ Use CUDA backend (optimized for Jetson)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

        self.get_logger().info("✅ FaceDetectorDNN node started with CUDA acceleration.")

    def image_callback(self, msg):
        # Convert ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = frame.shape[:2]

        # Preprocess the image for the network
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,
                                     (300, 300), (104.0, 177.0, 123.0))

        # Forward pass
        self.net.setInput(blob)
        detections = self.net.forward()

        # Process detections
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.6:  # Confidence threshold
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (x1, y1, x2, y2) = box.astype("int")

                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{confidence*100:.1f}%"
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(out_msg)

        # Local preview (optional)
        cv2.imshow("Face Detection (CUDA)", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorDNN()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
