Rakuda_examples

face_detector.py

/camera/image_raw               →  Nodo di pubblicazione video
/face_detector/image_faces      ←  Nodo di face detection (es. mediapipe o RetinaFace)
/face_recognizer/recognized     ←  Nodo di face recognition (es. InsightFace o FaceNet)
/depth_estimator/distance       ←  Nodo per calcolo distanza (da depth o monoculare)
/face_display/overlay           →  Nodo di visualizzazione (OpenCV + rqt_image_view)
