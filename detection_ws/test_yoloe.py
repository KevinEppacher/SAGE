#!/usr/bin/env python3
# yoloe_rgb_node.py
# Run: source /opt/ros/humble/setup.bash && python3 yoloe_rgb_node.py
# Deps: sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs
#       pip install ultralytics opencv-python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
from ultralytics import YOLOE
import numpy as np
from geometry_msgs.msg import Pose2D

class YOLOERGBNode(Node):
    def __init__(self):
        super().__init__("yoloe_rgb_node")

        # --- Parameters (can be overridden via ROS params) ---
        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("weights", "yoloe-11l-seg.pt")  # use *-seg for masks
        self.declare_parameter("prompts", ["oven", "chair","tv","bed", "door"])
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.50)
        self.declare_parameter("publish_overlay", True)
        self.declare_parameter("publish_detections", True)

        self.rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self.weights = self.get_parameter("weights").get_parameter_value().string_value
        self.prompts = [s for s in self.get_parameter("prompts").get_parameter_value().string_array_value]
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.publish_overlay = bool(self.get_parameter("publish_overlay").value)
        self.publish_detections = bool(self.get_parameter("publish_detections").value)

        # --- Load YOLOE and set open-vocabulary prompts ---
        # Note: set_classes enables zero-shot text prompts.
        self.model = YOLOE(self.weights)
        self.model.set_classes(self.prompts, self.model.get_text_pe(self.prompts))
        t = self.model.get_text_pe(self.prompts)

        self.bridge = CvBridge()

        # QoS tuned for camera streams
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.sub = self.create_subscription(Image, self.rgb_topic, self.on_image, qos)

        if self.publish_overlay:
            self.pub_overlay = self.create_publisher(Image, "yoloe/overlay", 1)
        if self.publish_detections:
            self.pub_det = self.create_publisher(Detection2DArray, "yoloe/detections", 1)

        self.get_logger().info(f"YOLOE loaded: {self.weights}, prompts={self.prompts}, topic={self.rgb_topic}")

    def on_image(self, msg: Image):
        # Convert ROS Image -> OpenCV BGR8
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # Inference (numpy array input)
        try:
            res = self.model.predict(
                source=frame,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                verbose=False
            )[0]
        except Exception as e:
            self.get_logger().error(f"YOLOE inference failed: {e}")
            return

        # Publish overlay image
        if self.publish_overlay:
            overlay = res.plot()  # annotated BGR image as np.ndarray
            out_img = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            out_img.header = msg.header
            self.pub_overlay.publish(out_img)

        # Publish Detection2DArray
        if self.publish_detections:
            det_arr = Detection2DArray()
            det_arr.header = msg.header

            if res.boxes is not None and len(res.boxes) > 0:
                names = res.names  # id -> class name mapping
                for b in res.boxes:
                    xyxy = b.xyxy[0].tolist()          # [x1,y1,x2,y2]
                    x1, y1, x2, y2 = map(float, xyxy)
                    w, h = (x2 - x1), (y2 - y1)
                    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

                    conf = float(b.conf[0])
                    cls_id = int(b.cls[0]) if b.cls is not None else -1
                    cls_name = names.get(cls_id, str(cls_id))

                    # Create Detection2D
                    det = Detection2D()
                    det.header = msg.header

                    # Classification hypothesis
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(res.names.get(cls_id, str(cls_id)))  # string label
                    hyp.hypothesis.score = float(conf)
                    det.results.append(hyp)

                    # Bounding box in pixel units
                    det.bbox = BoundingBox2D()

                    center = det.bbox.center
                    if hasattr(center, "x"):
                        center.x, center.y, center.theta = float(cx), float(cy), 0.0
                    elif hasattr(center, "position"):  # rare pose-type center
                        center.position.x, center.position.y = float(cx), float(cy)
                        det.bbox.center = center

                    det.bbox.size_x = float(w)
                    det.bbox.size_y = float(h)

                    # Optional: place class name into Detection2D id field via string?
                    # vision_msgs/Detection2D has no string label; keep mapping externally if needed.

                    det_arr.detections.append(det)

            self.pub_det.publish(det_arr)

def main():
    rclpy.init()
    node = YOLOERGBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
