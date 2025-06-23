from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from multimodal_query_msgs.msg import SemanticPrompt

class ROS2Backend(Node):
    """Handles all ROS2 publishers and message formatting."""
    def __init__(self):
        super().__init__('flask_node')
        self.text_pub = self.create_publisher(SemanticPrompt, '/user_text', 10)
        self.image_pub = self.create_publisher(Image, '/uploaded_image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/uploaded_image/camera_info', 10)
        self.bridge = CvBridge()

    def publish_text(self, text: str):
        msg = SemanticPrompt()
        msg.text_query = text
        self.text_pub.publish(msg)
        self.get_logger().info(f"Published user text: '{text}'")

    def publish_image(self, image_path: str):
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            self.get_logger().error(f"Failed to load image from {image_path}")
            return

        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "uploaded_camera"
        self.image_pub.publish(image_msg)

        info_msg = CameraInfo()
        info_msg.header = image_msg.header
        info_msg.height = cv_image.shape[0]
        info_msg.width = cv_image.shape[1]
        cx = info_msg.width / 2.0
        cy = info_msg.height / 2.0
        info_msg.k = [1.0, 0.0, cx, 0.0, 1.0, cy, 0.0, 0.0, 1.0]
        info_msg.p = [1.0, 0.0, cx, 0.0, 0.0, 1.0, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(info_msg)

        self.get_logger().info(f"Published image and CameraInfo from: {image_path}")
