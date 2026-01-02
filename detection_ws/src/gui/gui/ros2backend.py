from rclpy.node import Node
from cv_bridge import CvBridge
import cv2

from multimodal_query_msgs.msg import SemanticPrompt
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


class ROS2Backend(Node):
    """Handles ROS2 publishing of semantic prompts (text/image)."""

    def __init__(self):
        super().__init__('gui_node')

        qos_semantic_prompt = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publishers ---------------------------------------------------
        self.prompt_pub = self.create_publisher(
            SemanticPrompt, '/user_prompt', qos_semantic_prompt
        )
        self.evaluator_prompt_pub = self.create_publisher(
            SemanticPrompt, '/evaluator/prompt', qos_semantic_prompt
        )
        self.zero_shot_prompt_pub = self.create_publisher(
            SemanticPrompt, '/zero_shot_prompt', qos_semantic_prompt
        )

        self.bridge = CvBridge()
        self.get_logger().info("ROS2Backend initialized.")

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #
    def _build_prompt(self, text: str) -> SemanticPrompt:
        msg = SemanticPrompt()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gui_prompt"
        msg.text_query = text
        return msg

    # ------------------------------------------------------------------ #
    # Public API for GUI
    # ------------------------------------------------------------------ #
    def publish_user_prompt(self, text: str):
        msg = self._build_prompt(text)
        self.prompt_pub.publish(msg)
        self.get_logger().info(f"Published USER prompt: {text}")

    def publish_evaluator_prompt(self, text: str):
        msg = self._build_prompt(text)
        self.evaluator_prompt_pub.publish(msg)
        self.get_logger().info(f"Published EVALUATOR prompt: {text}")

    def publish_zero_shot_prompt(self, text: str):
        msg = self._build_prompt(text)
        self.zero_shot_prompt_pub.publish(msg)
        self.get_logger().info(f"Published ZERO-SHOT prompt: {text}")

    def publish_image(self, image_path: str):
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            self.get_logger().error(f"Failed to load image from {image_path}")
            return

        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "uploaded_image"

        msg = SemanticPrompt()
        msg.header = image_msg.header
        msg.image_query = image_msg

        self.prompt_pub.publish(msg)
        self.get_logger().info("Published image prompt.")
