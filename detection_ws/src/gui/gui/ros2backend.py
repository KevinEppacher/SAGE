from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
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
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Keeps last message in publisher's cache
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.prompt_pub = self.create_publisher(SemanticPrompt, '/user_prompt', qos_semantic_prompt)
        self.bridge = CvBridge()
        self.current_prompt = SemanticPrompt()
        self.current_prompt.header.frame_id = "semantic_prompt"
        self.get_logger().info("ROS2Backend initialized with empty SemanticPrompt.")

        self.create_timer(1.0, self.publish_semantic_prompt)  # Update header every second

    def _update_header(self):
        self.current_prompt.header.stamp = self.get_clock().now().to_msg()
        self.current_prompt.header.frame_id = "uploaded_prompt"

    def publish_text(self, text: str):
        """Update text_query field and publish full SemanticPrompt."""
        self.current_prompt.text_query = text
        self._update_header()
        self.prompt_pub.publish(self.current_prompt)
        self.get_logger().info("Published SemanticPrompt with updated text_query.")

    def publish_image(self, image_path: str):
        """Convert image and update image_query field in SemanticPrompt."""
        
        # Überprüfen, ob das Bild erfolgreich geladen wurde
        cv_image = cv2.imread(image_path)
        
        # if cv_image is not None:
        #     cv2.imshow('Gelesenes Bild', cv_image)
        #     cv2.waitKey(0)
        #     cv2.destroyAllWindows()
        # else:
        #     print("Fehler: Bild konnte nicht geladen werden.")

        # print(f"Loading from: {image_path}")
        # print(f"Exists? {os.path.exists(image_path)}")
        # print(f"Image shape: {cv_image.shape if cv_image is not None else 'None'}")
        # print("Min pixel value:", cv_image.min())
        # print("Max pixel value:", cv_image.max())
        # print("dtype:", cv_image.dtype)
        # print("shape:", cv_image.shape)

        if cv_image.sum() == 0:
            self.get_logger().warn("Loaded image is completely black.")

        if cv_image is None:
            self.get_logger().error(f"Failed to load image from {image_path}")
            return

        # import time
        # time.sleep(1)
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "uploaded_image"

        self.current_prompt.image_query = image_msg
        self._update_header()
        # self.prompt_pub.publish(self.current_prompt)
        # self.get_logger().info("Published SemanticPrompt with updated image_query.")

    def publish_semantic_prompt(self):
        """Publish the current SemanticPrompt with updated header."""
        self._update_header()
        self.prompt_pub.publish(self.current_prompt)
        self.get_logger().info("Published SemanticPrompt with updated header.")