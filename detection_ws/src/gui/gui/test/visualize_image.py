import rclpy
from rclpy.node import Node
from multimodal_query_msgs.msg import SemanticPrompt
from cv_bridge import CvBridge
import cv2

class SemanticPromptViewer(Node):
    def __init__(self):
        super().__init__('semantic_prompt_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            SemanticPrompt,
            '/user_prompt',
            self.listener_callback,
            10
        )
        self.get_logger().info("SemanticPromptViewer node started.")

    def listener_callback(self, msg: SemanticPrompt):
        if msg.image_query.data:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg.image_query, desired_encoding='bgr8')
                self.get_logger().info("Received image with height: %d, width: %d" % (cv_image.shape[0], cv_image.shape[1]))
                cv2.imshow("SemanticPrompt Image", cv_image)
                cv2.waitKey(0)
                self.get_logger().info(f"Received image from topic /user_prompt")
            except Exception as e:
                self.get_logger().error(f"Failed to convert image: {e}")
        else:
            self.get_logger().warn("SemanticPrompt contains no image.")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticPromptViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
