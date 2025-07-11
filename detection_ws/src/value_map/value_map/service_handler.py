from seem_ros_interfaces.srv import Panoptic, ObjectSegmentation, SemanticSimilarity
import rclpy
from cv_bridge import CvBridge
import cv2

#from vlm_interface.srv import SemanticSimilarity

# ANSI color codes
BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"

class ServiceHandler:
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()

        self.node.declare_parameter('vlm_target_namespace', '/seem_ros')
        self.vlm_namespace = self.node.get_parameter('vlm_target_namespace').get_parameter_value().string_value.rstrip('/')

        def resolve(service: str) -> str:
            return f'{self.vlm_namespace}/{service}'

        # Create clients
        self.cli_panoptic = self.node.create_client(Panoptic, resolve('panoptic_segmentation'))
        self.cli_object_segmentation = self.node.create_client(ObjectSegmentation, resolve('object_segmentation'))
        self.cli_semantic_similarity = self.node.create_client(SemanticSimilarity, resolve('semantic_similarity'))

        self.node.get_logger().info(
            f"{BLUE}{BOLD}ServiceHandler initialized with VLM namespace: {self.vlm_namespace}{RESET}"
        )

        self.panoptic_segemented_image = None
        self.object_segmented_image = None
        self.semantic_similarity_score = None

    def call_panoptic(self, image_msg):
        request = Panoptic.Request()
        request.image = image_msg
        future = self.cli_panoptic.call_async(request)
        future.add_done_callback(self.handle_panoptic_response)
        return self.panoptic_segemented_image

    def call_object_segmentation(self, image_msg, query):
        request = ObjectSegmentation.Request()
        request.image = image_msg
        request.query = query
        future = self.cli_object_segmentation.call_async(request)
        future.add_done_callback(self.handle_object_response)
        return self.object_segmented_image

    def call_semantic_similarity(self, image_msg, query):
        request = SemanticSimilarity.Request()
        request.image = image_msg
        request.query = query
        future = self.cli_semantic_similarity.call_async(request)
        future.add_done_callback(self.handle_similarity_response)
        return self.semantic_similarity_score

    def handle_panoptic_response(self, future):
        try:
            response = future.result()
            self.panoptic_segemented_image = response.panoptic_segmentation
            image = self.bridge.imgmsg_to_cv2(response.panoptic_segmentation, desired_encoding='rgb8')
            cv2.imshow("Panoptic Segmentation", image)
            cv2.waitKey(1)
        except Exception as e:
            self.panoptic_segemented_image = None
            self.node.get_logger().error(f"Panoptic response error: {e}")

    def handle_object_response(self, future):
        try:
            response = future.result()
            self.object_segmented_image = response.segmented_image
            image = self.bridge.imgmsg_to_cv2(response.segmented_image, desired_encoding='rgb8')
            cv2.imshow("Object Segmentation", image)
            cv2.waitKey(1)
        except Exception as e:
            self.object_segmented_image = None
            self.node.get_logger().error(f"Object response error: {e}")

    def handle_similarity_response(self, future):
        try:
            response = future.result()
            self.semantic_similarity_score = response.score
        except Exception as e:
            self.semantic_similarity_score = None
            self.node.get_logger().error(f"Similarity response error: {e}")