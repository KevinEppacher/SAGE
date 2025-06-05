from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from lifecycle_msgs.msg import Transition
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor
from seem_ros_interfaces.srv import Panoptic, ObjectSegmentation, SemanticSimilarity
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
import cv2

class ValueMap(LifecycleNode):
    def __init__(self):
        super().__init__('value_map_node')
        self.get_logger().info("Initializing Value Map Node...")

        self.declare_parameter('timer_frequency', 10.0, ParameterDescriptor(description='Frequency of the timer.'))
        self.timer_frequency = self.get_parameter("timer_frequency").get_parameter_value().double_value

        self.timer = None
        self.bridge = CvBridge()

        self.cli_panoptic = None
        self.cli_object_segmentation = None

        self.cli_semantic_similarity = None
        self.text_query = "desk"  # oder später parametrisierbar machen

        self.marker_pub = self.create_publisher(Marker, "/semantic_score_marker", 1)

        self.rgb_image = None

    def on_configure(self, state):
        self.get_logger().info('Configuring Value Map Node...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating...')

        try:
            self.image_sub = self.create_subscription(Image, '/rgb', self.image_callback, 10)

            # Service Clients
            self.cli_panoptic = self.create_client(Panoptic, 'panoptic_segmentation')
            self.cli_object_segmentation = self.create_client(ObjectSegmentation, 'object_segmentation')

            while not self.cli_panoptic.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Waiting for panoptic_segmentation service...')
            self.get_logger().info('Panoptic segmentation service is available.')

            while not self.cli_object_segmentation.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Waiting for object_segmentation service...')
            self.get_logger().info('Object segmentation service is available.')

            self.cli_semantic_similarity = self.create_client(SemanticSimilarity, 'semantic_similarity')

            while not self.cli_semantic_similarity.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Waiting for semantic_similarity service...')
            self.get_logger().info('Semantic similarity service is available.')

            self.createTimer()

            self.get_logger().info('Value Map Node activated.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Activation failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: State):
        self.get_logger().info('Cleaning up...')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def createTimer(self):
        self.timer = self.create_timer(1.0 / self.timer_frequency, self.timer_callback)
        self.get_logger().info(f"Timer started with frequency: {self.timer_frequency} Hz")

    def timer_callback(self):
        if self.rgb_image is None:
            self.get_logger().warn("No RGB image available.")
            return

        try:
            # Call Panoptic Service
            panoptic_req = Panoptic.Request()
            panoptic_req.image = self.rgb_image
            future_panoptic = self.cli_panoptic.call_async(panoptic_req)
            future_panoptic.add_done_callback(self.handle_panoptic_response)

            # Call Object Segmentation Service
            object_req = ObjectSegmentation.Request()
            object_req.image = self.rgb_image
            object_req.query = self.text_query
            future_object = self.cli_object_segmentation.call_async(object_req)
            future_object.add_done_callback(self.handle_object_response)

            # Call Semantic Similarity Service
            similarity_req = SemanticSimilarity.Request()
            similarity_req.image = self.rgb_image
            similarity_req.query = self.text_query
            future_similarity = self.cli_semantic_similarity.call_async(similarity_req)
            future_similarity.add_done_callback(self.handle_similarity_response)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def handle_panoptic_response(self, future):
        try:
            response = future.result()
            seg_image = self.bridge.imgmsg_to_cv2(response.panoptic_segmentation, desired_encoding='rgb8')
            cv2.imshow("Panoptic Segmentation", seg_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Panoptic service call failed: {e}')

    def handle_object_response(self, future):
        try:
            response = future.result()
            obj_image = self.bridge.imgmsg_to_cv2(response.segmented_image, desired_encoding='rgb8')
            cv2.imshow("Object Segmentation", obj_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Object segmentation service call failed: {e}')

    def handle_similarity_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Cosine Similarity Score for '{self.text_query}': {response.score:.4f}")
            self.publish_score_marker(response.score)
        except Exception as e:
            self.get_logger().error(f'Semantic similarity service call failed: {e}')

    def image_callback(self, msg):
        self.rgb_image = msg

    def publish_score_marker(self, score: float):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "semantic_score"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5  # Halbe Höhe, damit es auf Boden steht
        marker.pose.orientation.w = 1.0

        # Skaliere Durchmesser proportional zum Score, Höhe konstant
        diameter = max(0.01, float(score)*5)  # Mindestgröße zur Anzeige
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = 1.0  # konstante Höhe

        marker.color.a = 0.8
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 0.2

        self.marker_pub.publish(marker)
