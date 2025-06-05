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
from value_map.service_handler import ServiceHandler

class ValueMap(LifecycleNode):
    def __init__(self):
        super().__init__('value_map_node')
        self.get_logger().info("Initializing Value Map Node...")

        self.declare_parameter('timer_frequency', 10.0, ParameterDescriptor(description='Frequency of the timer.'))
        self.timer_frequency = self.get_parameter("timer_frequency").get_parameter_value().double_value

        self.timer = None
        self.bridge = CvBridge()

        self.text_query = "desk"
        self.rgb_image = None

        self.marker_pub = self.create_publisher(Marker, "/semantic_score_marker", 1)
        self.service_handler = None
        self.get_logger().info("Finished initializing Value Map Node")

    def on_configure(self, state):
        self.get_logger().info('Configuring Value Map Node...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating...')

        try:
            self.image_sub = self.create_subscription(Image, '/rgb', self.image_callback, 10)

            # Initialize service handler with this node
            self.service_handler = ServiceHandler(self)

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
            self.service_handler.call_panoptic(self.rgb_image)
            self.service_handler.call_object_segmentation(self.rgb_image, self.text_query)
            self.service_handler.call_semantic_similarity(self.rgb_image, self.text_query)
        except Exception as e:
            self.get_logger().error(f"Service calls failed: {e}")

    def image_callback(self, msg):
        self.rgb_image = msg
