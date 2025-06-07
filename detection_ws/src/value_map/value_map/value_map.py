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
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from octomap_msgs.msg import Octomap
from octomap_msgs.msg import OctomapWithPose
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from value_map.semantic_value_map import SemanticValueMap

class ValueMap(LifecycleNode):
    def __init__(self):
        super().__init__('value_map_node')
        self.get_logger().info("Initializing Value Map Node...")

        self.declare_parameter('timer_frequency', 1.0, ParameterDescriptor(description='Frequency of the timer.'))
        self.timer_frequency = self.get_parameter("timer_frequency").get_parameter_value().double_value

        self.timer = None
        self.bridge = CvBridge()

        self.text_query = "Seems like there is a chair ahead"
        self.rgb_image = None
        self.map = None

        self.marker_pub = self.create_publisher(Marker, "/semantic_score_marker", 1)
        self.service_handler = None
        self.get_logger().info("Finished initializing Value Map Node")

    def on_configure(self, state):
        self.get_logger().info('Configuring Value Map Node...')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating...')

        try:
            # Subscribers
            self.image_sub = self.create_subscription(Image, '/rgb', self.image_callback, 10)

            # Initialize service handler with this node
            self.service_handler = ServiceHandler(self)

            self.semantic_map = SemanticValueMap(self)

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

    def image_callback(self, msg):
        self.rgb_image = msg

    def timer_callback(self):
        if self.rgb_image is None:
            self.get_logger().warn("No RGB image available.")
            return
        
        current_pose = self.get_pose()

        if current_pose is None:
            self.get_logger().warn("Current pose could not be retrieved.")
            return

        try:
            panoptic_segmented_image = self.service_handler.call_panoptic(self.rgb_image)
            object_segmented_image = self.service_handler.call_object_segmentation(self.rgb_image, self.text_query)
            semantic_similarity_score = self.service_handler.call_semantic_similarity(self.rgb_image, self.text_query)
            
            self.publish_score_marker(semantic_similarity_score)
        except Exception as e:
            self.get_logger().error(f"Service calls failed: {e}")

        if semantic_similarity_score is None:
            self.get_logger().warn("No semantic similarity score received.")
            return

        self.semantic_map.update_semantic_map(semantic_similarity_score, current_pose)

    def get_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            pose = Pose()
            pose.position.x = translation.x
            pose.position.y = translation.y
            pose.position.z = translation.z
            pose.orientation.x = rotation.x
            pose.orientation.y = rotation.y
            pose.orientation.z = rotation.z
            pose.orientation.w = rotation.w
            return pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None
        
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