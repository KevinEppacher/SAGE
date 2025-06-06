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

def occupancy_grid_to_pointcloud(occupancy_grid):
    header = Header()
    header.stamp = occupancy_grid.header.stamp
    header.frame_id = occupancy_grid.header.frame_id

    resolution = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    points = []
    for row in range(height):
        for col in range(width):
            idx = row * width + col
            value = occupancy_grid.data[idx]
            if value > 0:  # Only include occupied or unknown
                x = origin.position.x + col * resolution
                y = origin.position.y + row * resolution
                z = 0.0
                r = int((100 - value) * 2.55)  # Red for low certainty
                g = int(value * 2.55)          # Green for high certainty
                b = 0
                rgb = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
                points.append((x, y, z, rgb))

    return pc2.create_cloud(header, fields, points)

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
            self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

            # Publishers
            self.publisher = self.create_publisher(PointCloud2, '/value_map', 10)

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

    def image_callback(self, msg):
        self.rgb_image = msg

    def timer_callback(self):
        if self.rgb_image is None:
            self.get_logger().warn("No RGB image available.")
            return
        
        if self.map is None:
            self.get_logger().warn("No occupancy grid map received yet.")
            return

        try:
            panoptic_segmented_image = self.service_handler.call_panoptic(self.rgb_image)
            object_segmented_image = self.service_handler.call_object_segmentation(self.rgb_image, self.text_query)
            semantic_similarity_score = self.service_handler.call_semantic_similarity(self.rgb_image, self.text_query)
            self.get_logger().info(f"semantic_similarity_score: {semantic_similarity_score}")
        except Exception as e:
            self.get_logger().error(f"Service calls failed: {e}")

        try:
            transform = self.tf_buffer.lookup_transform("map", "camera_link", rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        cloud_msg = occupancy_grid_to_pointcloud(self.map)
        self.publisher.publish(cloud_msg)

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg