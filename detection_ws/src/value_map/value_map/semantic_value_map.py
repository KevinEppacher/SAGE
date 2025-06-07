from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
import struct
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
import numpy as np
import math
from value_map.utils import normalize_angle, get_yaw_angle
from tf_transformations import euler_from_quaternion

class SemanticValueMap:
    def __init__(self, node: Node):
        self.node = node
        self.map = None
        # Publishers
        self.publisher = node.create_publisher(PointCloud2, '/value_map', 10)

        # Subscribers
        self.subscriber = node.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def update_semantic_map(self, semantic_similarity_score: float, pose : Pose):
        if self.map is None:
            self.node.get_logger().warn("No occupancy grid map received yet.")
            return

        fov_mask = self.generate_topdown_cone_mask(
            pose=pose,
            grid=self.map,
            fov_deg=90,
            max_range=5.0
        )

        confidence_mask = self.compute_confidence_map(pose, fov_mask, 90, grid=self.map)

        msg = self.map
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        resolution = msg.info.resolution
        origin = msg.info.origin
        width = msg.info.width
        height = msg.info.height

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
                value = msg.data[idx]

                if value >= 0 and fov_mask[row, col] == 1:
                    x = origin.position.x + col * resolution
                    y = origin.position.y + row * resolution
                    z = 0.0

                    confidence = confidence_mask[row, col]
                    intensity = int(confidence * semantic_similarity_score * 5 * 255)
                    r = max(0, min(255, intensity))
                    g = 0
                    b = 0

                    rgb = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
                    points.append((x, y, z, rgb))

        cloud = pc2.create_cloud(header, fields, points)
        self.publisher.publish(cloud)

    def generate_topdown_cone_mask(self, *, pose: Pose, grid: OccupancyGrid, fov_deg: float, max_range: float) -> np.ndarray:
        """Generate a FOV cone mask using raytracing that stops on occupied cells."""

        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin = grid.info.origin

        fov_mask = np.zeros((height, width), dtype=np.uint8)

        robot_x = (pose.position.x - origin.position.x) / resolution
        robot_y = (pose.position.y - origin.position.y) / resolution

        yaw = get_yaw_angle(pose)
        fov_rad = math.radians(fov_deg)
        half_fov = fov_rad / 2
        max_range_px = int(max_range / resolution)

        num_rays = 200  # number of rays within the FOV
        for i in range(num_rays):
            angle = yaw - half_fov + (i / (num_rays - 1)) * fov_rad
            dx = math.cos(angle)
            dy = math.sin(angle)

            for step in range(max_range_px):
                px = int(robot_x + dx * step)
                py = int(robot_y + dy * step)

                if not (0 <= px < width and 0 <= py < height):
                    break

                idx = py * width + px
                if grid.data[idx] == -1:
                    continue  # unknown
                elif grid.data[idx] >= 50:
                    fov_mask[py, px] = 1  # mark the hit cell
                    break  # stop ray at occupied cell
                else:
                    fov_mask[py, px] = 1  # mark free cell

        return fov_mask

    def compute_confidence_map(self, pose: Pose, fov_mask: np.ndarray, camera_fov: float, grid: OccupancyGrid):
        """Returns a confidence map (float32) based on cos²(theta) inside FOV."""
        height, width = fov_mask.shape
        confidence_map = np.zeros_like(fov_mask, dtype=np.float32)

        resolution = grid.info.resolution
        origin = grid.info.origin

        # Pose des Roboters in Map-Koordinaten (Pixel)
        robot_x = (pose.position.x - origin.position.x) / resolution
        robot_y = (pose.position.y - origin.position.y) / resolution

        # Yaw aus Quaternion extrahieren
        yaw = get_yaw_angle(pose)

        half_fov = math.radians(camera_fov) / 2

        for row in range(height):
            for col in range(width):
                if fov_mask[row, col] == 0:
                    continue

                dx = col - robot_x
                dy = row - robot_y
                angle = math.atan2(dy, dx)
                angle_diff = normalize_angle(angle - yaw)
                scaled_angle = angle_diff / half_fov * (math.pi / 2)

                # cos² weighting
                if abs(angle_diff) <= half_fov:
                    weight = math.cos(scaled_angle) ** 2
                    confidence_map[row, col] = weight

        return confidence_map

        
