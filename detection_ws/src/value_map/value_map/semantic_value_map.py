from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
import struct
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
import numpy as np
import math
from value_map.utils import normalize_angle, get_yaw_angle, value_to_inferno_rgb
from sensor_msgs.msg import CameraInfo

class SemanticValueMap:
    def __init__(self, node: Node):
        self.node = node
        self.map = None
        self.value_map = None
        self.confidence_map = None
        self.fx = None
        self.width = None
        self.max_semantic_score = 0

        # Publishers
        self.value_map_inferno_pub = node.create_publisher(PointCloud2, 'value_map', 10)
        self.value_map_raw_pub = node.create_publisher(PointCloud2, 'value_map_raw', 10)

        # Subscribers
        self.subscriber = node.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.camera_info_sub = node.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)

    def map_callback(self, msg: OccupancyGrid):
        self.resize_maps_preserving_values(msg)

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]  # focal length in x
        self.width = msg.width

    def update_semantic_map(self, semantic_similarity_score: float, pose : Pose):
        if self.map is None:
            self.node.get_logger().warn("No occupancy grid map received yet.")
            return
        
        fov_deg = self.get_horizontal_fov()
        
        # --- Step 1: Decay previous values ---
        decay_factor = 0.95
        self.value_map *= decay_factor
        self.confidence_map *= decay_factor

        fov_mask = self.generate_topdown_cone_mask(
            pose=pose,
            grid=self.map,
            fov_deg=fov_deg,
            max_range=5.0
        )

        confidence = self.compute_confidence_map(pose, fov_mask, fov_deg, grid=self.map)

        # Fusion logic
        for row in range(self.map.info.height):
            for col in range(self.map.info.width):
                if fov_mask[row, col] == 0:
                    continue

                prev_value = self.value_map[row, col]
                prev_conf = self.confidence_map[row, col]
                new_value = semantic_similarity_score
                new_conf = confidence[row, col]

                if (prev_conf + new_conf) == 0:
                    continue

                fused_value = (prev_value * prev_conf + new_value * new_conf) / (prev_conf + new_conf)
                fused_conf = (prev_conf**2 + new_conf**2) / (prev_conf + new_conf)

                self.value_map[row, col] = fused_value
                self.confidence_map[row, col] = fused_conf

        self.publish()

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

    def publish(self):
        self.publish_value_map_inferno()
        self.publish_value_map_raw()

    def publish_value_map_inferno(self):
        if self.map is None:
            return

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
                value = self.value_map[row, col]
                if value > 0:
                    x = origin.position.x + col * resolution
                    y = origin.position.y + row * resolution
                    z = 0.0

                    if value > self.max_semantic_score:
                        self.max_semantic_score = value

                    r, g, b = value_to_inferno_rgb(value, vmin=0.0, vmax=self.max_semantic_score)
                    rgb = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

                    points.append((x, y, z, rgb))

        cloud = pc2.create_cloud(header, fields, points)
        self.value_map_inferno_pub.publish(cloud)

    def publish_value_map_raw(self):
        if self.map is None:
            return

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
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        points = []
        for row in range(height):
            for col in range(width):
                value = self.value_map[row, col]
                if value > 0:
                    x = origin.position.x + col * resolution
                    y = origin.position.y + row * resolution
                    z = 0.0
                    intensity = value
                    points.append((x, y, z, intensity))

        cloud = pc2.create_cloud(header, fields, points)
        self.value_map_raw_pub.publish(cloud)

        
    def resize_maps_preserving_values(self, new_msg: OccupancyGrid):
        new_shape = (new_msg.info.height, new_msg.info.width)

        if self.value_map is None or self.confidence_map is None:
            # Initialisierung bei erster Map
            self.value_map = np.zeros(new_shape, dtype=np.float32)
            self.confidence_map = np.zeros(new_shape, dtype=np.float32)
            self.map = new_msg
            self.node.get_logger().info("Initialized value/confidence maps.")
            return

        old_shape = self.value_map.shape

        if old_shape == new_shape:
            # Keine Änderung nötig
            self.map = new_msg
            return

        # Logging zur Debugging-Zwecken
        self.node.get_logger().info(f"Resizing value map from {old_shape} to {new_shape}")

        new_value_map = np.zeros(new_shape, dtype=np.float32)
        new_confidence_map = np.zeros(new_shape, dtype=np.float32)

        min_rows = min(old_shape[0], new_shape[0])
        min_cols = min(old_shape[1], new_shape[1])

        new_value_map[:min_rows, :min_cols] = self.value_map[:min_rows, :min_cols]
        new_confidence_map[:min_rows, :min_cols] = self.confidence_map[:min_rows, :min_cols]

        self.value_map = new_value_map
        self.confidence_map = new_confidence_map
        self.map = new_msg

    def get_horizontal_fov(self) -> float:
        if self.fx is None or self.width is None:
            self.node.get_logger().warn("Camera intrinsics not received yet.")
            return 45.0  # fallback
        return 2 * math.atan2(self.width / 2, self.fx) * 180 / math.pi