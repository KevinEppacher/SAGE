import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2  # Stelle sicher, dass `ros-humble-sensor-msgs-py` installiert ist

class ColoredPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('colored_pointcloud_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/colored_pointcloud', 10)
        self.timer = self.create_timer(1.0, self.publish_pointcloud)

    def publish_pointcloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        points = []
        size = 5
        resolution = 0.2
        for x in range(-size, size):
            for y in range(-size, size):
                for z in range(0, size):
                    r = int(255 * (x + size) / (2 * size))
                    g = int(255 * (y + size) / (2 * size))
                    b = int(255 * z / size)
                    rgb = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
                    points.append((x * resolution, y * resolution, z * resolution, rgb))

        cloud_msg = pc2.create_cloud(header, fields, points)
        self.publisher.publish(cloud_msg)
        self.get_logger().info("Published colored point cloud")

def main(args=None):
    rclpy.init(args=args)
    node = ColoredPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
