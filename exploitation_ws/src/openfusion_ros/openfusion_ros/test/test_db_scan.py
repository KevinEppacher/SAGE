import numpy as np
from sklearn.cluster import DBSCAN
import open3d as o3d

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


class PointCloudClusteringNode(Node):
    def __init__(self):
        super().__init__('pointcloud_clustering_node')

        self.sub = self.create_subscription(
            PointCloud2,
            '/openfusion/semantic_pointcloud',
            self.pointcloud_callback,
            10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/clusters_bboxes', 10)
        self.frame_id = 'map'  # ggf. anpassen

    def pointcloud_callback(self, msg: PointCloud2):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            points.append([x, y, z])
        points = np.array(points)

        if points.shape[0] < 10:
            self.get_logger().warn("Zu wenige Punkte in der PointCloud.")
            return

        # Clustering mit DBSCAN
        db = DBSCAN(eps=0.2, min_samples=10)
        labels = db.fit_predict(points)

        marker_array = MarkerArray()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        for label in np.unique(labels):
            if label == -1:
                continue  # noise
            cluster = points[labels == label]
            if len(cluster) < 3:
                continue

            # Berechne Bounding Box mit Open3D
            cluster_pcd = o3d.geometry.PointCloud()
            cluster_pcd.points = o3d.utility.Vector3dVector(cluster)
            bbox = cluster_pcd.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extent = bbox.get_extent()

            marker = Marker()
            marker.header = header
            marker.ns = "bbox_cluster"
            marker.id = int(label)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.scale.x = float(extent[0])
            marker.scale.y = float(extent[1])
            marker.scale.z = float(extent[2])
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.4
            marker.lifetime.sec = 1

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"DBSCAN: {len(marker_array.markers)} Cluster gefunden.")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudClusteringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
