#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import json
from sklearn.cluster import DBSCAN
import os


class SemanticPointCloudLoader(Node):
    def __init__(self):
        super().__init__("semantic_pcl_loader")

        # Parameters
        self.declare_parameter("ply_path", "/app/data/semantic_maps/semantic_20251105_173245.ply")
        self.declare_parameter("json_path", "/app/data/semantic_maps/semantic_20251105_173245.json")
        self.declare_parameter("target_class", "sofa")
        self.declare_parameter("dbscan_eps", 0.4)
        self.declare_parameter("dbscan_min_samples", 30)
        self.declare_parameter("frame_id", "map")

        self.ply_path = self.get_parameter("ply_path").value
        self.json_path = self.get_parameter("json_path").value
        self.target_class = self.get_parameter("target_class").value
        self.frame_id = self.get_parameter("frame_id").value
        self.eps = float(self.get_parameter("dbscan_eps").value)
        self.min_samples = int(self.get_parameter("dbscan_min_samples").value)

        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, "filtered_semantic_pc", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "semantic_centroids", 1)

        # Load files
        self.get_logger().info(f"Loading semantic map from {self.ply_path}")
        self.points, self.colors, self.class_ids = self._load_ply(self.ply_path)
        self.class_map = self._load_json(self.json_path)
        self.inv_class_map = {v: int(k) for k, v in self.class_map.items()}

        # Periodic publish
        self.timer = self.create_timer(2.0, self.timer_callback)

    # ---------------- File I/O ----------------
    def _load_ply(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"PLY file not found: {path}")
            return np.empty((0, 3)), np.empty((0, 3)), np.empty((0,))
        pcd = o3d.t.io.read_point_cloud(path)

        pts = pcd.point["positions"].numpy()
        cols = pcd.point["colors"].numpy()
        cls = pcd.point["class_id"].numpy().flatten()

        return pts, cols, cls

    def _load_json(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"JSON file not found: {path}")
            return {}
        with open(path, "r") as f:
            return json.load(f)

    # ---------------- Processing ----------------
    def _filter_class(self, class_name):
        class_id = self.inv_class_map.get(class_name)
        if class_id is None:
            self.get_logger().warn(f"Class '{class_name}' not found.")
            return np.empty((0, 3)), np.empty((0, 3))
        mask = self.class_ids == class_id
        pts, cols = self.points[mask], self.colors[mask]
        # paint everything green for visualization
        cols[:] = np.array([0.0, 1.0, 0.0])
        return pts, cols

    def _cluster(self, pts):
        if len(pts) == 0:
            return np.empty((0, 3))
        labels = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(pts).labels_
        centroids = []
        for lbl in np.unique(labels):
            if lbl == -1:
                continue
            cluster_pts = pts[labels == lbl]
            centroids.append(cluster_pts.mean(axis=0))
        return np.array(centroids)

    # ---------------- ROS Publish ----------------
    def _publish_pc(self, pts, cols):
        if len(pts) == 0:
            return
        rgb_uint32 = (
            (np.clip(cols[:, 0], 0, 1) * 255).astype(np.uint32) << 16 |
            (np.clip(cols[:, 1], 0, 1) * 255).astype(np.uint32) << 8 |
            (np.clip(cols[:, 2], 0, 1) * 255).astype(np.uint32)
        )
        cloud = [(x, y, z, rgb) for (x, y, z), rgb in zip(pts, rgb_uint32)]
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        msg = pc2.create_cloud(header, fields, cloud)
        self.pc_pub.publish(msg)

    def _publish_markers(self, centroids):
        if len(centroids) == 0:
            return
        marker_array = MarkerArray()
        for i, c in enumerate(centroids):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "semantic_centroids"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(c[0])
            m.pose.position.y = float(c[1])
            m.pose.position.z = float(c[2])
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)

    # ---------------- Timer ----------------
    def timer_callback(self):
        pts, cols = self._filter_class(self.target_class)
        centroids = self._cluster(pts)
        self._publish_pc(pts, cols)
        self._publish_markers(centroids)
        self.get_logger().info(
            f"Published {len(pts)} '{self.target_class}' points "
            f"and {len(centroids)} clusters."
        )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPointCloudLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
