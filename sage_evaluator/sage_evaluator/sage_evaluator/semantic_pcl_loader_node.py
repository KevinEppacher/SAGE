#!/usr/bin/env python3
import os
import json
import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import sensor_msgs_py.point_cloud2 as pc2

from multimodal_query_msgs.msg import SemanticPrompt

class SemanticPCLLoaderNode(Node):
    def __init__(self):
        super().__init__("semantic_pcl_loader")

        # ---------------- Parameters ----------------
        self.declare_parameter("ply_path", "/app/src/sage_evaluator/datasets/matterport_isaac/00809-Qpor2mEya8F/annotations/v1.0/semantic_20251106_182804.ply")
        self.declare_parameter("json_path", "/app/src/sage_evaluator/datasets/matterport_isaac/00809-Qpor2mEya8F/annotations/v1.0/semantic_20251106_182804.json")
        self.declare_parameter("dbscan_eps", 0.4)
        self.declare_parameter("dbscan_min_samples", 30)
        self.declare_parameter("frame_id", "robot_original_pose_at_scan")
        self.declare_parameter("user_prompt_topic", "/user_prompt")
        self.declare_parameter("publish_rate", 0.5)  # Hz, 0.5 = every 2 seconds

        self.ply_path = self.get_parameter("ply_path").value
        self.json_path = self.get_parameter("json_path").value
        self.frame_id = self.get_parameter("frame_id").value
        self.eps = float(self.get_parameter("dbscan_eps").value)
        self.min_samples = int(self.get_parameter("dbscan_min_samples").value)
        user_prompt_topic = self.get_parameter("user_prompt_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        # ---------------- Publishers ----------------
        self.pc_pub = self.create_publisher(PointCloud2, "filtered_semantic_pc", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "semantic_centroids", 1)

        # ---------------- Load data ----------------
        self.get_logger().info(f"Loading semantic map from {self.ply_path}")
        self.points, self.colors, self.class_ids = self._load_ply(self.ply_path)
        self.class_map = self._load_json(self.json_path)
        self.inv_class_map = {v.lower(): int(k) for k, v in self.class_map.items()}

        if len(self.points) == 0:
            self.get_logger().error("Empty semantic map. Aborting.")
            return

        # Store original full data
        self.points_full = np.copy(self.points)
        self.colors_full = np.copy(self.colors)
        self.current_points = np.copy(self.points)
        self.current_colors = np.copy(self.colors)

        self.prompt_received = False

        # ---------------- Timer for continuous publishing ----------------
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # ---------------- Subscribe for prompts ----------------
        self.prompt_sub = self.create_subscription(
            SemanticPrompt, user_prompt_topic, self.handle_prompt, 10
        )
        self.get_logger().info("Ready for user prompts to filter semantic map.")

    # ---------------- Timer Callback ----------------
    def timer_callback(self):
        """Continuously publish current pointcloud (filtered or full)."""
        if not self.prompt_received:
            # Before any prompt, publish the full map
            self._publish_pc(self.points_full, self.colors_full)
        else:
            # After first prompt, publish the filtered map
            self._publish_pc(self.current_points, self.current_colors)

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

    # ---------------- Prompt Handler ----------------
    def handle_prompt(self, msg):
        """Callback for incoming SemanticPrompt messages."""
        class_name = msg.text_query.strip().lower()
        self.get_logger().info(f"Received prompt: '{class_name}'")
        self.prompt_received = True

        if class_name in ("all", "reset", "everything"):
            self.get_logger().info("Resetting to full semantic map...")
            self.current_points = np.copy(self.points_full)
            self.current_colors = np.copy(self.colors_full)
            self.marker_pub.publish(MarkerArray())
            return

        if class_name not in self.inv_class_map:
            self.get_logger().warn(f"Class '{class_name}' not found in semantic map.")
            return

        pts, cols = self._filter_class(class_name)
        centroids = self._cluster(pts)
        self.current_points = pts
        self.current_colors = cols
        self._publish_markers(centroids)
        self.get_logger().info(
            f"Updated current map: {len(pts)} '{class_name}' points, {len(centroids)} centroids."
        )

    # ---------------- Processing ----------------
    def _filter_class(self, class_name):
        class_id = self.inv_class_map.get(class_name)
        if class_id is None:
            return np.empty((0, 3)), np.empty((0, 3))
        mask = self.class_ids == class_id
        pts, cols = self.points[mask], self.colors[mask]
        if len(pts) == 0:
            self.get_logger().warn(f"No points found for '{class_name}'")
        else:
            cols[:] = np.array([0.0, 1.0, 0.0])  # highlight green
        return pts, cols

    def _cluster(self, pts):
        if len(pts) == 0:
            return np.empty((0, 3))
        labels = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(pts).labels_
        centroids = [pts[labels == lbl].mean(axis=0) for lbl in np.unique(labels) if lbl != -1]
        return np.array(centroids)

    # ---------------- ROS Publishing ----------------
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


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPCLLoaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
