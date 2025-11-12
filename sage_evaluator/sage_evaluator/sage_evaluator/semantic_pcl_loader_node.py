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
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
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
        self.declare_parameter("user_prompt_topic", "/evaluator/prompt")
        self.declare_parameter("publish_rate", 0.5)

        # ---------------- Parameter values ----------------
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

        # NEW: Centroid PoseArray publisher for evaluator node
        self.centroid_pub = self.create_publisher(PoseArray, "semantic_centroid_targets", 1)

        # ---------------- Load data ----------------
        self.get_logger().info(f"Loading semantic map from {self.ply_path}")
        self.points, self.colors, self.class_ids = self._load_ply(self.ply_path)
        self.class_map = self._load_json(self.json_path)
        self.inv_class_map = {v.lower(): int(k) for k, v in self.class_map.items()}

        if len(self.points) == 0:
            self.get_logger().error("Empty semantic map. Aborting.")
            return

        self.points_full = np.copy(self.points)
        self.colors_full = np.copy(self.colors)
        self.current_points = np.copy(self.points)
        self.current_colors = np.copy(self.colors)
        self.current_centroids = np.empty((0, 3))  # <-- cache for PoseArray publishing

        self.prompt_received = False

        # ---------------- Timers ----------------
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.prompt_sub = self.create_subscription(SemanticPrompt, user_prompt_topic, self.handle_prompt, 10)

        self.get_logger().info("Ready for user prompts to filter semantic map.")
        self._publish_all_class_markers()

    # ---------------- Timer Callback ----------------
    def timer_callback(self):
        """Continuously publish current pointcloud and centroid targets."""
        if not self.prompt_received:
            self._publish_pc(self.points_full, self.colors_full)
        else:
            self._publish_pc(self.current_points, self.current_colors)

        # Always publish current centroids as PoseArray for evaluator
        if len(self.current_centroids) > 0:
            self._publish_centroid_targets(self.current_centroids)

    # ---------------- Publish Centroid Targets ----------------
    def _publish_centroid_targets(self, centroids):
        """Publish current cluster centroids as PoseArray."""
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Identity quaternion since we only care about positions
        q_identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        for c in centroids:
            pose = Pose()
            pose.position = Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))
            pose.orientation = q_identity
            msg.poses.append(pose)

        self.centroid_pub.publish(msg)

    # ---------------- Initial Markers for All Classes (Clustered) ----------------
    def _publish_all_class_markers(self):
        """Publish one marker per cluster per semantic class, with same color as pointcloud."""
        if self.class_ids is None or len(self.class_ids) == 0:
            self.get_logger().warn("No class IDs available for marker publishing.")
            return

        marker_array = MarkerArray()
        marker_id = 0
        unique_classes = np.unique(self.class_ids)

        for class_id in unique_classes:
            pts = self.points[self.class_ids == class_id]
            cols = self.colors[self.class_ids == class_id]
            if len(pts) == 0:
                continue

            class_name = self.class_map.get(str(int(class_id)), f"class_{class_id}")
            centroids = self._cluster(pts)

            if len(centroids) == 0:
                # fallback if clustering fails
                centroids = [np.mean(pts, axis=0)]

            for i, centroid in enumerate(centroids):
                # color based on mean of cluster
                cluster_mask = np.linalg.norm(pts - centroid, axis=1) < self.eps * 2.0
                cluster_colors = cols[cluster_mask] if np.any(cluster_mask) else cols
                mean_color = np.clip(np.mean(cluster_colors, axis=0), 0.0, 1.0)

                # Sphere marker
                sphere = Marker()
                sphere.header.frame_id = self.frame_id
                sphere.header.stamp = self.get_clock().now().to_msg()
                sphere.ns = class_name
                sphere.id = marker_id
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position.x = float(centroid[0])
                sphere.pose.position.y = float(centroid[1])
                sphere.pose.position.z = float(centroid[2])
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25
                sphere.color = ColorRGBA(
                    r=float(mean_color[0]),
                    g=float(mean_color[1]),
                    b=float(mean_color[2]),
                    a=0.9
                )
                marker_array.markers.append(sphere)
                marker_id += 1

                # Label marker
                text = Marker()
                text.header.frame_id = self.frame_id
                text.header.stamp = sphere.header.stamp
                text.ns = "semantic_labels"
                text.id = marker_id
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = float(centroid[0])
                text.pose.position.y = float(centroid[1])
                text.pose.position.z = float(centroid[2]) + 0.3
                text.scale.z = 0.35
                text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                text.text = class_name
                marker_array.markers.append(text)
                marker_id += 1

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {marker_id // 2} clustered semantic markers across {len(unique_classes)} classes.")

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
        raw_query = msg.text_query.strip().lower()
        self.prompt_received = True

        requested_classes = [c.strip() for c in raw_query.replace(",", " ").split() if c.strip()]
        self.get_logger().info(f"Received prompt for classes: {requested_classes}")

        if any(c in ("all", "reset", "everything") for c in requested_classes):
            self.get_logger().info("Resetting to full semantic map...")
            self.current_points = np.copy(self.points_full)
            self.current_colors = np.copy(self.colors_full)
            self.current_centroids = np.empty((0, 3))
            self.marker_pub.publish(MarkerArray())
            return

        valid_classes = []
        all_pts, all_cols, all_centroids = [], [], []

        for class_name in requested_classes:
            if class_name not in self.inv_class_map:
                self.get_logger().warn(f"Class '{class_name}' not found in semantic map.")
                continue

            pts, cols = self._filter_class(class_name)
            if len(pts) > 0:
                all_pts.append(pts)
                all_cols.append(cols)
                valid_classes.append(class_name)
                centroids = self._cluster(pts)
                if len(centroids) > 0:
                    all_centroids.extend(centroids)

        if not all_pts:
            self.get_logger().warn("No valid classes found in prompt.")
            return

        self.current_points = np.concatenate(all_pts, axis=0)
        self.current_colors = np.concatenate(all_cols, axis=0)
        self.current_centroids = np.array(all_centroids)

        # Delete all old markers
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[delete_all]))

        # Publish cluster markers and centroids
        self._publish_markers(self.current_centroids)
        self._publish_centroid_targets(self.current_centroids)

        self.get_logger().info(
            f"Updated {len(valid_classes)} classes → {len(self.current_centroids)} centroids published as targets."
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
