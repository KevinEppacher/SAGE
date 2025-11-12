#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from multimodal_query_msgs.msg import SemanticPrompt
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
import tf2_ros, tf2_geometry_msgs
import json, os, time
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# ANSI color codes for clarity in console
GREEN = "\033[92m"
RESET = "\033[0m"

class SemanticRelabelerNode(Node):
    """Interactive relabeling node for semantic point clouds."""

    def __init__(self):
        super().__init__("semantic_relabeler")

        # ---------------- Parameters ----------------
        self.declare_parameter("ply_path",
            "/app/src/sage_evaluator/sage_datasets/matterport_isaac/"
            "00800-TEEsavR23oF/annotations/v1.7/semantic_corrected_20251111_200612.ply")
        self.declare_parameter("json_path",
            "/app/src/sage_evaluator/sage_datasets/matterport_isaac/"
            "00800-TEEsavR23oF/annotations/v1.7/semantic_20251111_104339.json")
        self.declare_parameter("frame_id", "robot_original_pose_at_scan")
        self.declare_parameter("cluster_eps", 0.20)
        self.declare_parameter("min_samples", 30)

        self.ply_path = self.get_parameter("ply_path").value
        self.json_path = self.get_parameter("json_path").value
        self.frame_id = self.get_parameter("frame_id").value
        self.eps = float(self.get_parameter("cluster_eps").value)
        self.min_samples = int(self.get_parameter("min_samples").value)

        # ---------------- TF setup ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- Load semantic map ----------------
        self.points, self.colors, self.class_ids = self.load_ply(self.ply_path)
        self.class_map = self.load_json(self.json_path)
        self.inv_class_map = {v.lower(): int(k) for k, v in self.class_map.items()}

        self.get_logger().info(f"Loaded {len(self.points)} points from {self.ply_path}")

        # -------- Cluster within each class --------
        self.cluster_labels = self.compute_clusters_per_class()
        n_clusters = len(np.unique(self.cluster_labels[self.cluster_labels >= 0]))
        self.get_logger().info(f"Clustered per class → {n_clusters} clusters.")
        self.active_cluster = None

        # -------- Compute and print color table --------
        self.class_colors = self.compute_class_colors()
        self.print_class_colors()

        # ---------------- ROS interfaces ----------------
        self.clicked_sub = self.create_subscription(PointStamped, "/clicked_point", self.handle_click, 10)
        self.prompt_sub = self.create_subscription(SemanticPrompt, "/user_prompt", self.handle_prompt, 10)
        self.pc_pub = self.create_publisher(PointCloud2, "semantic_cloud", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "semantic_labels", 1)

        # ---------------- Continuous publishing ----------------
        self.publish_cloud()
        self.timer = self.create_timer(0.5, self.publish_cloud)  # publish every half second

        self.get_logger().info(
            "Semantic cloud ready on topic: /semantic_cloud\n"
            "Click in RViz (Publish Point tool) and send /user_prompt to rename clusters."
        )

    # -----------------------------------------------------
    # Load utilities
    # -----------------------------------------------------
    def load_ply(self, path):
        pcd = o3d.t.io.read_point_cloud(path)
        key = "positions" if "positions" in pcd.point else "points"
        pts = pcd.point[key].numpy()
        cols = pcd.point["colors"].numpy()
        cls = pcd.point["class_id"].numpy().flatten()
        return pts, cols, cls

    def load_json(self, path):
        with open(path, "r") as f:
            return json.load(f)

    def save_json(self):
        with open(self.json_path, "w") as f:
            json.dump(self.class_map, f, indent=2)

    # -----------------------------------------------------
    # Compute per-class color map
    # -----------------------------------------------------
    def compute_class_colors(self):
        """Compute mean RGB color per class from loaded point cloud."""
        class_colors = {}
        for cid in np.unique(self.class_ids):
            mask = self.class_ids == cid
            if np.any(mask):
                mean_color = np.mean(self.colors[mask], axis=0)
                class_colors[int(cid)] = np.clip(mean_color, 0.0, 1.0)
        return class_colors

    def print_class_colors(self):
        """Pretty-print the class → color mapping."""
        self.get_logger().info("Detected class colors:")
        for cid, color in self.class_colors.items():
            cname = self.class_map.get(str(cid), f"class_{cid}")
            r, g, b = (color * 255).astype(int)
            self.get_logger().info(f"  {cname:20s} (id {cid:3d}) → RGB({r:3d}, {g:3d}, {b:3d})")

    # -----------------------------------------------------
    # Cluster within each class
    # -----------------------------------------------------
    def compute_clusters_per_class(self):
        """Run DBSCAN separately for each class to keep semantic separation."""
        cluster_labels = np.full(len(self.points), -1, dtype=int)
        next_cluster = 0
        for cid in np.unique(self.class_ids):
            mask = self.class_ids == cid
            if np.count_nonzero(mask) < self.min_samples:
                continue
            pts = self.points[mask]
            labels = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit_predict(pts)
            valid = labels != -1
            cluster_labels[mask] = np.where(valid, labels + next_cluster, -1)
            next_cluster += labels.max() + 1
        return cluster_labels

    # -----------------------------------------------------
    # Click handling
    # -----------------------------------------------------
    def handle_click(self, msg: PointStamped):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, msg.header.frame_id,
                rclpy.time.Time())
            p = tf2_geometry_msgs.do_transform_point(msg, tf)
            click = np.array([[p.point.x, p.point.y, p.point.z]])
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # find nearest cluster centroid
        dist = np.linalg.norm(self.points - click, axis=1)
        nearest_idx = np.argmin(dist)
        cluster_id = self.cluster_labels[nearest_idx]
        if cluster_id == -1:
            self.get_logger().warn("Clicked on noise/outlier region.")
            return

        mask = self.cluster_labels == cluster_id
        current_id = int(np.bincount(self.class_ids[mask]).argmax())
        current_label = self.class_map.get(str(current_id), str(current_id))
        self.active_cluster = cluster_id
        self.publish_markers()

        self.get_logger().info(
            f"\nClicked cluster {cluster_id} "
            f"({np.sum(mask)} pts) → current class '{GREEN}{current_label}{RESET}'.\n"
            "Awaiting new label via /user_prompt (SemanticPrompt.text_query).\n"
            "Example cmd:\n"
            "  ros2 topic pub --once /user_prompt "
            "multimodal_query_msgs/msg/SemanticPrompt \"{text_query: bed}\"\n"
        )

    # -----------------------------------------------------
    # Prompt handling (rename cluster)
    # -----------------------------------------------------
    def handle_prompt(self, msg: SemanticPrompt):
        if self.active_cluster is None:
            self.get_logger().warn("No active cluster selected yet.")
            return

        new_name = msg.text_query.strip().lower()
        if not new_name:
            self.get_logger().warn("Empty label ignored.")
            return

        if new_name not in self.inv_class_map:
            new_id = max(map(int, self.class_map.keys())) + 1
            self.class_map[str(new_id)] = new_name
            self.inv_class_map[new_name] = new_id
            rng = np.random.default_rng(seed=new_id)
            new_color = rng.random(3)
            self.class_colors[new_id] = new_color
            self.get_logger().info(f"Added new class '{new_name}' (id {new_id}) with color {new_color.round(2)}")
        else:
            new_id = self.inv_class_map[new_name]

        mask = self.cluster_labels == self.active_cluster
        target_color = self.get_class_color(new_id)

        self.class_ids[mask] = new_id
        self.colors[mask] = target_color

        self.save_json()
        self.save_ply()

        # Reload and re-publish
        time.sleep(0.1)
        self.points, self.colors, self.class_ids = self.load_ply(self.ply_path)
        self.cluster_labels = self.compute_clusters_per_class()
        self.publish_cloud()
        self.publish_markers()

        self.get_logger().info(
            f"Relabeled cluster {self.active_cluster} → '{new_name}' "
            f"({np.sum(mask)} pts), color={target_color.round(2)}"
        )

        self.active_cluster = None

    # -----------------------------------------------------
    # Color lookup
    # -----------------------------------------------------
    def get_class_color(self, class_id: int) -> np.ndarray:
        """Lookup color from precomputed table (or assign if new)."""
        if class_id in self.class_colors:
            return self.class_colors[class_id]
        rng = np.random.default_rng(seed=int(class_id))
        color = rng.random(3)
        self.class_colors[class_id] = color
        return color

    # -----------------------------------------------------
    # Cloud + marker publishing
    # -----------------------------------------------------
    def publish_cloud(self):
        if len(self.points) == 0:
            self.get_logger().warn("No points to publish.")
            return

        rgb_uint32 = (
            (np.clip(self.colors[:, 0], 0, 1) * 255).astype(np.uint32) << 16 |
            (np.clip(self.colors[:, 1], 0, 1) * 255).astype(np.uint32) << 8 |
            (np.clip(self.colors[:, 2], 0, 1) * 255).astype(np.uint32)
        )

        cloud_arr = np.zeros(
            len(self.points),
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.uint32)]
        )
        cloud_arr["x"] = self.points[:, 0]
        cloud_arr["y"] = self.points[:, 1]
        cloud_arr["z"] = self.points[:, 2]
        cloud_arr["rgb"] = rgb_uint32

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]

        msg = pc2.create_cloud(header, fields, cloud_arr)
        msg.width = len(self.points)
        msg.height = 1
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width

        self.pc_pub.publish(msg)
        self.publish_markers()

    def save_ply(self):
        pcd_new = o3d.t.geometry.PointCloud()
        key = "positions"
        pcd_new.point[key] = o3d.core.Tensor(self.points, o3d.core.Dtype.Float32)
        pcd_new.point["colors"] = o3d.core.Tensor(self.colors, o3d.core.Dtype.Float32)
        pcd_new.point["class_id"] = o3d.core.Tensor(self.class_ids.reshape(-1, 1), o3d.core.Dtype.Int32)
        o3d.t.io.write_point_cloud(self.ply_path, pcd_new)
        self.get_logger().info(f"Overwritten {self.ply_path} with updated labels.")

    def publish_markers(self):
        if self.cluster_labels is None or len(self.cluster_labels) == 0:
            return

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        marker_id = 0

        for cluster_id in np.unique(self.cluster_labels):
            if cluster_id == -1:
                continue

            mask = self.cluster_labels == cluster_id
            cluster_points = self.points[mask]
            if len(cluster_points) == 0:
                continue

            centroid = np.mean(cluster_points, axis=0)
            class_id = int(np.bincount(self.class_ids[mask]).argmax())
            class_name = self.class_map.get(str(class_id), f"class_{class_id}")
            mean_color = self.get_class_color(class_id)

            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = now
            sphere.ns = "semantic_clusters"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(centroid[0])
            sphere.pose.position.y = float(centroid[1])
            sphere.pose.position.z = float(centroid[2])
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2
            sphere.color = ColorRGBA(r=float(mean_color[0]),
                                     g=float(mean_color[1]),
                                     b=float(mean_color[2]),
                                     a=0.6)
            marker_array.markers.append(sphere)
            marker_id += 1

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = now
            text.ns = "semantic_text"
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(centroid[0])
            text.pose.position.y = float(centroid[1])
            text.pose.position.z = float(centroid[2]) + 0.25
            text.scale.z = 0.25
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = class_name
            marker_array.markers.append(text)
            marker_id += 1

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {marker_id // 2} cluster markers.")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticRelabelerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
