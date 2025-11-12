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

        # ---------------- ROS interfaces ----------------
        self.clicked_sub = self.create_subscription(PointStamped, "/clicked_point", self.handle_click, 10)
        self.prompt_sub = self.create_subscription(SemanticPrompt, "/user_prompt", self.handle_prompt, 10)
        self.pc_pub = self.create_publisher(PointCloud2, "semantic_cloud", 1)

        # ---------------- Continuous publishing ----------------
        self.publish_cloud()
        self.timer = self.create_timer(1.0, self.publish_cloud)  # publish every second

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
            self.get_logger().info(f"Added new class '{new_name}' with id {new_id}.")
        else:
            new_id = self.inv_class_map[new_name]

        mask = self.cluster_labels == self.active_cluster
        self.class_ids[mask] = new_id
        self.colors[mask] = [1.0, 0.0, 0.0]  # visual feedback

        self.save_json()
        self.save_ply()

        self.get_logger().info(
            f"Relabeled cluster {self.active_cluster} → '{new_name}' "
            f"({np.sum(mask)} pts)"
        )
        self.active_cluster = None

    # -----------------------------------------------------
    # Continuous publishing + save
    # -----------------------------------------------------
    def publish_cloud(self):
        if len(self.points) == 0:
            self.get_logger().warn("No points to publish.")
            return

        # Build structured numpy array
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
        self.get_logger().debug(f"Published {msg.width} points.")


    def save_ply(self):
        """Overwrite same PLY file instead of creating new versions."""
        pcd_new = o3d.t.geometry.PointCloud()
        key = "positions"
        pcd_new.point[key] = o3d.core.Tensor(self.points, o3d.core.Dtype.Float32)
        pcd_new.point["colors"] = o3d.core.Tensor(self.colors, o3d.core.Dtype.Float32)
        pcd_new.point["class_id"] = o3d.core.Tensor(self.class_ids.reshape(-1, 1), o3d.core.Dtype.Int32)
        o3d.t.io.write_point_cloud(self.ply_path, pcd_new)
        self.get_logger().info(f"Overwritten {self.ply_path} with updated labels.")


def main(args=None):
    rclpy.init(args=args)
    node = SemanticRelabelerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
