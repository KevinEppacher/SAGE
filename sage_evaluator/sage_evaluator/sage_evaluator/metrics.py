#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
import tf2_ros
import numpy as np
from math import sqrt


class Metrics:
    def __init__(self, node: Node):
        self.node = node

        # TF buffer for robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        # Action client for global planner (Nav2)
        # Use namespaced planner if you prefer: "/evaluator/compute_path_to_pose"
        # self.path_client = ActionClient(node, ComputePathToPose, "/evaluator/compute_path_to_pose")

        # Parameters
        self.success_radius = 0.5  # m
        self.frame_id = "map"
        self.robot_frame = "base_link"

        # Internal state
        self.centroids = []   # list of (x, y)
        self.start_pose = None
        self.paths = []       # list of dicts: {success, geo_dist, euclid}

        # Subscriptions and publishers
        node.create_subscription(
            PoseArray,
            "/evaluator/semantic_centroid_targets",
            self._centroids_cb,
            10
        )
        self.path_pub = node.create_publisher(Path, "/evaluator/shortest_path", 10)

        # Periodic metric updates
        node.create_timer(2.0, self._update)

        self.node.get_logger().info("Metrics evaluator initialized using /compute_path_to_pose.")

    # ---------------------------------------------------------------
    def _centroids_cb(self, msg: PoseArray):
        self.centroids = [(p.position.x, p.position.y) for p in msg.poses]
        # if self.centroids:
        #     self.node.get_logger().info(
        #         f"Received {len(self.centroids)} centroid targets for evaluation."
        #     )
        # else:
        #     self.node.get_logger().warn("Received empty centroid PoseArray for evaluation.")

    # ---------------------------------------------------------------
    def _get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.robot_frame, rclpy.time.Time())
            t = tf.transform.translation
            return np.array([t.x, t.y])
        except Exception:
            return None

    # ---------------------------------------------------------------
    def _update(self):
        pose = self._get_robot_pose()
        if pose is None or not self.centroids:
            return

        # # Initialize start pose once
        # if self.start_pose is None:
        #     self.start_pose = pose
        #     self.node.get_logger().info(f"Start pose set at {self.start_pose}")
        #     return

        # best_goal = None
        # best_length = float("inf")
        # best_path = None

        # # Compute geodesic distance to every centroid
        # for c in self.centroids:
        #     geo_length, path_msg = self._compute_path(pose, c)
        #     if geo_length is not None and geo_length < best_length:
        #         best_length = geo_length
        #         best_goal = c
        #         best_path = path_msg

        # if best_goal is None:
        #     self.node.get_logger().warn("No valid geodesic path to any centroid.")
        #     return

        # # Publish best path
        # if best_path:
        #     self.path_pub.publish(best_path)

        # # Euclidean distance to that goal
        # euclid_dist = np.linalg.norm(np.array(best_goal) - pose)
        # success = euclid_dist < self.success_radius

        # self.node.get_logger().info(
        #     f"Nearest goal (geodesic) at {best_goal} | "
        #     f"geo={best_length:.2f} m | euclid={euclid_dist:.2f} m → "
        #     f"{'✅ success' if success else '⏳ pending'}"
        # )

        # # Store metrics sample
        # self.paths.append(
        #     {
        #         "success": success,
        #         "geo_dist": best_length,
        #         "euclid": euclid_dist,
        #     }
        # )

    # ---------------------------------------------------------------
    def _compute_path(self, start, goal):
        """Use Nav2 ComputePathToPose action to get path and compute its length."""
        if not self.path_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn("Planner action server not available.")
            return None, None

        start_pose = PoseStamped()
        start_pose.header.frame_id = self.frame_id
        start_pose.pose.position.x = float(start[0])
        start_pose.pose.position.y = float(start[1])
        start_pose.pose.orientation.w = 1.0

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.pose.position.x = float(goal[0])
        goal_pose.pose.position.y = float(goal[1])
        goal_pose.pose.orientation.w = 1.0

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose
        goal_msg.goal = goal_pose
        goal_msg.use_start = True

        try:
            send_future = self.path_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=3.0)
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.node.get_logger().warn("Planner goal rejected.")
                return None, None

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)
            result = result_future.result()
            if not result or not result.result.path.poses:
                self.node.get_logger().warn("Planner returned empty path.")
                return None, None

            path = result.result.path
            length = self._path_length(path)
            return length, path

        except Exception as e:
            self.node.get_logger().error(f"Path planning failed: {e}")
            return None, None

    # ---------------------------------------------------------------
    def _path_length(self, path: Path):
        length = 0.0
        poses = path.poses
        for i in range(1, len(poses)):
            p1 = poses[i - 1].pose.position
            p2 = poses[i].pose.position
            length += sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)
        return length

    # ---------------------------------------------------------------
    def summarize(self):
        """Compute SR and SPL over all evaluated samples."""
        if not self.paths:
            self.node.get_logger().warn("No path samples recorded, SR/SPL = 0.")
            return 0.0, 0.0

        sr = sum(1 for p in self.paths if p["success"]) / len(self.paths)

        spl_values = [
            p["geo_dist"] / max(p["geo_dist"], p["euclid"])
            for p in self.paths
            if p["success"]
        ]
        spl = float(np.mean(spl_values)) if spl_values else 0.0

        self.node.get_logger().info(f"📈 SR: {sr:.3f}, SPL: {spl:.3f}")
        return sr, spl
