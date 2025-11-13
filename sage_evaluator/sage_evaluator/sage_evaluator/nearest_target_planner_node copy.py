#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
import math
import numpy as np
import time


class NearestPathPlanner(Node):
    """Selects the nearest valid centroid and republishes its shortest Nav2 path."""

    def __init__(self):
        super().__init__("nearest_path_planner")

        # Nav2 action client
        self.client = ActionClient(self, ComputePathToPose, "/evaluator/compute_path_to_pose")

        # ROS interfaces
        self.create_subscription(PoseArray, "/evaluator/navigable_targets", self._centroids_cb, 10)
        self.path_pub = self.create_publisher(Path, "/evaluator/nearest_path", 10)

        # Internal state
        self.centroids = []
        self.ready = False
        self.shortest_path = None
        self.shortest_length = float("inf")

        # Frames
        self.map_frame = "robot_original_pose_at_scan"
        self.robot_frame = "base_link"

        # Info
        self.get_logger().info("⏳ Waiting for planner server /evaluator/compute_path_to_pose...")
        self.client.wait_for_server()
        self.get_logger().info("✅ Planner action server available.")

        self.timer = self.create_timer(3.5, self._evaluate_targets)
        self.get_logger().info("📡 NearestPathPlanner initialized. Waiting for centroid targets...")

    # ------------------------------------------------------------------
    def _centroids_cb(self, msg: PoseArray):
        """Called when semantic centroid targets are published."""
        if not msg.poses:
            return
        self.centroids = msg.poses
        self.ready = True
        self.get_logger().info(f"Received {len(self.centroids)} centroid targets.")

    # ------------------------------------------------------------------
    def _make_pose(self, x, y, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    # ------------------------------------------------------------------
    def _evaluate_targets(self):
        """Compute shortest valid path among all current centroid targets."""
        if not self.ready or not self.centroids:
            return

        self.shortest_length = float("inf")
        self.shortest_path = None

        for i, target in enumerate(self.centroids):
            start = self._make_pose(0.0, 0.0, self.robot_frame)
            goal = self._make_pose(target.position.x, target.position.y, self.map_frame)

            goal_msg = ComputePathToPose.Goal()
            goal_msg.start = start
            goal_msg.goal = goal
            goal_msg.use_start = True

            send_future = self.client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=2.0)
            goal_handle = send_future.result()

            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn(f"Planner goal to target {i+1} rejected.")
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=3.0)
            result = result_future.result()

            print("Waiting for result for target", i+1)

            if not result or not result.result.path.poses:
                continue

            path = result.result.path
            length = self._path_length(path)
            print("-----")
            print(f"Path to target {i+1}/{len(self.centroids)} length: {length:.2f} m")

            if length < self.shortest_length:
                self.shortest_length = length
                self.shortest_path = path

        print("Shortest path:", self.shortest_path)
        print("Shortest length:", self.shortest_length)

        # Publish best path if found
        if self.shortest_path:
            self.path_pub.publish(self.shortest_path)
            self.get_logger().info(f"📍 Published nearest path ({self.shortest_length:.2f} m).")
        else:
            self.get_logger().warn("No valid paths found.")

    # ------------------------------------------------------------------
    def _path_length(self, path: Path):
        """Compute total Euclidean length of a nav_msgs/Path."""
        return sum(
            math.sqrt(
                (path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x) ** 2
                + (path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y) ** 2
            )
            for i in range(1, len(path.poses))
        )


def main(args=None):
    rclpy.init(args=args)
    node = NearestPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
