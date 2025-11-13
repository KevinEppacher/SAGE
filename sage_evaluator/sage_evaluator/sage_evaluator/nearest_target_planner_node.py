#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
import math


class NearestPathPlanner(Node):
    """Plans sequentially for all centroids, finds shortest path, republishes it."""

    def __init__(self):
        super().__init__("nearest_path_planner")

        self.client = ActionClient(self, ComputePathToPose, "/evaluator/compute_path_to_pose")

        self.create_subscription(PoseArray, "/evaluator/navigable_targets", self._centroid_cb, 10)
        self.path_pub = self.create_publisher(Path, "/evaluator/shortest_path", 10)

        # Internal state
        self.centroids = []
        self.index = 0
        self.planning = False

        # Shortest path storage
        self.shortest_path = None
        self.shortest_length = float("inf")

        self.get_logger().info("⏳ Waiting for planner server /evaluator/compute_path_to_pose...")
        self.client.wait_for_server()
        self.get_logger().info("🟢 Planner available. Waiting for centroid array...")

    # ------------------------------------------------------------------
    def _centroid_cb(self, msg):
        """Triggered when a centroid PoseArray arrives."""
        if not msg.poses:
            return

        self.centroids = msg.poses
        self.index = 0
        self.planning = False

        # Reset shortest result
        self.shortest_path = None
        self.shortest_length = float("inf")

        self.get_logger().info(f"🎯 Received {len(self.centroids)} centroids. Starting planning...")
        self._plan_next()

    # ------------------------------------------------------------------
    def _plan_next(self):
        """Plan for the next centroid in sequence."""
        if self.planning:
            return

        if self.index >= len(self.centroids):
            self._finalize_shortest_path()
            return

        target = self.centroids[self.index]
        x = target.position.x
        y = target.position.y

        self.get_logger().info(f"➡️ Planning {self.index+1}/{len(self.centroids)}: x={x:.2f}, y={y:.2f}")

        start = self._make_pose(0.0, 0.0, "base_link")
        goal = self._make_pose(x, y, "robot_original_pose_at_scan")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.use_start = True

        self.planning = True

        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------
    def _make_pose(self, x, y, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    # ------------------------------------------------------------------
    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn(f"❌ Planner rejected goal {self.index+1}. Skipping.")
            self.planning = False
            self.index += 1
            self._plan_next()
            return

        self.get_logger().info("🚀 Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    # ------------------------------------------------------------------
    def _result_callback(self, future):
        result = future.result().result
        path = result.path
        n = len(path.poses)

        if n == 0:
            self.get_logger().warn(f"⚠ Empty path for centroid {self.index+1}.")
        else:
            length = self._compute_path_length(path)
            self.get_logger().info(
                f"📦 Path {self.index+1}: {n} poses, length={length:.2f} m"
            )

            # Update shortest
            if length < self.shortest_length:
                self.shortest_length = length
                self.shortest_path = path

        # Continue sequence
        self.index += 1
        self.planning = False
        self._plan_next()

    # ------------------------------------------------------------------
    def _compute_path_length(self, path: Path):
        """Cartesian path length."""
        total = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            total += math.sqrt(
                (p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2
            )
        return total

    # ------------------------------------------------------------------
    def _finalize_shortest_path(self):
        """Publish the shortest path after evaluating all centroids."""
        if self.shortest_path is None:
            self.get_logger().warn("❌ No valid paths found for any centroid.")
            return

        self.get_logger().info(
            f"🏆 Shortest path = {self.shortest_length:.2f} m → publishing /evaluator/nearest_path"
        )

        self.path_pub.publish(self.shortest_path)


def main(args=None):
    rclpy.init(args=args)
    node = NearestPathPlanner()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
