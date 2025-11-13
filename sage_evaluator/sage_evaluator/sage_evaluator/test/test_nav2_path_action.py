#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
import math
import time


class Nav2PathTester(Node):
    def __init__(self):
        super().__init__("nav2_path_tester")

        # Action client for Nav2 global planner
        self.client = ActionClient(self, ComputePathToPose, "/evaluator/compute_path_to_pose")

        # Subscribe to centroid topic
        self.create_subscription(PoseArray, "/evaluator/semantic_centroid_targets", self._centroid_cb, 10)

        # Publisher for nearest path visualization
        self.path_pub = self.create_publisher(Path, "/evaluator/nearest_path", 10)

        self.centroids = []
        self.ready = False
        self.best_path = None
        self.shortest_length = float("inf")

        self.get_logger().info("⏳ Waiting for /evaluator/compute_path_to_pose server...")
        self.client.wait_for_server()
        self.get_logger().info("✅ Planner action server available.")
        self.get_logger().info("Waiting for centroid targets on /evaluator/semantic_centroid_targets...")

        # Start loop timer
        self.timer = self.create_timer(2.0, self._plan_all_targets)

    # ------------------------------------------------------------------
    def _centroid_cb(self, msg: PoseArray):
        if not msg.poses:
            self.get_logger().warn("Received empty PoseArray.")
            return
        self.centroids = msg.poses
        self.ready = True
        self.get_logger().info(f"🎯 Received {len(self.centroids)} centroid targets.")

    # ------------------------------------------------------------------
    def _make_pose(self, x, y, frame_id="map"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    # ------------------------------------------------------------------
    def _plan_all_targets(self):
        """Sequentially plan to all centroid targets, publishing the nearest path."""
        if not self.ready or not self.centroids:
            return

        self.best_path = None
        self.shortest_length = float("inf")

        for i, target in enumerate(self.centroids):
            start = self._make_pose(0.0, 0.0, "base_link")
            goal = self._make_pose(target.position.x, target.position.y, "robot_original_pose_at_scan")

            goal_msg = ComputePathToPose.Goal()
            goal_msg.start = start
            goal_msg.goal = goal
            goal_msg.use_start = True

            self.get_logger().info(
                f"🧭 Planning to target {i+1}/{len(self.centroids)} → "
                f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})"
            )

            # Send goal and wait synchronously for result
            send_future = self.client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
            goal_handle = send_future.result()

            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn(f"Planner rejected goal {i+1}.")
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)

            result = result_future.result()
            if not result or not result.result.path.poses:
                self.get_logger().warn(f"Path {i+1} is empty.")
                continue

            path = result.result.path
            length = self._path_length(path)
            self.get_logger().info(f"✅ Path {i+1}: {length:.2f} m ({len(path.poses)} poses)")

            # Update shortest path and publish live
            if length < self.shortest_length:
                self.shortest_length = length
                self.best_path = path
                self.path_pub.publish(path)
                self.get_logger().info(f"📡 Published nearest path ({length:.2f} m)")

            # Small pause to avoid spamming planner
            time.sleep(0.2)

    # ------------------------------------------------------------------
    def _path_length(self, path: Path):
        length = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            length += math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)
        return length


# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
