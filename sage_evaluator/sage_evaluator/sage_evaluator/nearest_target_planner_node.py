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

        # Parameters
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "robot_original_pose_at_scan")
        self.declare_parameter("centroid_topic", "navigable_targets")
        self.declare_parameter("shortest_path_topic", "shortest_path")
        self.declare_parameter("planner_action_topic", "/evaluator/compute_path_to_pose")
        self.declare_parameter("debug_info", False)

        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.centroid_topic = self.get_parameter("centroid_topic").value
        self.shortest_path_topic = self.get_parameter("shortest_path_topic").value
        self.debug = self.get_parameter("debug_info").value
        self.planner_action_topic = self.get_parameter("planner_action_topic").value

        # Nav2 ActionClient (namespace stays intentionally hardcoded)
        self.client = ActionClient(self, ComputePathToPose, self.planner_action_topic)

        # Subscriber (namespace stays intentionally hardcoded)
        self.create_subscription(PoseArray, self.centroid_topic, self._centroid_cb, 10)

        # Publisher (parameterized)
        self.path_pub = self.create_publisher(Path, self.shortest_path_topic, 10)

        # Internal state
        self.centroids = []
        self.index = 0
        self.planning = False

        # Shortest path storage
        self.shortest_path = None
        self.shortest_length = float("inf")

        if self.debug:
            self.get_logger().info("Waiting for planner server /evaluator/compute_path_to_pose...")

        self.client.wait_for_server()

        if self.debug:
            self.get_logger().info("Planner available. Waiting for centroid array...")

    # ------------------------------------------------------------------
    def _centroid_cb(self, msg):
        if not msg.poses:
            return

        self.centroids = msg.poses
        self.index = 0
        self.planning = False

        self.shortest_path = None
        self.shortest_length = float("inf")

        if self.debug:
            self.get_logger().info(
                f"Received {len(self.centroids)} centroids. Beginning sequential planning."
            )

        self._plan_next()

    # ------------------------------------------------------------------
    def _plan_next(self):
        if self.planning:
            return

        if self.index >= len(self.centroids):
            self._finalize_shortest_path()
            return

        target = self.centroids[self.index]
        x = target.position.x
        y = target.position.y

        if self.debug:
            self.get_logger().info(
                f"Planning {self.index + 1}/{len(self.centroids)}: x={x:.2f}, y={y:.2f}"
            )

        start = self._make_pose(0.0, 0.0, self.base_frame)
        goal = self._make_pose(x, y, self.map_frame)

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
            if self.debug:
                self.get_logger().warn(
                    f"Planner rejected goal {self.index + 1}. Skipping."
                )
            self.planning = False
            self.index += 1
            self._plan_next()
            return

        if self.debug:
            self.get_logger().info("Goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    # ------------------------------------------------------------------
    def _result_callback(self, future):
        result = future.result().result
        path = result.path
        n = len(path.poses)

        if n == 0:
            if self.debug:
                self.get_logger().warn(
                    f"Empty path for centroid {self.index + 1}."
                )
        else:
            length = self._compute_path_length(path)

            if self.debug:
                self.get_logger().info(
                    f"Path {self.index + 1}: {n} poses, length={length:.2f} m"
                )

            # Update shortest path
            if length < self.shortest_length:
                self.shortest_length = length
                self.shortest_path = path

        self.index += 1
        self.planning = False
        self._plan_next()

    # ------------------------------------------------------------------
    def _compute_path_length(self, path: Path):
        total = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            total += math.sqrt(
                (p1.x - p2.x) ** 2 +
                (p1.y - p2.y) ** 2 +
                (p1.z - p2.z) ** 2
            )
        return total

    # ------------------------------------------------------------------
    def _finalize_shortest_path(self):
        if self.shortest_path is None:
            if self.debug:
                self.get_logger().warn("No valid paths found for any centroid.")
            return

        # Ensure timestamp is valid for RViz
        self.shortest_path.header.stamp = self.get_clock().now().to_msg()

        if self.debug:
            self.get_logger().info(
                f"Publishing shortest path ({self.shortest_length:.2f} m) to "
                f"{self.shortest_path_topic}"
            )

        self.path_pub.publish(self.shortest_path)


def main(args=None):
    rclpy.init(args=args)
    node = NearestPathPlanner()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
