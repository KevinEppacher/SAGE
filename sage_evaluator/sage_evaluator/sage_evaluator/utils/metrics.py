#!/usr/bin/env python3

import math
from rclpy.node import Node
from nav_msgs.msg import Path

class Metrics:
    """Compute SR and SPL metrics and log them via a ROS2 node."""

    def __init__(self, node: Node, success: bool, actual_path: Path, shortest_path: Path):
        self.node = node
        self.success = success
        self.actual_path = actual_path
        self.shortest_path = shortest_path

        self.l_shortest = self._compute_length(shortest_path)
        self.l_actual   = self._compute_length(actual_path)

        self.sr = float(success)
        self.spl = self._compute_spl()

        self._print_path_info()

    def _compute_length(self, path: Path) -> float:
        if path is None or not path.poses:
            return 0.0

        length = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            length += math.sqrt((p1.x - p2.x)**2 +
                                (p1.y - p2.y)**2 +
                                (p1.z - p2.z)**2)
        return length

    def _compute_spl(self) -> float:
        if not self.success:
            return 0.0
        if self.l_shortest <= 0.0 or self.l_actual <= 0.0:
            return 0.0
        return self.l_shortest / max(self.l_shortest, self.l_actual)

    def _print_path_info(self):
        self.node.get_logger().info("---- PATH METRICS ----")
        self.node.get_logger().info(f"Shortest path length: {self.l_shortest:.2f} m")
        self.node.get_logger().info(f"Actual executed path: {self.l_actual:.2f} m")
        self.node.get_logger().info(f"SR = {self.sr:.2f}, SPL = {self.spl:.3f}")
        self.node.get_logger().info("-----------------------")

    def to_dict(self):
        return {
            "SR": self.sr,
            "SPL": self.spl,
            "shortest_path_length": self.l_shortest,
            "actual_path_length": self.l_actual
        }

