#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
import math


class Nav2PathTester(Node):
    def __init__(self):
        super().__init__("nav2_path_tester")

        # Action client for the global planner
        self.client = ActionClient(self, ComputePathToPose, "/evaluator/compute_path_to_pose")

        # Example poses (you can edit these)
        self.start = self._make_pose(0.0, 0.0, frame_id="base_link")
        self.goal = self._make_pose(10.0, -5.0, frame_id="robot_original_pose_at_scan")

        self.get_logger().info("Waiting for /evaluator/compute_path_to_pose action server...")
        self.client.wait_for_server()
        self.get_logger().info("Planner available. Sending goal...")

        # Build the goal request
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self.start
        goal_msg.goal = self.goal
        goal_msg.use_start = True  # explicitly define start pose

        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------
    def _make_pose(self, x, y, frame_id = "map"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # no rotation for simplicity
        return pose

    # ------------------------------------------------------------------
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by planner.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    # ------------------------------------------------------------------
    def _result_callback(self, future):
        result = future.result().result
        path = result.path
        self.get_logger().info(f"Planner returned {len(path.poses)} poses.")

        # Compute total length of the path
        if len(path.poses) > 1:
            length = 0.0
            for i in range(1, len(path.poses)):
                p1 = path.poses[i - 1].pose.position
                p2 = path.poses[i].pose.position
                length += math.sqrt(
                    (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2
                )
            self.get_logger().info(f"Total path length: {length:.2f} m")
        else:
            self.get_logger().warn("Received an empty or single-point path.")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathTester()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
