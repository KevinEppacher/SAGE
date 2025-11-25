#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from graph_node_msgs.msg import GraphNodeArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import math
import time


class GoToBestGraphNode(Node):
    """
    Test node that subscribes to a GraphNodeArray and navigates to the
    node with the highest score. This is only intended for testing the
    scoring and fusion pipeline.
    """

    def __init__(self):
        super().__init__("go_to_best_graphnode")

        # Parameters
        self.declare_parameter("graph_topic", "/fused/exploration_graph_nodes/graph_nodes")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("min_score", 0.05)
        self.declare_parameter("cooldown_sec", 5.0)  # avoid goal-spamming

        self.graph_topic = self.get_parameter("graph_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.min_score = float(self.get_parameter("min_score").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)

        # Subscribers
        self.create_subscription(GraphNodeArray, self.graph_topic, self.cb_graph_nodes, 10)

        # Action Client
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Avoid sending goals too often
        self.last_goal_time = time.time() - self.cooldown_sec

        self.get_logger().info(f"Subscribed to: {self.graph_topic}")

    # ----------------------------------------------------------------------
    # Callback: process graph nodes and send navigation goal
    # ----------------------------------------------------------------------
    def cb_graph_nodes(self, msg: GraphNodeArray):
        if not msg.nodes:
            return

        # Find best-scoring node
        best = max(msg.nodes, key=lambda n: n.score)

        if best.score < self.min_score:
            self.get_logger().info(f"No node above score threshold ({best.score:.3f} < {self.min_score})")
            return

        now = time.time()
        if now - self.last_goal_time < self.cooldown_sec:
            return  # do not spam actions

        self.last_goal_time = now

        # Log chosen target
        self.get_logger().info(
            f"Navigating to best node: (x={best.position.x:.2f}, "
            f"y={best.position.y:.2f}, score={best.score:.2f})"
        )

        self._send_goal(best.position.x, best.position.y)

    # ----------------------------------------------------------------------
    # Send Nav2 Action Goal
    # ----------------------------------------------------------------------
    def _send_goal(self, x: float, y: float):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Set yaw = 0 → robot will reorient after reaching target
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Failed to connect to Nav2 action server.")
            return

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation finished: {result}")



def main(args=None):
    rclpy.init(args=args)
    node = GoToBestGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
