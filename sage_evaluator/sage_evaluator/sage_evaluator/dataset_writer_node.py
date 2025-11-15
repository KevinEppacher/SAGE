#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import os
import json
import numpy as np

from evaluator_msgs.msg import (
    EvaluationEvent,
    EvaluationResult,            # May still be used later
    EvaluationSummary,
    EvaluationPathEvent
)

from nav_msgs.msg import Path


class DatasetWriter(Node):
    def __init__(self):
        super().__init__("dataset_writer")

        self.declare_parameter("dataset_root", "")

        self.dataset_root = self.get_parameter("dataset_root").value

        # state
        self.current_event = None
        self.shortest_path_length = None
        self.shortest_goal = None
        self.results = {}                 # prompt → dict

        # QoS matching dashboard
        event_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # -----------------------
        # Subscribers
        # -----------------------

        # full evaluation config
        self.event_sub = self.create_subscription(
            EvaluationEvent,
            "/evaluator/dashboard/event",
            self.on_event,
            event_qos
        )

        # shortest Nav2 plan
        self.plan_sub = self.create_subscription(
            Path,
            "/evaluator/shortest_path",
            self.on_shortest_path,
            10
        )

        # actual path from TrajectoryRecorder
        self.path_event_sub = self.create_subscription(
            EvaluationPathEvent,
            "/evaluator/path_event",
            self.on_path_event,
            10
        )

        self.get_logger().info("DatasetWriter ready")


    # ------------------------------------------------------------
    def on_event(self, msg: EvaluationEvent):
        """New episode or prompt batch started."""
        self.current_event = msg
        self.get_logger().info(
            f"DatasetWriter: received EvaluationEvent for episode {msg.episode_id}"
        )


    # ------------------------------------------------------------
    def on_shortest_path(self, msg: Path):
        """Shortest Nav2 plan for the target centroid."""
        if not msg.poses:
            return

        length = 0.0
        poses = msg.poses
        for i in range(1, len(poses)):
            p1 = poses[i - 1].pose.position
            p2 = poses[i].pose.position
            length += np.hypot(p1.x - p2.x, p1.y - p2.y)

        self.shortest_path_length = float(length)
        goal = poses[-1].pose.position
        self.shortest_goal = [goal.x, goal.y]

        self.get_logger().info(
            f"DatasetWriter: shortest path length = {self.shortest_path_length:.2f}"
        )


    # ------------------------------------------------------------
    def on_path_event(self, msg: EvaluationPathEvent):
        """Actual robot path from start pose → goal."""
        if self.current_event is None:
            self.get_logger().warn("Ignoring path event: no EvaluationEvent received yet")
            return

        dataset_root = self.current_event.save_path

        prompt = msg.prompt.replace(" ", "_")

        metrics_path = os.path.join(
            dataset_root,
            "detections",
            prompt,
            "metrics.json"
        )
        os.makedirs(os.path.dirname(metrics_path), exist_ok=True)

        metrics = {
            "prompt": msg.prompt,
            "shortest_path_length": self.shortest_path_length,
            "shortest_goal": self.shortest_goal,
            "actual_path_length": msg.actual_length,
            "start_pose": [msg.start_pose.x, msg.start_pose.y],
            "end_pose": [msg.end_pose.x, msg.end_pose.y],
        }

        # write prompt metrics
        with open(metrics_path, "w") as f:
            json.dump(metrics, f, indent=4)

        self.results[msg.prompt] = metrics

        self.get_logger().info(
            f"DatasetWriter: wrote metrics for prompt '{msg.prompt}'"
        )

        # if all prompts finished → write summary
        prompt_count = len(self.current_event.prompt_list.prompt_list)
        if len(self.results) == prompt_count:
            self.write_summary()


    # ------------------------------------------------------------
    def write_summary(self):
        """Write summary.json for the entire episode."""
        summary_path = os.path.join(
            self.current_event.save_path,
            "summary.json"
        )

        with open(summary_path, "w") as f:
            json.dump(self.results, f, indent=4)

        self.get_logger().info(
            f"DatasetWriter: wrote summary.json with {len(self.results)} prompts"
        )



def main(args=None):
    rclpy.init(args=args)
    node = DatasetWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
