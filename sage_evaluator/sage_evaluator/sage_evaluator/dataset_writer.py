#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from evaluator_msgs.msg import EvaluationEvent, EvaluationResult, EvaluationSummary
from nav_msgs.msg import Path
import json
import os
import numpy as np

class DatasetWriter(Node):
    def __init__(self):
        super().__init__("dataset_writer")

        self.declare_parameter("dataset_root", "")

        self.dataset_root = self.get_parameter("dataset_root").value

        self.current_event = None
        self.shortest_path_length = None
        self.goal_pose = None
        self.result_list = []

        self.event_sub = self.create_subscription(
            EvaluationEvent,
            "/evaluation/event",
            self.on_event,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            "/evaluator/shortest_path",
            self.on_shortest_path,
            10
        )

        self.result_sub = self.create_subscription(
            EvaluationResult,
            "/evaluation/result",
            self.on_result,
            10
        )

    def on_event(self, msg: EvaluationEvent):
        self.current_event = msg

    def on_shortest_path(self, path_msg: Path):
        if not path_msg.poses:
            return

        length = 0.0
        poses = path_msg.poses
        for i in range(1, len(poses)):
            p1 = poses[i - 1].pose.position
            p2 = poses[i].pose.position
            length += np.hypot(p1.x - p2.x, p1.y - p2.y)

        self.shortest_path_length = float(length)
        last = poses[-1].pose.position
        self.goal_pose = [last.x, last.y]

    def on_result(self, msg: EvaluationResult):
        # Write per-prompt metrics
        path = os.path.join(
            self.dataset_root,
            msg.scene,
            "episodes",
            msg.episode_id,
            "detections",
            msg.prompt.replace(" ", "_"),
            "metrics.json"
        )

        os.makedirs(os.path.dirname(path), exist_ok=True)

        data = {
            "prompt": msg.prompt,
            "success": bool(msg.success),
            "status_reason": msg.status_reason,
            "shortest_path_length": msg.shortest_path_length,
            "actual_path_length": msg.actual_path_length,
            "elapsed_time": msg.elapsed_time
        }

        with open(path, "w") as f:
            json.dump(data, f, indent=4)

        # Store for summary
        self.result_list.append(data)

        # If all prompts done → write summary
        # (EvaluationDashboard would signal end via a final message or a known count)

