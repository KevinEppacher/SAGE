#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from evaluator_msgs.msg import EvaluationEvent
from multimodal_query_msgs.msg import SemanticPrompt, SemanticPromptArray
from geometry_msgs.msg import Point
from std_msgs.msg import String, Header
import time
from datetime import datetime

from sage_evaluator.metrics import Metrics


# --------------------------------------------------------------------------- #
# Evaluation Dashboard Node
# --------------------------------------------------------------------------- #
class EvaluationDashboard(Node):
    def __init__(self):
        super().__init__('evaluation_dashboard')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # --- ROS interfaces ---
        self.event_pub = self.create_publisher(EvaluationEvent, '/evaluation/event', qos)
        self.status_sub = self.create_subscription(
            String, '/evaluation/iteration_status', self.status_callback, 10
        )

        # --- Experiment configuration ---
        self.scene = "00809-Qpor2mEya8F"
        self.experiment_id = 'EXP_001'
        self.episode_id = 'E01'
        self.phase = 'START'
        self.prompt_texts = ['bed', 'chair', 'bath tub', 'toilet']

        # --- Internal state ---
        self.start_time = time.time()
        self.prompt_index = 0
        self.published = False

        # --- Metrics instance ---
        self.metrics = Metrics(self)

        # --- Timer ---
        self.timer = self.create_timer(2.0, self.publish_event_once)
        self.get_logger().info("🧭 Evaluation Dashboard initialized. Waiting for BT feedback...")

    # ======================================================
    def publish_event_once(self):
        if self.published:
            return

        prompt_array = SemanticPromptArray()
        prompt_array.header = Header()
        prompt_array.header.stamp = self.get_clock().now().to_msg()

        for text in self.prompt_texts:
            p = SemanticPrompt()
            p.text_query = text
            prompt_array.prompt_list.append(p)

        event = EvaluationEvent()
        event.phase = self.phase
        event.scene = self.scene
        event.experiment_id = self.experiment_id
        event.episode_id = self.episode_id
        event.prompt_list = prompt_array
        event.elapsed_time = 0.0
        event.success = False
        event.reason = "init"
        event.goal_pose = Point()
        event.start_pose = Point()

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M")
        event.save_path = f"/app/src/sage_evaluator/sage_evaluator/data/{self.scene}/{timestamp}/"

        self.event_pub.publish(event)
        self.published = True
        self.start_time = time.time()

        self.get_logger().info(
            f"📤 Published EvaluationEvent: {len(self.prompt_texts)} prompts → {[p.text_query for p in prompt_array.prompt_list]}"
        )

    # ======================================================
    def status_callback(self, msg: String):
        elapsed = time.time() - self.start_time
        timestamp = datetime.now().strftime("%H:%M:%S")
        text = msg.data.strip()

        # Parse message
        parts = text.split("'")
        current_prompt = parts[1] if len(parts) > 1 else "unknown"
        status = next((key for key in ["SUCCESS", "FAILURE", "TIMEOUT"] if key in text), "UNKNOWN")
        symbol = {"SUCCESS": "✅", "FAILURE": "❌", "TIMEOUT": "⏳"}.get(status, "🔹")

        self.prompt_index += 1
        next_prompt = (
            self.prompt_texts[self.prompt_index]
            if self.prompt_index < len(self.prompt_texts)
            else "— none (complete) —"
        )

        self.get_logger().info(
            f"\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"🕓 [{timestamp}]\n"
            f"🎯 Current Object : {current_prompt}\n"
            f"⏱  Elapsed Time   : {elapsed:6.2f} s\n"
            f"🏁  Status         : {symbol} {status}\n"
            f"➡️  Next Object    : {next_prompt}\n"
            f"📄  Message        : {text}\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        )

        # On success or completion → summarize metrics
        if status in ["SUCCESS", "FAILURE", "TIMEOUT"] and next_prompt == "— none (complete) —":
            self.metrics.summarize()
