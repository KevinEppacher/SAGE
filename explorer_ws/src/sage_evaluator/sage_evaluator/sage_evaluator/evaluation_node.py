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
        self.experiment_id = 'EXP_001'
        self.episode_id = 'E01'
        self.phase = 'START'
        self.prompt_texts = ['bed', 'chair', 'table', 'lamp']

        # --- Internal state ---
        self.start_time = time.time()
        self.current_prompt = None
        self.prompt_index = 0
        self.published = False

        # --- Startup timer ---
        self.timer = self.create_timer(2.0, self.publish_event_once)

        self.get_logger().info("🧭 Evaluation Dashboard initialized. Waiting for BT feedback...")

    # ======================================================
    # Publish EvaluationEvent (setup message)
    # ======================================================
    def publish_event_once(self):
        if self.published:
            return

        prompt_array = SemanticPromptArray()
        prompt_array.header = Header()
        prompt_array.header.stamp = self.get_clock().now().to_msg()

        for text in self.prompt_texts:
            p = SemanticPrompt()
            p.header = Header()
            p.text_query = text
            prompt_array.prompt_list.append(p)

        event = EvaluationEvent()
        event.phase = self.phase
        event.experiment_id = self.experiment_id
        event.episode_id = self.episode_id
        event.prompt_list = prompt_array
        event.elapsed_time = 0.0
        event.success = False
        event.reason = "init"
        event.goal_pose = Point()
        event.start_pose = Point()

        self.event_pub.publish(event)
        self.published = True
        self.timer.cancel()
        self.start_time = time.time()

        self.get_logger().info(
            f"📤 Published EvaluationEvent: {len(self.prompt_texts)} prompts → "
            f"{[p.text_query for p in prompt_array.prompt_list]}"
        )

    # ======================================================
    # Handle feedback from BT node (/evaluation/iteration_status)
    # ======================================================
    def status_callback(self, msg: String):
        elapsed = time.time() - self.start_time
        timestamp = datetime.now().strftime("%H:%M:%S")

        text = msg.data.strip()
        parts = text.split("'")
        current_prompt = parts[1] if len(parts) > 1 else "unknown"

        # detect status keywords
        status = "UNKNOWN"
        for key in ["SUCCESS", "FAILURE", "TIMEOUT"]:
            if key in text:
                status = key
                break

        symbol = {"SUCCESS": "✅", "FAILURE": "❌", "TIMEOUT": "⏳"}.get(status, "🔹")

        # --- figure out which prompt comes next ---
        self.current_prompt = current_prompt
        self.prompt_index += 1
        next_prompt = (
            self.prompt_texts[self.prompt_index]
            if self.prompt_index < len(self.prompt_texts)
            else "— none (experiment complete) —"
        )

        # --- pretty console log ---
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


def main(args=None):
    rclpy.init(args=args)
    node = EvaluationDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
