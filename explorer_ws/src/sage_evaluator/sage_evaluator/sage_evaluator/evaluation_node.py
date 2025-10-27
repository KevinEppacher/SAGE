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

        # QoS for persistent evaluator messages
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Publisher: initial evaluation event
        self.event_pub = self.create_publisher(EvaluationEvent, '/evaluation/event', qos)

        # Subscriber: BT iteration status feedback
        self.status_sub = self.create_subscription(
            String, '/evaluation/iteration_status', self.status_callback, 10
        )

        # Example configuration
        self.experiment_id = 'EXP_001'
        self.episode_id = 'E01'
        self.phase = 'START'
        self.prompt_texts = ['bed', 'chair', 'table', 'lamp']

        # Timing
        self.start_time = time.time()
        self.current_prompt = None
        self.prompt_index = 0

        # Publish config after delay
        self.timer = self.create_timer(2.0, self.publish_event_once)
        self.published = False

        self.get_logger().info("🧭 Evaluation Dashboard initialized. Waiting for BT feedback...")

    # ======================================================
    # Publish EvaluationEvent (setup)
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

        self.get_logger().info(
            f"📤 Published EvaluationEvent: {len(self.prompt_texts)} prompts → {[p.text_query for p in prompt_array.prompt_list]}"
        )

        self.published = True
        self.timer.cancel()
        self.start_time = time.time()

    # ======================================================
    # Receive iteration feedback from BT
    # ======================================================
    def status_callback(self, msg: String):
        elapsed = time.time() - self.start_time
        timestamp = datetime.now().strftime("%H:%M:%S")

        # Parse message structure (from ForEachEvaluationPrompt)
        text = msg.data.strip()
        parts = text.split("'")

        current_prompt = parts[1] if len(parts) > 1 else "unknown"
        status = "unknown"
        for keyword in ["SUCCESS", "FAILURE", "TIMEOUT"]:
            if keyword in text:
                status = keyword
                break

        # Emojis for readability
        symbol = {"SUCCESS": "✅", "FAILURE": "❌", "TIMEOUT": "⏳"}.get(status, "🔹")

        # Console log (human-readable summary)
        self.get_logger().info(
            f"\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"🕓 [{timestamp}]  Prompt: {current_prompt}\n"
            f"⏱  Elapsed Time: {elapsed:6.2f} s\n"
            f"🏁  Status: {symbol} {status}\n"
            f"📄  Message: {text}\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        )

        # Update current state
        self.current_prompt = current_prompt
        self.prompt_index += 1


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
