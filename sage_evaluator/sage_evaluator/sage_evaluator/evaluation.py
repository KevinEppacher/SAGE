#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from evaluator_msgs.msg import EvaluationEvent
from multimodal_query_msgs.msg import SemanticPrompt, SemanticPromptArray
from geometry_msgs.msg import Point, PoseArray
from std_msgs.msg import String, Header
import time
import json
from datetime import datetime
import os

from sage_evaluator.metrics import Metrics
from sage_datasets.utils import DatasetManager


class EvaluationDashboard(Node):
    def __init__(self):
        super().__init__('evaluation_dashboard')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # --- Declare ROS parameters ---
        self.declare_parameter("scene", "00800-TEEsavR23oF")
        self.declare_parameter("version", "v1.7")
        self.declare_parameter("episode_id", "001")
        self.declare_parameter("phase", "START")
        self.declare_parameter("prompt_set", "train")  # train | eval | zero_shot

        # --- Retrieve parameters ---
        self.scene = self.get_parameter("scene").value
        self.version = self.get_parameter("version").value
        self.episode_id = self.get_parameter("episode_id").value
        self.phase = self.get_parameter("phase").value
        self.prompt_set = self.get_parameter("prompt_set").value

        # --- ROS interfaces ---
        self.event_pub = self.create_publisher(EvaluationEvent, '/evaluation/event', qos)
        self.status_sub = self.create_subscription(
            String, '/evaluation/iteration_status', self.status_callback, 10
        )

        self.pcl_prompt_pub = self.create_publisher(SemanticPrompt, '/evaluator/prompt', 10)

        # --- Load dataset + prompts ---
        dataset = DatasetManager(self.scene, self.version, self.episode_id)
        prompts_data = dataset.prompts()
        if self.prompt_set not in prompts_data:
            raise KeyError(f"Prompt set '{self.prompt_set}' not found in prompts.json")
        self.prompt_texts = prompts_data[self.prompt_set]
        self.get_logger().info(
            f"Loaded {len(self.prompt_texts)} '{self.prompt_set}' prompts for {self.scene}/{self.episode_id}"
        )

        # --- Stable episode directory ---
        self.episode_dir = dataset.episode_dir()
        os.makedirs(self.episode_dir, exist_ok=True)

        # --- Save run summary ---
        self.save_run_summary()

        # --- Internal state ---
        self.start_time = time.time()
        self.prompt_index = 0
        self.published_event = False
        self.centroids_ready = False
        self.centroid_wait_logged = False
        self.metrics = Metrics(self)
        self.image_counter = 0

        # Subscribe to centroid targets (for handshake)
        self.centroid_sub = self.create_subscription(
            PoseArray,
            '/evaluator/semantic_centroid_targets',
            self.centroid_callback,
            10
        )

        # --- Timers ---
        # Periodically check readiness and publish EvaluationEvent
        self.event_timer = self.create_timer(2.0, self.publish_event_once)
        # Continuously publish current prompt until centroids are ready
        self.prompt_timer = self.create_timer(1.0, self.republish_prompt)

        self.get_logger().info("🧭 Evaluation Dashboard initialized. Waiting for centroids & BT feedback...")

    # ======================================================
    def centroid_callback(self, msg: PoseArray):
        """Handshake: Once valid centroids arrive, allow evaluation to start."""
        if msg.poses:
            if not self.centroids_ready:
                self.get_logger().info(
                    f"✅ Received {len(msg.poses)} centroid targets → starting evaluation pipeline."
                )
            self.centroids_ready = True
        else:
            self.get_logger().warn("Received empty PoseArray on /evaluator/semantic_centroid_targets.")

    # ======================================================
    def save_run_summary(self):
        summary_path = os.path.join(self.episode_dir, "run_summary.json")
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        summary_data = {
            "scene": self.scene,
            "version": self.version,
            "episode_id": self.episode_id,
            "prompt_set": self.prompt_set,
            "phase": self.phase,
            "start_time": timestamp,
            "num_prompts": len(getattr(self, "prompt_texts", [])),
        }

        with open(summary_path, "w") as f:
            json.dump(summary_data, f, indent=4)

        self.get_logger().info(f"🗂️  Saved run summary → {summary_path}")

    # ======================================================
    def republish_prompt(self):
        """Continuously publish current prompt until centroids arrive."""
        if self.centroids_ready or not self.prompt_texts:
            return

        current_prompt = self.prompt_texts[self.prompt_index]
        msg = SemanticPrompt()
        msg.text_query = current_prompt
        self.pcl_prompt_pub.publish(msg)

        if not self.centroid_wait_logged:
            self.get_logger().info(
                f"📡 Waiting for centroids... Re-publishing prompt '{current_prompt}' → /evaluator/prompt"
            )
            self.centroid_wait_logged = True
        else:
            # Print a simple throttled heartbeat
            self.get_logger().debug(f"Re-publishing '{current_prompt}' while waiting for centroids.")

    # ======================================================
    def publish_event_once(self):
        """Publish EvaluationEvent only once, after centroids are available."""
        if self.published_event or not self.centroids_ready:
            return

        # Stop prompt republishing once centroids are ready
        self.prompt_timer.cancel()

        # Construct prompt array
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
        event.episode_id = self.episode_id
        event.prompt_list = prompt_array
        event.elapsed_time = 0.0
        event.success = False
        event.reason = "init"
        event.goal_pose = Point()
        event.start_pose = Point()
        event.save_path = self.episode_dir

        self.event_pub.publish(event)
        self.published_event = True
        self.start_time = time.time()

        self.get_logger().info(
            f"📤 Published EvaluationEvent ({self.prompt_set}) with {len(self.prompt_texts)} prompts."
        )

    # ======================================================
    def status_callback(self, msg: String):
        elapsed = time.time() - self.start_time
        timestamp = datetime.now().strftime("%H:%M:%S")
        text = msg.data.strip()

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

        # Publish next prompt once BT requests new evaluation
        if self.prompt_index < len(self.prompt_texts):
            self.centroids_ready = False  # reset handshake for next object
            self.centroid_wait_logged = False
            self.prompt_timer.reset()  # resume continuous publishing
            self.publish_prompt_now(self.prompt_texts[self.prompt_index])

        # Final metrics after all prompts
        if status in ["SUCCESS", "FAILURE", "TIMEOUT"] and next_prompt == "— none (complete) —":
            self.metrics.summarize()

    # ======================================================
    def publish_prompt_now(self, prompt_text: str):
        """Publish a single SemanticPrompt message immediately."""
        msg = SemanticPrompt()
        msg.text_query = prompt_text
        self.pcl_prompt_pub.publish(msg)
        self.get_logger().info(f"📡 Sent prompt → /evaluator/prompt: '{prompt_text}'")
