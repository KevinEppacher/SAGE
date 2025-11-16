#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from evaluator_msgs.msg import EvaluationEvent
from multimodal_query_msgs.msg import SemanticPrompt, SemanticPromptArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sage_datasets.utils import DatasetManager

class EvaluatorDashboard(Node):
    def __init__(self):
        super().__init__("evaluator_dashboard")

        # Parameters
        self.declare_parameter("scene", "")
        self.declare_parameter("version", "")
        self.declare_parameter("episode_id", "")
        self.declare_parameter("prompt_set", "")

        self.scene = self.get_parameter("scene").value
        self.version = self.get_parameter("version").value
        self.episode_id = self.get_parameter("episode_id").value
        self.prompt_set = self.get_parameter("prompt_set").value

        # Dataset loading
        dataset = DatasetManager(self.scene, self.version, self.episode_id)
        self.episode_dir = dataset.episode_dir()
        self.get_logger().info(f"Episode directory: {self.episode_dir}")
        prompts_data = dataset.prompts()

        if self.prompt_set not in prompts_data:
            raise KeyError(f"Prompt set '{self.prompt_set}' not found for {self.scene}/{self.episode_id}")

        self.prompt_list = prompts_data[self.prompt_set]  # list[str]

        self.get_logger().info(
            f"Loaded {len(self.prompt_list)} prompts for {self.scene}/{self.episode_id}"
        )

        # State
        self.current_index = 0


        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL       # <-- required
        qos.reliability = ReliabilityPolicy.RELIABLE

        # Publishers
        self.event_pub = self.create_publisher(EvaluationEvent, "dashboard/event", qos)
        self.prompt_pub = self.create_publisher(SemanticPrompt, "prompt", qos)

        # Subscriber for BT feedback
        self.status_sub = self.create_subscription(
            String,
            "/evaluator/dashboard/iteration_status",
            self.status_callback,
            10
        )

        # Send initial event ONCE with full prompt list
        self.publish_initial_event()

        # Publish first prompt
        self.publish_prompt(self.prompt_list[0])

    # ------------------------------------------------------------------
    def publish_initial_event(self):
        """Sends one EvaluationEvent containing the full prompt list."""
        event = EvaluationEvent()
        event.experiment_id = f"EXP_{self.episode_id}"
        event.episode_id = self.episode_id
        event.scene = self.scene
        event.save_path = self.episode_dir
        event.current_prompt = self.prompt_list[self.current_index]
        event.start_pose = Point()
        event.goal_pose = Point()
        event.stamp = self.get_clock().now().to_msg()

        # Fill prompt list array
        prompt_array = SemanticPromptArray()
        for p in self.prompt_list:
            sp = SemanticPrompt()
            sp.text_query = p
            prompt_array.prompt_list.append(sp)
        event.prompt_list = prompt_array

        self.event_pub.publish(event)

        self.get_logger().info(
            f"Published EvaluationEvent with {len(self.prompt_list)} prompts."
        )

    # ------------------------------------------------------------------
    def publish_prompt(self, prompt_text: str):
        """Publish a SemanticPrompt to /evaluator/prompt."""
        msg = SemanticPrompt()
        msg.text_query = prompt_text
        self.prompt_pub.publish(msg)
        self.get_logger().info(f"Published prompt: '{prompt_text}'")

    # ------------------------------------------------------------------
    def status_callback(self, msg: String):
        """
        BT sends messages like:
        "[ForEachEvaluationPrompt] Object 'chair' finished with status: SUCCESS"
        """
        if "'" not in msg.data:
            return

        finished_prompt = msg.data.split("'")[1]

        # Check if this matches the current prompt
        if finished_prompt != self.prompt_list[self.current_index]:
            return

        self.get_logger().info(f"BT finished prompt '{finished_prompt}'")

        # Move to next prompt
        self.current_index += 1

        if self.current_index >= len(self.prompt_list):
            self.get_logger().info("All prompts completed. Evaluation finished.")
            return

        # Publish next prompt
        next_prompt = self.prompt_list[self.current_index]
        self.publish_prompt(next_prompt)


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorDashboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
