#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from evaluator_msgs.msg import EvaluationEvent
from multimodal_query_msgs.msg import SemanticPrompt, SemanticPromptArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, DurabilityPolicy

class EvaluationPublisher(Node):
    def __init__(self):
        super().__init__('evaluation_publisher')

        # Publisher to /evaluation/event
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(EvaluationEvent, '/evaluation/event', qos)
        
        # Example configuration
        self.experiment_id = 'EXP_001'
        self.episode_id = 'E01'
        self.phase = 'START'

        # Define your prompts (only text_query required)
        self.prompt_texts = ['bed', 'chair', 'table', 'lamp']

        # Wait a little for subscribers (like your BT node)
        self.timer = self.create_timer(2.0, self.publish_event_once)
        self.published = False

    def publish_event_once(self):
        """Publish the EvaluationEvent once, then stop timer."""
        if self.published:
            return

        # Fill the SemanticPromptArray
        prompt_array = SemanticPromptArray()
        prompt_array.header = Header()
        prompt_array.header.stamp = self.get_clock().now().to_msg()

        for text in self.prompt_texts:
            p = SemanticPrompt()
            p.header = Header()
            p.text_query = text
            prompt_array.prompt_list.append(p)

        # Build EvaluationEvent message
        event = EvaluationEvent()
        event.phase = self.phase
        event.experiment_id = self.experiment_id
        event.episode_id = self.episode_id
        event.prompt_list = prompt_array
        event.confidence = 0.0
        event.elapsed_time = 0.0
        event.shortest_path_length = 0.0
        event.actual_path_length = 0.0
        event.success = False
        event.reason = 'init'
        event.goal_pose = Point(x=0.0, y=0.0, z=0.0)
        event.start_pose = Point(x=0.0, y=0.0, z=0.0)

        # Publish once
        self.publisher.publish(event)
        self.get_logger().info(
            f'📤 Published EvaluationEvent with {len(prompt_array.prompt_list)} prompts: '
            f'{[p.text_query for p in prompt_array.prompt_list]}'
        )

        self.published = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = EvaluationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
