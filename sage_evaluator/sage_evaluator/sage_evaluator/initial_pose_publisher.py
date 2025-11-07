#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Declare parameter for the directory to search
        self.declare_parameter(
            'annotations_dir',
            '/app/src/sage_evaluator/datasets/matterport_isaac/00809-Qpor2mEya8F/annotations/v1.1'
        )
        annotations_dir = self.get_parameter('annotations_dir').get_parameter_value().string_value

        # Find robot_start_pose.json
        self.pose_file = os.path.join(annotations_dir, 'robot_start_pose.json')
        if not os.path.exists(self.pose_file):
            self.get_logger().error(f"Pose file not found: {self.pose_file}")
            return

        # Load the pose data
        try:
            with open(self.pose_file, 'r') as f:
                self.pose_data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load JSON file: {e}")
            return

        # Create publisher and timer
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(2.0, self.publish_pose_once)
        self.published = False

        self.get_logger().info(f"Loaded pose from: {self.pose_file}")

    def publish_pose_once(self):
        if self.published:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.pose_data.get('x', 0.0)
        msg.pose.pose.position.y = self.pose_data.get('y', 0.0)
        msg.pose.pose.position.z = self.pose_data.get('z', 0.0)

        msg.pose.pose.orientation.x = self.pose_data.get('qx', 0.0)
        msg.pose.pose.orientation.y = self.pose_data.get('qy', 0.0)
        msg.pose.pose.orientation.z = self.pose_data.get('qz', 0.0)
        msg.pose.pose.orientation.w = self.pose_data.get('qw', 1.0)

        # Add small covariance so AMCL accepts it
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.publisher.publish(msg)
        self.get_logger().info(f"Published initial pose: {self.pose_data}")
        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
