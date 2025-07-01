#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix
from rclpy.time import Time

class TFMonitorNode(Node):
    def __init__(self):
        super().__init__('tf_pose_monitor')

        # Parameter: parent and child frame
        self.parent_frame = 'map'
        self.child_frame = 'camera'

        # TF Buffer & Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer: check pose every second
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Counter for None poses
        self.none_count = 0

        self.get_logger().info(f"Monitoring transform {self.parent_frame} → {self.child_frame}")

    def timer_callback(self):
        try:
            # Current time
            now = rclpy.time.Time()
            # Try to get the transform
            trans = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, now)
            
            # Convert to matrix
            t = trans.transform.translation
            q = trans.transform.rotation
            matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
            matrix[0, 3] = t.x
            matrix[1, 3] = t.y
            matrix[2, 3] = t.z

            # Print the matrix nicely
            self.get_logger().info(f"\nTransform at time {trans.header.stamp.sec}.{trans.header.stamp.nanosec:09d}:\n{matrix}")
        except Exception as e:
            self.none_count += 1
            self.get_logger().warn(f"Transform not available (count: {self.none_count}): {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TFMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
