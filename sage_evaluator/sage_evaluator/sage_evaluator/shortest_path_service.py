#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Path
from multimodal_query_msgs.msg import SemanticPrompt
from evaluator_msgs.srv import GetShortestPath
import threading, time

class ShortestPathService(Node):
    def __init__(self):
        super().__init__("shortest_path_service")

        self.callback_group = ReentrantCallbackGroup()

        self.prompt_pub = self.create_publisher(
            SemanticPrompt, "/evaluator/prompt", 10
        )

        self.path_sub = self.create_subscription(
            Path, "/evaluator/shortest_path", self._path_cb, 10,
            callback_group=self.callback_group
        )

        self.srv = self.create_service(
            GetShortestPath, "get_shortest_path",
            self._handle_request, callback_group=self.callback_group
        )

        self.latest_path = None
        self.path_lock = threading.Lock()
        self.latest_stamp = 0.0
        self.timeout_sec = 30.0
        self.path_version = 0 

        self.get_logger().info("ShortestPathService ready (multithreaded).")


    def _path_cb(self, msg):
        with self.path_lock:
            self.path_version += 1
            self.latest_path = msg

    def _handle_request(self, req, res):
        self.get_logger().info(f"Prompt published: '{req.query}'")
        with self.path_lock:
            self.latest_path = None
            current_version = self.path_version

        prompt = SemanticPrompt()
        prompt.text_query = req.query
        self.prompt_pub.publish(prompt)

        start = time.time()
        while time.time() - start < self.timeout_sec:
            time.sleep(0.05)
            with self.path_lock:
                if self.path_version > current_version and self.latest_path:
                    res.success = True
                    res.path = self.latest_path
                    res.path_length = self._compute_path_length(self.latest_path)
                    self.latest_path = None
                    return res

        res.success = False
        res.path_length = 0.0
        self.get_logger().warn(f"No path within {self.timeout_sec} s.")
        return res

    def _compute_path_length(self, path):
        total = 0.0
        poses = path.poses
        for i in range(1, len(poses)):
            p1, p2 = poses[i-1].pose.position, poses[i].pose.position
            total += ((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)**0.5
        return total


def main(args=None):
    rclpy.init(args=args)
    node = ShortestPathService()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
