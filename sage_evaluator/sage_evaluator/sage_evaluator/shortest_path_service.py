#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Path
from multimodal_query_msgs.msg import SemanticPrompt
from evaluator_msgs.srv import GetShortestPath
import threading, time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

class ShortestPathService(Node):
    def __init__(self):
        super().__init__("shortest_path_service")

        # ---------------- Parameters ----------------
        self.declare_parameter("prompt_topic", "/evaluator/prompt")
        self.declare_parameter("path_topic", "/evaluator/shortest_path")
        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("debug_info", False)

        prompt_topic = self.get_parameter("prompt_topic").get_parameter_value().string_value
        path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        self.timeout_sec = self.get_parameter("timeout_sec").get_parameter_value().double_value
        self.debug = self.get_parameter("debug_info").get_parameter_value().bool_value

        # ---------------- Callback group ----------------
        self.callback_group = ReentrantCallbackGroup()

        # ---------------- ROS I/O ----------------
        qos_semantic_prompt = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.prompt_pub = self.create_publisher(SemanticPrompt, prompt_topic, qos_semantic_prompt)
        
        self.path_sub = self.create_subscription(
            Path, path_topic, self._path_cb, 10, callback_group=self.callback_group
        )
        self.srv = self.create_service(
            GetShortestPath,
            "get_shortest_path",
            self._handle_request,
            callback_group=self.callback_group,
        )

        # ---------------- Internal state ----------------
        self.latest_path = None
        self.path_lock = threading.Lock()
        self.path_version = 0

        # ---------------- Info ----------------
        self.get_logger().info("ShortestPathService ready (multithreaded).")
        self.get_logger().info(f"Prompt topic : {prompt_topic}")
        self.get_logger().info(f"Path topic   : {path_topic}")
        self.get_logger().info(f"Timeout      : {self.timeout_sec} s")
        self.get_logger().info(f"Debug mode   : {self.debug}")

    # ----------------------------------------------------
    def _path_cb(self, msg):
        with self.path_lock:
            self.path_version += 1
            self.latest_path = msg
        if self.debug:
            self.get_logger().info(f"Received path ({len(msg.poses)} poses) [version={self.path_version}]")

    # ----------------------------------------------------
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
            time.sleep(0.05)  # yield to executor
            with self.path_lock:
                if self.path_version > current_version and self.latest_path:
                    res.success = True
                    res.path = self.latest_path
                    res.path_length = self._compute_path_length(self.latest_path)
                    if self.debug:
                        self.get_logger().info(
                            f"Returning path (length={res.path_length:.2f} m, poses={len(res.path.poses)})"
                        )
                    self.latest_path = None
                    return res

        res.success = False
        res.path_length = 0.0
        self.get_logger().warn(f"No path received within {self.timeout_sec} s timeout.")
        return res

    # ----------------------------------------------------
    def _compute_path_length(self, path):
        total = 0.0
        poses = path.poses
        for i in range(1, len(poses)):
            p1, p2 = poses[i - 1].pose.position, poses[i].pose.position
            total += ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2) ** 0.5
        return total


# --------------------------------------------------------
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
