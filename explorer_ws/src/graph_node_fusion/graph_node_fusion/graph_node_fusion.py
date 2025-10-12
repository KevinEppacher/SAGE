#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from graph_node_msgs.msg import GraphNodeArray, GraphNode
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener, LookupException
from geometry_msgs.msg import TransformStamped
import math
import random
from rclpy.duration import Duration
import matplotlib.pyplot as plt
import numpy as np

BOLD = "\033[1m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

class GraphNodeFusion(Node):
    def __init__(self):
        super().__init__('graph_node_fusion')
        self._declare_params()
        self._read_params()

        # QoS and topics
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(GraphNodeArray, self.exploration_topic, self.cb_exploration, qos)
        self.create_subscription(GraphNodeArray, self.exploitation_topic, self.cb_exploitation, qos)
        self.create_subscription(GraphNodeArray, self.detection_topic, self.cb_detection, qos)

        self.pub_fused_exploration = self.create_publisher(GraphNodeArray,
            'exploration_graph_nodes/graph_nodes', qos)
        self.pub_fused_detection = self.create_publisher(GraphNodeArray,
            'detection_graph_nodes/graph_nodes', qos)
        self.pub_markers = self.create_publisher(MarkerArray, 'graph_node_markers', qos)

        # TF interface
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.exploration_nodes = []
        self.exploitation_nodes = []
        self.detection_nodes = []
        self.visited_nodes = []

        # High-rate tasks
        self.high_timer = self.create_timer(0.1, self.high_freq_cb)   # 10 Hz
        # Low-rate publishing
        self.low_timer = self.create_timer(1.0, self.low_freq_cb)     # 1 Hz

    # ---------- Callbacks ----------
    def cb_exploration(self, msg):
        self.exploration_nodes = list(msg.nodes)

    def cb_exploitation(self, msg):
        self.exploitation_nodes = list(msg.nodes)

    def cb_detection(self, msg):
        self.detection_nodes = list(msg.nodes)
        print(f"Received {len(self.detection_nodes)} detection nodes")
        self.pub_fused_detection.publish(GraphNodeArray(nodes=self.detection_nodes))

    # ---------- High-rate loop ----------
    def high_freq_cb(self):
        self._mark_visited_via_tf()
        self.fused_exploration, self.fused_detection = self._fuse_nodes()
        self._publish_markers(self.fused_exploration, self.fused_detection)

    # ---------- Low-rate publishing ----------
    def low_freq_cb(self):
        msg_ex = GraphNodeArray(nodes=self.fused_exploration)
        msg_det = GraphNodeArray(nodes=self.fused_detection)
        self.pub_fused_exploration.publish(msg_ex)
        self.pub_fused_detection.publish(msg_det)

    # ---------- Mark visited ----------
    def _mark_visited_via_tf(self):
        try:
            # use latest transform with timeout and catch extrapolation
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
        except Exception as e:
            # non-fatal handling
            self.get_logger().warn(f"TF lookup skipped: {str(e)}", throttle_duration_sec=2.0)
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y

        new_visited = []
        for node in self.exploration_nodes:
            dx = node.position.x - rx
            dy = node.position.y - ry
            if math.hypot(dx, dy) < self.approach_radius:
                new_visited.append((node.position.x, node.position.y))

        if new_visited:
            self.visited_nodes.extend(new_visited)
            self.visited_nodes = self.visited_nodes[-self.max_visited:]

    # ---------- Fusion with weights ----------
    def _fuse_nodes(self):
        fused_exploration = []
        fused_detection = []

        # Weighting: modify scores before sort
        for n in self.exploration_nodes:
            n.score *= self.exploration_weight
        for n in self.exploitation_nodes:
            n.score *= (1.0 - self.exploration_weight)

        all_exp = self.exploration_nodes + self.exploitation_nodes
        all_exp.sort(key=lambda n: n.score, reverse=True)

        for n in all_exp:
            if not self._is_visited(n):
                fused_exploration.append(n)

        fused_detection = sorted(self.detection_nodes, key=lambda n: n.score, reverse=True)
        return fused_exploration, fused_detection

    def _is_visited(self, node):
        for vx, vy in self.visited_nodes:
            if math.hypot(node.position.x - vx, node.position.y - vy) < self.approach_radius:
                return True
        return False
    
        # ---------- Parameters ----------

    def _declare_params(self):
        self.declare_parameter('approach_radius', 1.0)
        self.declare_parameter('max_visited', 200)
        self.declare_parameter('exploration_weight', 0.7)
        self.declare_parameter('exploration_topic', '/exploration_graph_nodes/graph_nodes')
        self.declare_parameter('exploitation_topic', '/exploitation_graph_nodes/graph_nodes')
        self.declare_parameter('detection_topic', '/detection_graph_nodes/graph_nodes')

    def _read_params(self):
        gp = self.get_parameter
        self.approach_radius = gp('approach_radius').get_parameter_value().double_value
        self.max_visited = gp('max_visited').get_parameter_value().integer_value
        self.exploration_weight = gp('exploration_weight').get_parameter_value().double_value
        self.exploration_topic = gp('exploration_topic').get_parameter_value().string_value
        self.exploitation_topic = gp('exploitation_topic').get_parameter_value().string_value
        self.detection_topic = gp('detection_topic').get_parameter_value().string_value

    def _publish_markers(self, exploration_nodes, detection_nodes):
        import matplotlib.pyplot as plt
        marker_array = MarkerArray()

        # Clear previous markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        # Normalize scores for color + size
        all_scores = [n.score for n in exploration_nodes + detection_nodes] or [0.0]
        min_s, max_s = min(all_scores), max(all_scores)
        norm = plt.Normalize(vmin=min_s, vmax=max_s)
        cmap = plt.get_cmap("viridis")

        def score_to_color(score):
            r, g, b, a = cmap(norm(score))
            return ColorRGBA(r=r, g=g, b=b, a=a)

        def score_to_scale(score):
            # scale between 0.1 and 0.4 meters
            return 0.1 + 0.3 * norm(score)

        def make_marker(node, idx, color, scale):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "graph_nodes"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = node.position
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color = color
            m.lifetime.sec = 1
            return m

        idx = 0
        # Exploration + detection nodes scaled and colored by score
        for node in exploration_nodes + detection_nodes:
            c = score_to_color(node.score)
            s = score_to_scale(node.score)
            marker_array.markers.append(make_marker(node, idx, c, s))
            idx += 1

        # Visited nodes constant small blue
        for vx, vy in self.visited_nodes[-200:]:
            node = GraphNode()
            node.position.x, node.position.y, node.position.z = vx, vy, 0.05
            color = ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.6)
            marker_array.markers.append(make_marker(node, idx, color, 0.1))
            idx += 1

        self.pub_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GraphNodeFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('GraphNodeFusion interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
