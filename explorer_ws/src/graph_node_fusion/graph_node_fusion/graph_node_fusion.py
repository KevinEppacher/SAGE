#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from graph_node_msgs.msg import GraphNodeArray, GraphNode
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt


class GraphNodeFusion(Node):
    def __init__(self):
        super().__init__("graph_node_fusion")

        # --- Parameters ---
        self.declare_parameter("approach_radius", 1.0)
        self.declare_parameter("max_visited", 200)
        self.declare_parameter("exploration_weight", 0.7)
        self.declare_parameter("exploration_topic", "/exploration_graph_nodes/graph_nodes")
        self.declare_parameter("exploitation_topic", "/exploitation_graph_nodes/graph_nodes")
        self.declare_parameter("detection_topic", "/detection_graph_nodes/graph_nodes")

        gp = self.get_parameter
        self.approach_radius = gp("approach_radius").value
        self.max_visited = gp("max_visited").value
        self.exploration_weight = gp("exploration_weight").value
        self.exploration_topic = gp("exploration_topic").value
        self.exploitation_topic = gp("exploitation_topic").value
        self.detection_topic = gp("detection_topic").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Subscriptions and Publishers ---
        self.create_subscription(GraphNodeArray, self.exploration_topic, self.cb_exploration, qos)
        self.create_subscription(GraphNodeArray, self.exploitation_topic, self.cb_exploitation, qos)
        self.create_subscription(GraphNodeArray, self.detection_topic, self.cb_detection, qos)

        self.pub_fused_exploration = self.create_publisher(
            GraphNodeArray, "exploration_graph_nodes/graph_nodes", qos)
        self.pub_fused_detection = self.create_publisher(
            GraphNodeArray, "detection_graph_nodes/graph_nodes", qos)
        self.pub_markers = self.create_publisher(MarkerArray, "graph_node_markers", qos)

        # --- TF ---
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # --- State ---
        self.exploration_nodes = []
        self.exploitation_nodes = []
        self.detection_nodes = []
        self.visited_nodes = []

        # --- Single timer ---
        self.create_timer(0.1, self._loop)  # 10 Hz

    # ---------- Subscriptions ----------
    def cb_exploration(self, msg): self.exploration_nodes = list(msg.nodes)
    def cb_exploitation(self, msg): self.exploitation_nodes = list(msg.nodes)
    def cb_detection(self, msg): self.detection_nodes = list(msg.nodes)

    # ---------- Main loop ----------
    def _loop(self):
        # Get robot pose once per tick
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return

        self._update_visited(robot_pose)
        fused_exploration, fused_detection = self._fuse_nodes(robot_pose)

        # --- Debug info ---
        for n in self.exploration_nodes:
            dist = math.hypot(n.position.x - robot_pose[0], n.position.y - robot_pose[1])
            self.get_logger().info(f"Node {n.id}: distance {dist:.2f} m")

        self.get_logger().info(f"Visited nodes: {len(self.visited_nodes)}")

        # publish fused arrays
        msg_ex = GraphNodeArray()
        msg_ex.nodes = fused_exploration
        self.pub_fused_exploration.publish(msg_ex)

        msg_det = GraphNodeArray()
        msg_det.nodes = fused_detection
        self.pub_fused_detection.publish(msg_det)

        self._publish_markers(fused_exploration, fused_detection)

    def _get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.1))
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    # ---------- TF and visited logic ----------
    def _update_visited(self, robot_pose):
        rx, ry = robot_pose
        for n in self.exploration_nodes:
            dist = math.hypot(n.position.x - rx, n.position.y - ry)
            if dist < self.approach_radius:
                self.get_logger().info(f"Marking node {n.id} as visited (dist {dist:.2f})")
                self.visited_nodes.append((n.position.x, n.position.y))
        self.visited_nodes = self.visited_nodes[-self.max_visited:]

    # ---------- Fusion ----------
    def _fuse_nodes(self, robot_pose):
        fused_exploration, fused_detection = [], []
        rx, ry = robot_pose

        for n in self.exploration_nodes:
            n.score *= self.exploration_weight
        for n in self.exploitation_nodes:
            n.score *= (1.0 - self.exploration_weight)

        all_exp = self.exploration_nodes + self.exploitation_nodes
        all_exp.sort(key=lambda n: n.score, reverse=True)

        for n in all_exp:
            if not self._is_visited(n):
                fused_exploration.append(n)
            else:
                self.get_logger().info(f"Node {n.id} filtered (already visited)")

        fused_detection = sorted(self.detection_nodes, key=lambda n: n.score, reverse=True)
        return fused_exploration, fused_detection

    def _is_visited(self, node):
        for vx, vy in self.visited_nodes:
            hypot = math.hypot(node.position.x - vx, node.position.y - vy)
            self.get_logger().info(f"  -> Distance to visited point: {hypot:.2f}")
            if hypot < self.approach_radius:
                return True
        return False

    # ---------- Visualization ----------
    def _publish_markers(self, exploration_nodes, detection_nodes):
        marker_array = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        all_scores = [n.score for n in exploration_nodes + detection_nodes] or [0.0]
        min_s, max_s = min(all_scores), max(all_scores)
        norm = plt.Normalize(vmin=min_s, vmax=max_s)
        cmap = plt.get_cmap("inferno")

        def make_marker(node, idx, score):
            r, g, b, a = cmap(norm(score))
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "graph_nodes"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = node.position
            m.pose.position.z += 0.05  # small lift off ground
            m.pose.orientation.w = 1.0
            scale = 0.1 + 0.3 * norm(score)
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            m.lifetime.sec = 1
            return m

        idx = 0
        for n in exploration_nodes + detection_nodes:
            marker_array.markers.append(make_marker(n, idx, n.score))
            idx += 1

        for vx, vy in self.visited_nodes[-200:]:
            n = GraphNode()
            n.position.x, n.position.y, n.position.z = vx, vy, 0.05
            m = make_marker(n, idx, 0.5)
            m.color = ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.6)
            m.scale.x = m.scale.y = m.scale.z = 0.1
            marker_array.markers.append(m)
            idx += 1

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = GraphNodeFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
