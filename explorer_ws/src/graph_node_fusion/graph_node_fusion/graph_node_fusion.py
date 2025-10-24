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


# ===============================================================
# Robot class: handles TF lookup and provides robot position
# ===============================================================
class Robot:
    def __init__(self, node: Node, target_frame: str = "base_link", reference_frame: str = "map"):
        self.node = node
        self.target_frame = target_frame
        self.reference_frame = reference_frame
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, node, spin_thread=True)
        self.last_pose = (0.0, 0.0)

    def get_position(self):
        """Return (x, y) of robot in map frame or last known pose."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame, self.target_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            t = tf.transform.translation
            self.last_pose = (t.x, t.y)
        except Exception as e:
            self.node.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
        return self.last_pose


# ===============================================================
# Markers class: handles visualization
# ===============================================================
class Markers:
    def __init__(self, node: Node, topic_name: str = "graph_node_markers", debug_distance: bool = False):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.node = node
        self.publisher = node.create_publisher(MarkerArray, topic_name, qos)
        self.debug_distance = debug_distance
        self.cmap = plt.get_cmap("inferno")

    def publish(self, exploration_nodes, detection_nodes, visited_nodes, robot_pose):
        marker_array = MarkerArray()

        # --- Clear existing markers ---
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = self.node.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        # --- Normalization for color mapping ---
        all_scores = [n.score for n in exploration_nodes + detection_nodes] or [0.0]
        norm = plt.Normalize(vmin=min(all_scores), vmax=max(all_scores))

        idx = 0

        # --- Draw spheres for all nodes (exploration + detection) ---
        for n in exploration_nodes + detection_nodes:
            marker_array.markers.append(self._make_sphere(n, idx, norm))
            idx += 1

        # --- Draw visited nodes (small blue dots) ---
        for vx, vy in visited_nodes[-200:]:
            marker_array.markers.append(self._make_visited(vx, vy, idx))
            idx += 1

        # --- Draw debug distance lines and text (if enabled) ---
        if self.debug_distance:
            for n in exploration_nodes:
                line_markers = self._make_line(robot_pose, (n.position.x, n.position.y), idx)
                for marker in line_markers:   # add both line and text separately
                    marker_array.markers.append(marker)
                idx += 1

        # --- Publish marker array ---
        self.publisher.publish(marker_array)

    def _make_sphere(self, node: GraphNode, idx: int, norm):
        r, g, b, a = self.cmap(norm(node.score))
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = "graph_nodes"
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = node.position
        m.pose.position.z += 0.05
        m.pose.orientation.w = 1.0
        s = 0.1 + 0.3 * norm(node.score)
        m.scale.x = m.scale.y = m.scale.z = s
        m.color = ColorRGBA(r=r, g=g, b=b, a=a)
        m.lifetime.sec = 1
        return m

    def _make_visited(self, vx: float, vy: float, idx: int):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = "visited_nodes"
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = vx
        m.pose.position.y = vy
        m.pose.position.z = 0.05
        m.scale.x = m.scale.y = m.scale.z = 0.1
        m.color = ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.6)
        m.lifetime.sec = 1
        return m

    def _make_line(self, robot_pose, node_pose, idx: int):
        rx, ry = robot_pose
        nx, ny = node_pose
        distance = math.hypot(nx - rx, ny - ry)

        # --- Line Marker (black) ---
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = "debug_lines"
        m.id = idx
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.8)

        from geometry_msgs.msg import Point
        p1 = Point(x=rx, y=ry, z=0.05)
        p2 = Point(x=nx, y=ny, z=0.05)
        m.points = [p1, p2]
        m.lifetime.sec = 1

        # --- Distance text marker ---
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.node.get_clock().now().to_msg()
        text_marker.ns = "debug_text"
        text_marker.id = idx + 10000  # ensure unique ID separate from line
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = (rx + nx) / 2.0
        text_marker.pose.position.y = (ry + ny) / 2.0
        text_marker.pose.position.z = 0.2
        text_marker.scale.z = 0.15
        text_marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        text_marker.text = f"{distance:.2f} m"
        text_marker.lifetime.sec = 1

        return [m, text_marker]


# ===============================================================
# NodeWeighter: handles scoring and weighting logic
# ===============================================================
class NodeWeighter:
    def __init__(self, exploration_weight: float = 0.7):
        self.exploration_weight = exploration_weight

    def weight_nodes(self, exploration_nodes, exploitation_nodes, detection_nodes):
        """Applies weighting between node groups and returns fused lists."""
        fused_exploration, fused_detection = [], []

        # exploration vs exploitation weighting
        for n in exploration_nodes:
            n.score *= self.exploration_weight
        for n in exploitation_nodes:
            n.score *= (1.0 - self.exploration_weight)

        all_exp = exploration_nodes + exploitation_nodes
        all_exp.sort(key=lambda n: n.score, reverse=True)
        fused_exploration.extend(all_exp)

        fused_detection = sorted(detection_nodes, key=lambda n: n.score, reverse=True)
        return fused_exploration, fused_detection


# ===============================================================
# Main orchestrator node
# ===============================================================
class GraphNodeFusion(Node):
    def __init__(self):
        super().__init__("graph_node_fusion")

        # --- Parameters ---
        self.declare_parameter("approach_radius", 1.0)
        self.declare_parameter("max_visited", 200)
        self.declare_parameter("exploration_weight", 0.7)
        self.declare_parameter("debug_distance", True)
        self.declare_parameter("exploration_topic", "/exploration_graph_nodes/graph_nodes")
        self.declare_parameter("exploitation_topic", "/exploitation_graph_nodes/graph_nodes")
        self.declare_parameter("detection_topic", "/detection_graph_nodes/graph_nodes")
        self.declare_parameter("timer_period", 0.1)

        gp = self.get_parameter
        self.approach_radius = gp("approach_radius").value
        self.max_visited = gp("max_visited").value
        self.exploration_weight = gp("exploration_weight").value
        self.debug_distance = gp("debug_distance").value
        self.exploration_topic = gp("exploration_topic").value
        self.exploitation_topic = gp("exploitation_topic").value
        self.detection_topic = gp("detection_topic").value
        self.timer_period = gp("timer_period").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Subscriptions and publishers ---
        self.create_subscription(GraphNodeArray, self.exploration_topic, self.cb_exploration, qos)
        self.create_subscription(GraphNodeArray, self.exploitation_topic, self.cb_exploitation, qos)
        self.create_subscription(GraphNodeArray, self.detection_topic, self.cb_detection, qos)

        self.pub_exploration = self.create_publisher(GraphNodeArray, "exploration_graph_nodes/graph_nodes_fused", qos)
        self.pub_detection = self.create_publisher(GraphNodeArray, "detection_graph_nodes/graph_nodes_fused", qos)

        # --- Modules ---
        self.robot = Robot(self)
        self.markers = Markers(self, debug_distance=self.debug_distance)
        self.weighter = NodeWeighter(self.exploration_weight)

        # --- State ---
        self.exploration_nodes = []
        self.exploitation_nodes = []
        self.detection_nodes = []
        self.visited_nodes = []

        self.create_timer(self.timer_period, self._loop)
        self.last_loop_time = self.get_clock().now()
        self.get_logger().info("GraphNodeFusion modular version initialized.")

    # ---------- Callbacks ----------
    def cb_exploration(self, msg): self.exploration_nodes = list(msg.nodes)
    def cb_exploitation(self, msg): self.exploitation_nodes = list(msg.nodes)
    def cb_detection(self, msg): self.detection_nodes = list(msg.nodes)

    # ---------- Main loop ----------
    def _loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now

        # --- Frequency watchdog ---
        if elapsed > self.timer_period * 2.5:  # 2.5× slower than expected
            self.get_logger().warn(
                f"Loop delay detected: {elapsed:.3f}s (expected {self.timer_period:.3f}s)"
            )

        # --- Regular logic ---
        robot_pose = self.robot.get_position()
        if robot_pose is None:
            return

        self._update_visited(robot_pose)

        fused_exp, fused_det = self.weighter.weight_nodes(
            self.exploration_nodes, self.exploitation_nodes, self.detection_nodes
        )

        fused_exp = [n for n in fused_exp if not self._is_visited(n)]
        self._publish_arrays(fused_exp, fused_det)
        self.markers.publish(fused_exp, fused_det, self.visited_nodes, robot_pose)

    # ---------- Utilities ----------
    def _update_visited(self, robot_pose):
        rx, ry = robot_pose
        for n in self.exploration_nodes:
            if math.hypot(n.position.x - rx, n.position.y - ry) < self.approach_radius:
                self.visited_nodes.append((n.position.x, n.position.y))
        self.visited_nodes = self.visited_nodes[-self.max_visited:]

    def _is_visited(self, node):
        for vx, vy in self.visited_nodes:
            if math.hypot(node.position.x - vx, node.position.y - vy) < self.approach_radius:
                return True
        return False

    def _publish_arrays(self, exploration_nodes, detection_nodes):
        msg_ex = GraphNodeArray()
        msg_ex.nodes = exploration_nodes
        self.pub_exploration.publish(msg_ex)

        msg_det = GraphNodeArray()
        msg_det.nodes = detection_nodes
        self.pub_detection.publish(msg_det)


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
