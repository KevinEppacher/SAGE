#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from graph_node_msgs.msg import GraphNode, GraphNodeArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt


# ===============================================================
# GaussianWeighting: spatial confidence around the robot
# ===============================================================
class GaussianWeighting:
    def __init__(self, sigma: float = 1.0, sight_horizon: float = 5.0):
        self.sigma = sigma
        self.sight_horizon = sight_horizon

    def spatial_weight(self, node: GraphNode, robot_pose: Point) -> float:
        dx = node.position.x - robot_pose.x
        dy = node.position.y - robot_pose.y
        d = math.hypot(dx, dy)

        if d > self.sight_horizon:
            return 1.0  # beyond sight range → no spatial bias
        return math.exp(-d**2 / (2.0 * self.sigma**2))


# ===============================================================
# MarkerFactory: visualize nodes + Gaussian field
# ===============================================================
class MarkerFactory:
    def __init__(self, node: Node):
        self.node = node
        self.cmap = plt.get_cmap("inferno")

    def create_markers(self, exploration_nodes, detection_nodes, visited_nodes, gaussian_points=None):
        marker_array = MarkerArray()
        marker_array.markers.append(self._clear_all())

        all_scores = [n.score for n in exploration_nodes + detection_nodes] or [0.0]
        norm = plt.Normalize(vmin=min(all_scores), vmax=max(all_scores))

        idx = 0
        # Graph nodes
        for n in exploration_nodes + detection_nodes:
            color = self._score_to_color(n.score, norm)
            scale = self._score_to_scale(n.score, norm)
            marker_array.markers.append(self._make_marker(n.position, idx, color, scale))
            idx += 1

        # Visited nodes
        for vx, vy in visited_nodes[-200:]:
            pos = Point(x=vx, y=vy, z=0.05)
            color = ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.6)
            marker_array.markers.append(self._make_marker(pos, idx, color, 0.1))
            idx += 1

        # Gaussian circle visualization
        if gaussian_points:
            gvals = np.array([gp[3] for gp in gaussian_points])
            norm_g = plt.Normalize(vmin=gvals.min(), vmax=gvals.max())
            for gp in gaussian_points:
                pos = Point(x=gp[0], y=gp[1], z=gp[2])
                r, g, b, a = self.cmap(norm_g(gp[3]))
                color = ColorRGBA(r=r, g=g, b=b, a=0.5)
                marker_array.markers.append(self._make_marker(pos, idx, color, 0.05))
                idx += 1

        return marker_array

    def _clear_all(self):
        m = Marker()
        m.action = Marker.DELETEALL
        return m

    def _score_to_color(self, score, norm):
        r, g, b, a = self.cmap(norm(score))
        return ColorRGBA(r=r, g=g, b=b, a=a)

    def _score_to_scale(self, score, norm):
        return 0.1 + 0.3 * norm(score)

    def _make_marker(self, position: Point, idx: int, color: ColorRGBA, scale: float):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = "graph_nodes"
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = position
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = color
        m.lifetime.sec = 1
        return m


# ===============================================================
# NodeFusion: combine semantic score and spatial confidence
# ===============================================================
class NodeFusion:
    def __init__(self, gaussian_model: GaussianWeighting, approach_radius: float,
                 exploration_weight: float, gaussian_influence: float):
        self.gaussian_model = gaussian_model
        self.approach_radius = approach_radius
        self.exploration_weight = exploration_weight
        self.gaussian_influence = gaussian_influence

    def fuse_nodes(self, exploration, exploitation, detection, robot_pose, visited_nodes):
        for n in exploration:
            self._apply_weight(n, robot_pose, self.exploration_weight)
        for n in exploitation:
            self._apply_weight(n, robot_pose, (1.0 - self.exploration_weight))

        fused_exp = [n for n in (exploration + exploitation) if not self._is_visited(n, visited_nodes)]
        self._normalize(fused_exp)
        self._normalize(detection)
        return fused_exp, detection

    def _apply_weight(self, node, robot_pose, base_weight):
        w = self.gaussian_model.spatial_weight(node, robot_pose)
        node.score = base_weight * ((1.0 - self.gaussian_influence) * node.score +
                                    self.gaussian_influence * node.score * w)

    def _is_visited(self, node, visited_nodes):
        for vx, vy in visited_nodes:
            if math.hypot(node.position.x - vx, node.position.y - vy) < self.approach_radius:
                return True
        return False

    def _normalize(self, nodes):
        if not nodes:
            return
        scores = np.array([n.score for n in nodes], dtype=np.float32)
        smin, smax = scores.min(), scores.max()
        denom = max(smax - smin, 1e-6)
        for n in nodes:
            n.score = (n.score - smin) / denom


# ===============================================================
# Main Node
# ===============================================================
class GraphNodeFusion(Node):
    def __init__(self):
        super().__init__("graph_node_fusion")

        # Parameters
        self.declare_parameter("approach_radius", 1.0)
        self.declare_parameter("max_visited", 200)
        self.declare_parameter("exploration_weight", 0.7)
        self.declare_parameter("sigma", 0.5)
        self.declare_parameter("gaussian_influence", 0.5)
        self.declare_parameter("sight_horizon", 5.0)
        self.declare_parameter("exploration_topic", "/exploration_graph_nodes/graph_nodes")
        self.declare_parameter("exploitation_topic", "/exploitation_graph_nodes/graph_nodes")
        self.declare_parameter("detection_topic", "/detection_graph_nodes/graph_nodes")

        gp = self.get_parameter
        self.approach_radius = gp("approach_radius").value
        self.max_visited = gp("max_visited").value
        self.exploration_weight = gp("exploration_weight").value
        self.sigma = gp("sigma").value
        self.gaussian_influence = gp("gaussian_influence").value
        self.sight_horizon = gp("sight_horizon").value
        self.exploration_topic = gp("exploration_topic").value
        self.exploitation_topic = gp("exploitation_topic").value
        self.detection_topic = gp("detection_topic").value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(GraphNodeArray, self.exploration_topic, self.cb_exploration, qos)
        self.create_subscription(GraphNodeArray, self.exploitation_topic, self.cb_exploitation, qos)
        self.create_subscription(GraphNodeArray, self.detection_topic, self.cb_detection, qos)

        self.pub_exploration = self.create_publisher(GraphNodeArray, "exploration_graph_nodes/graph_nodes", qos)
        self.pub_detection = self.create_publisher(GraphNodeArray, "detection_graph_nodes/graph_nodes", qos)
        self.pub_markers = self.create_publisher(MarkerArray, "graph_node_markers", qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.gaussian_model = GaussianWeighting(self.sigma, self.sight_horizon)
        self.fuser = NodeFusion(self.gaussian_model, self.approach_radius,
                                self.exploration_weight, self.gaussian_influence)
        self.marker_factory = MarkerFactory(self)

        self.exploration_nodes, self.exploitation_nodes, self.detection_nodes = [], [], []
        self.visited_nodes = []

        self.create_timer(0.1, self._on_timer)
        self.get_logger().info("GraphNodeFusion initialized with circular Gaussian weighting")

    def cb_exploration(self, msg): self.exploration_nodes = list(msg.nodes)
    def cb_exploitation(self, msg): self.exploitation_nodes = list(msg.nodes)
    def cb_detection(self, msg): self.detection_nodes = list(msg.nodes)

    def _on_timer(self):
        robot_pose = self._get_robot_pose()
        self.visited_nodes = self._update_visited(self.exploration_nodes)

        fused_exp, fused_det = self.fuser.fuse_nodes(
            self.exploration_nodes, self.exploitation_nodes, self.detection_nodes,
            robot_pose, self.visited_nodes
        )

        self.pub_exploration.publish(GraphNodeArray(nodes=fused_exp))
        self.pub_detection.publish(GraphNodeArray(nodes=fused_det))

        gaussian_points = self._generate_gaussian_circle(robot_pose)
        markers = self.marker_factory.create_markers(fused_exp, fused_det, self.visited_nodes, gaussian_points)
        self.pub_markers.publish(markers)

    # --- Simple TF + visited tracker inline (compact version) ---
    def _get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(),
                                                 timeout=Duration(seconds=0.2))
            t = tf.transform.translation
            return Point(x=t.x, y=t.y, z=t.z)
        except Exception:
            return Point()

    def _update_visited(self, nodes):
        pose = self._get_robot_pose()
        visited = [(n.position.x, n.position.y)
                   for n in nodes
                   if math.hypot(n.position.x - pose.x, n.position.y - pose.y) < self.approach_radius]
        self.visited_nodes.extend(visited)
        self.visited_nodes = self.visited_nodes[-200:]
        return self.visited_nodes

    def _generate_gaussian_circle(self, robot_pose: Point, step: float = 0.2):
        """Generate a circular Gaussian field within sight_horizon."""
        xs = np.arange(robot_pose.x - self.sight_horizon, robot_pose.x + self.sight_horizon, step)
        ys = np.arange(robot_pose.y - self.sight_horizon, robot_pose.y + self.sight_horizon, step)
        points = []
        for x in xs:
            for y in ys:
                dx = x - robot_pose.x
                dy = y - robot_pose.y
                d = math.hypot(dx, dy)
                if d > self.sight_horizon:
                    continue
                val = math.exp(-(d ** 2) / (2 * self.sigma ** 2))
                z = val
                points.append((x, y, z, val))
        return points


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
