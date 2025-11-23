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
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import SetParametersResult

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

        # --- Clear all markers ---
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = self.node.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        # --- Score normalization for color ---
        all_scores = [n.score for n in exploration_nodes + detection_nodes] or [0.0]
        norm = plt.Normalize(vmin=min(all_scores), vmax=max(all_scores))

        idx = 0

        # --- Spheres and score texts ---
        for n in exploration_nodes + detection_nodes:
            sphere = self._make_sphere(n, idx, norm)
            marker_array.markers.append(sphere)
            idx += 1

            score_text = self._make_score_text(n, idx, norm)
            marker_array.markers.append(score_text)
            idx += 1

        # --- Visited nodes ---
        for vx, vy in visited_nodes[-200:]:
            marker_array.markers.append(self._make_visited(vx, vy, idx))
            idx += 1

        # --- Ropes and distance texts ---
        if self.debug_distance:
            for n in exploration_nodes:
                rope = self._make_line(robot_pose, n, idx, norm)
                marker_array.markers.append(rope)
                idx += 1

                dist_text = self._make_distance_text(robot_pose, n, idx)
                marker_array.markers.append(dist_text)
                idx += 1

        self.publisher.publish(marker_array)

    # -------------------------------------------------------------
    # Spheres for nodes
    # -------------------------------------------------------------
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

    # -------------------------------------------------------------
    # Score text above each node
    # -------------------------------------------------------------
    def _make_score_text(self, node: GraphNode, idx: int, norm):
        r, g, b, _ = self.cmap(norm(node.score))

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = "score_text"
        m.id = idx
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = node.position.x
        m.pose.position.y = node.position.y
        m.pose.position.z = 0.35   # above the sphere

        m.scale.z = 0.12           # text height
        m.color = ColorRGBA(r=r, g=g, b=b, a=1.0)

        m.text = f"{node.score:.2f}"
        m.lifetime.sec = 1
        return m

    # -------------------------------------------------------------
    # Ropes (colored & thickness from score)
    # -------------------------------------------------------------
    def _make_line(self, robot_pose, node, idx, norm):
        rx, ry = robot_pose
        nx, ny = node.position.x, node.position.y
        r, g, b, a = self.cmap(norm(node.score))

        thickness = 0.01 + 0.05 * norm(node.score)

        from geometry_msgs.msg import Point

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = "rope_lines"
        m.id = idx
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        m.scale.x = thickness
        m.color = ColorRGBA(r=r, g=g, b=b, a=max(0.4, a))

        p1 = Point(x=rx, y=ry, z=0.05)
        p2 = Point(x=nx, y=ny, z=0.05)
        m.points = [p1, p2]
        m.lifetime.sec = 1
        return m

    # -------------------------------------------------------------
    # Distance text near the midpoint of the rope
    # -------------------------------------------------------------
    def _make_distance_text(self, robot_pose, node, idx):
        rx, ry = robot_pose
        nx, ny = node.position.x, node.position.y
        dist = math.hypot(nx - rx, ny - ry)

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = "distance_text"
        m.id = idx
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = (rx + nx) * 0.5
        m.pose.position.y = (ry + ny) * 0.5
        m.pose.position.z = 0.25

        m.scale.z = 0.12
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)

        m.text = f"{dist:.2f} m"
        m.lifetime.sec = 1
        return m

    # -------------------------------------------------------------
    # Visited nodes
    # -------------------------------------------------------------
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

# ===============================================================
# NodeWeighter: handles scoring and weighting logic
# ===============================================================
class NodeWeighter:
    def __init__(self, node, exploration_weight: float = 0.7):
        self.node = node
        self.exploration_weight = exploration_weight
        self.value_map_points = None  # numpy array (x, y, intensity)

        # --- Dynamic parameters ---
        self.det_weight = 1.0
        self.map_weight = 1.0
        self.mem_weight = 1.0
        self.mem_sigma = 0.5  # spatial falloff [m]

        node.declare_parameter("weights.det_weight", 1.0)
        node.declare_parameter("weights.map_weight", 1.0)
        node.declare_parameter("weights.mem_weight", 1.0)
        node.declare_parameter("weights.mem_sigma", 0.5)
        node.declare_parameter("weights.proximity_weight", 1.0)

        self.det_weight = node.get_parameter("weights.det_weight").value
        self.map_weight = node.get_parameter("weights.map_weight").value
        self.mem_weight = node.get_parameter("weights.mem_weight").value
        self.mem_sigma = node.get_parameter("weights.mem_sigma").value
        self.proximity_weight = node.get_parameter("weights.proximity_weight").value

        self.det_weight = max(0.0, min(1.0, self.det_weight))
        self.map_weight = max(0.0, min(1.0, self.map_weight))
        self.mem_weight = max(0.0, min(1.0, self.mem_weight))
        self.mem_sigma = max(0.05, float(self.mem_sigma))
        self.proximity_weight = max(0.0, min(1.0, self.proximity_weight))

        self.node.get_logger().info(
            f"NodeWeighter initialized with \n det_weight={self.det_weight}, \n "
            f"map_weight={self.map_weight} \n, mem_weight={self.mem_weight} \n, "
            f"mem_sigma={self.mem_sigma} \n, proximity_weight={self.proximity_weight} \n"
        )

         # Register parameter change callback

        node.add_on_set_parameters_callback(self._param_callback)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribe to the value map
        node.create_subscription(
            PointCloud2,
            "/value_map/value_map_raw",
            self._cb_value_map,
            qos,
        )

    # ----------------------------------------------------------------------
    # Dynamic parameter callback
    # ----------------------------------------------------------------------
    def _param_callback(self, params):
        for p in params:
            if p.name == "weights.det_weight":
                self.det_weight = max(0.0, min(1.0, p.value))
            elif p.name == "weights.map_weight":
                self.map_weight = max(0.0, min(1.0, p.value))
            elif p.name == "weights.mem_weight":
                self.mem_weight = max(0.0, min(1.0, p.value))
            elif p.name == "weights.mem_sigma":
                self.mem_sigma = max(0.05, float(p.value))
            elif p.name == "weights.proximity_weight":
                self.proximity_weight = max(0.0, min(1.0, p.value))
        res = SetParametersResult()
        res.successful = True
        return res

    # ----------------------------------------------------------------------
    # Parse PointCloud2 manually (no ros_numpy)
    # ----------------------------------------------------------------------
    def _cb_value_map(self, msg: PointCloud2):
        try:
            fmt = "<ffff"  # x, y, z, intensity
            step = msg.point_step
            npts = msg.width * msg.height
            pts = []
            for i in range(npts):
                x, y, z, inten = struct.unpack_from(fmt, msg.data, i * step)
                if inten > 0.0:
                    pts.append((x, y, inten))
            if pts:
                self.value_map_points = np.array(pts, dtype=np.float32)
        except Exception as e:
            self.node.get_logger().warn(f"Failed to parse /value_map/value_map_raw: {e}")

    # ----------------------------------------------------------------------
    # Mean intensity around (x, y)
    # ----------------------------------------------------------------------
    def _get_score_from_value_map(self, x, y, radius=0.3):
        if self.value_map_points is None or len(self.value_map_points) == 0:
            return 0.0
        pts = self.value_map_points
        dx = pts[:, 0] - x
        dy = pts[:, 1] - y
        mask = (dx*dx + dy*dy) < radius**2
        if not np.any(mask):
            return 0.0
        return float(np.mean(pts[mask, 2]))

    # ----------------------------------------------------------------------
    # Memory relevance = spatial proximity to exploitation nodes
    # ----------------------------------------------------------------------
    def _get_score_from_memory(self, x, y, exploitation_nodes):
        if not exploitation_nodes:
            return 0.0
        sigma2 = self.mem_sigma * self.mem_sigma
        num, denom = 0.0, 0.0
        for m in exploitation_nodes:
            dx = x - m.position.x
            dy = y - m.position.y
            d2 = dx*dx + dy*dy
            w = math.exp(-d2 / (2.0 * sigma2))
            num += m.score * w
            denom += w
        return num / denom if denom > 0 else 0.0

    # ----------------------------------------------------------------------
    # Weighted Noisy-OR fusion (detector + value map + memory)
    # ----------------------------------------------------------------------
    def weight_nodes(self, exploration_nodes, exploitation_nodes, detection_nodes, robot_pose=None):
        fused_exploration, fused_detection = [], []

        # --- Exploration vs exploitation weighting ---
        for n in exploration_nodes:
            n.score *= self.exploration_weight
        for n in exploitation_nodes:
            n.score *= (1.0 - self.exploration_weight)

        if robot_pose is not None:
            for n in exploration_nodes:
                pr = self._proximity_reward(n, robot_pose)
                n.score = (1 - self.proximity_weight) * n.score + self.proximity_weight * (n.score * pr)

        all_exp = exploration_nodes + exploitation_nodes
        all_exp.sort(key=lambda n: n.score, reverse=True)
        fused_exploration.extend(all_exp)

        # --- Fuse detections using semantic + memory priors ---
        for n in detection_nodes:
            s_det = max(0.0, min(1.0, n.score))
            s_map = self._get_score_from_value_map(n.position.x, n.position.y)
            s_mem = self._get_score_from_memory(n.position.x, n.position.y, exploitation_nodes)

            s_fused = 1.0 - (1.0 - self.det_weight * s_det) \
                            * (1.0 - self.map_weight * s_map) \
                            * (1.0 - self.mem_weight * s_mem)

            n.score = min(max(s_fused, 0.0), 1.0)

        fused_detection = sorted(detection_nodes, key=lambda n: n.score, reverse=True)
        return fused_exploration, fused_detection

    def _proximity_reward(self, node, robot_pos, k=0.15):
        x, y = node.position.x, node.position.y
        rx, ry = robot_pos
        d = math.hypot(x - rx, y - ry)
        return 1.0 / (1.0 + k * d)

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
        self.declare_parameter("fused_topics.detection_graph_nodes", "/fused/detection_graph_nodes/graph_nodes")
        self.declare_parameter("fused_topics.exploration_graph_nodes", "/fused/exploration_graph_nodes/graph_nodes")
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
        self.pub_fused_detection_topic = gp("fused_topics.detection_graph_nodes").value
        self.pub_fused_exploration_topic = gp("fused_topics.exploration_graph_nodes").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Subscriptions and publishers ---
        self.create_subscription(GraphNodeArray, self.exploration_topic, self.cb_exploration, qos)
        self.create_subscription(GraphNodeArray, self.exploitation_topic, self.cb_exploitation, qos)
        self.create_subscription(GraphNodeArray, self.detection_topic, self.cb_detection, qos)

        self.pub_exploration = self.create_publisher(GraphNodeArray, self.pub_fused_exploration_topic, qos)
        self.pub_detection = self.create_publisher(GraphNodeArray, self.pub_fused_detection_topic, qos)

        # --- Modules ---
        self.robot = Robot(self)
        self.markers = Markers(self, debug_distance=self.debug_distance)
        self.weighter = NodeWeighter(self, self.exploration_weight)

        # --- State ---
        self.exploration_nodes = []
        self.exploitation_nodes = []
        self.detection_nodes = []
        self.visited_nodes = []

        self.create_timer(self.timer_period, self._loop)
        self.last_loop_time = self.get_clock().now()

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
            self.exploration_nodes, 
            self.exploitation_nodes, 
            self.detection_nodes,
            robot_pose
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
        # --- Exploration nodes ---
        msg_ex = GraphNodeArray()
        msg_ex.header.frame_id = "map"
        msg_ex.header.stamp = self.get_clock().now().to_msg()
        msg_ex.nodes = exploration_nodes
        self.pub_exploration.publish(msg_ex)

        # --- Detection nodes ---
        msg_det = GraphNodeArray()
        msg_det.header.frame_id = "map"
        msg_det.header.stamp = self.get_clock().now().to_msg()
        for detection_node in detection_nodes:
            node = GraphNode()  # ensure fresh instance
            node.position.x = detection_node.position.x
            node.position.y = detection_node.position.y
            node.position.z = 0.0
            node.score = detection_node.score
            node.id = detection_node.id
            msg_det.nodes.append(node)
        self.pub_detection.publish(msg_det)

        # Debug info
        self.get_logger().debug(
            f"Published {len(exploration_nodes)} exploration nodes and {len(detection_nodes)} detection nodes."
        )


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
