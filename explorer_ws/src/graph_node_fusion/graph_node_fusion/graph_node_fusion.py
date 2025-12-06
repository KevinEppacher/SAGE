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
from tf_transformations import euler_from_quaternion
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from nav_msgs.msg import OccupancyGrid

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
        self.last_pose = (0.0, 0.0, 0.0)

    def get_pose(self):
        """Return (x, y, yaw) in map frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame, self.target_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            pose = (t.x, t.y, yaw)
            self.last_pose = pose
            return pose
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

    def publish(self, exploration_nodes, detection_nodes, robot_pose):
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
            for marker in self._make_sphere(n, idx, norm):
                marker_array.markers.append(marker)
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
        base_id = idx * 2  # reserve consecutive IDs for sphere + text

        # --- Sphere marker ---
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = "graph_nodes"
        m.id = base_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = node.position
        m.pose.position.z += 0.05
        m.pose.orientation.w = 1.0
        s = 0.1 + 0.3 * norm(node.score)
        m.scale.x = m.scale.y = m.scale.z = s
        m.color = ColorRGBA(r=r, g=g, b=b, a=a)
        m.lifetime.sec = 1

        # --- Text marker showing score ---
        t = Marker()
        t.header.frame_id = "map"
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.ns = "graph_scores"
        t.id = base_id + 1
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = node.position.x
        t.pose.position.y = node.position.y
        t.pose.position.z = node.position.z + 0.25
        t.scale.z = 0.12
        t.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        t.text = f"{node.score:.2f}"
        t.lifetime.sec = 1

        return [m, t]

    def _make_line(self, robot_pose, node_pose, idx: int):
        rx, ry, _ = robot_pose
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
        text_marker.scale.z = 0.1
        text_marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        text_marker.text = f"{distance:.2f}"
        text_marker.lifetime.sec = 1

        return [m, text_marker]


# ===============================================================
# NodeWeighter: handles scoring and weighting logic
# ===============================================================
class NodeWeighter:
    def __init__(self, node):
        self.node = node
        self.value_map_points = None
        self.costmap = None
        self.costmap_info = None

        # --- Default dynamic weights ---
        self.det_weight = 1.0
        self.map_weight = 1.0
        self.mem_weight = 1.0
        self.mem_sigma = 0.5
        self.proximity_weight = 1.0
        self.proximity_radius = 2.0
        self.source_balance = 0.7
        self.costmap_weight = 1.0
        self.costmap_radius = 0.5  # meters

        range_0_3 = [FloatingPointRange(from_value=0.0, to_value=3.0, step=0.001)]
        range_0_5 = [FloatingPointRange(from_value=0.0, to_value=5.0, step=0.001)]

        # --- Declare dynamic parameters ---
        node.declare_parameter("weights.det_weight", 1.0, ParameterDescriptor(floating_point_range=range_0_3))
        node.declare_parameter("weights.map_weight", 1.0, ParameterDescriptor(floating_point_range=range_0_3))
        node.declare_parameter("weights.mem_weight", 1.0, ParameterDescriptor(floating_point_range=range_0_3))
        node.declare_parameter("weights.mem_sigma", 0.5, ParameterDescriptor(floating_point_range=range_0_5))
        node.declare_parameter("weights.proximity_weight", 1.0, ParameterDescriptor(floating_point_range=range_0_3))
        node.declare_parameter("weights.proximity_radius", 2.0, ParameterDescriptor(floating_point_range=range_0_5))
        node.declare_parameter("weights.source_balance", 0.7, ParameterDescriptor(floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.001)]))
        node.declare_parameter("weights.debug_costmap", True)

        self.debug_costmap = node.get_parameter("weights.debug_costmap").value

        # new parameters for costmap penalty
        node.declare_parameter("weights.costmap_weight", 1.0, ParameterDescriptor(floating_point_range=range_0_3))
        node.declare_parameter("weights.costmap_radius", 0.5, ParameterDescriptor(floating_point_range=range_0_5))

        node.add_on_set_parameters_callback(self._param_callback)

        node.create_subscription(
            PointCloud2,
            "/value_map/value_map_raw",
            self._cb_value_map,
            10
        )

        node.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._cb_costmap,
            10
        )

        self.costmap_marker_pub = node.create_publisher(
            MarkerArray,
            "/graph_node_costmap_penalties",
            10
        )

    # ----------------------------------------------------------------------
    # Dynamic parameter callback
    # ----------------------------------------------------------------------
    def _param_callback(self, params):
        for p in params:
            if p.name == "weights.det_weight":
                self.det_weight = max(0.0, min(1.0, p.value))
                self.node.get_logger().info(f"Updated det_weight: {self.det_weight}")
            elif p.name == "weights.map_weight":
                self.map_weight = max(0.0, min(1.0, p.value))
                self.node.get_logger().info(f"Updated map_weight: {self.map_weight}")
            elif p.name == "weights.mem_weight":
                self.mem_weight = max(0.0, min(1.0, p.value))
                self.node.get_logger().info(f"Updated mem_weight: {self.mem_weight}")
            elif p.name == "weights.mem_sigma":
                self.mem_sigma = max(0.05, float(p.value))
                self.node.get_logger().info(f"Updated mem_sigma: {self.mem_sigma}")
            elif p.name == "weights.proximity_weight":
                self.proximity_weight = max(0.0, float(p.value))
                self.node.get_logger().info(f"Updated proximity_weight: {self.proximity_weight}")
            elif p.name == "weights.proximity_radius":
                self.proximity_radius = max(0.1, float(p.value))
                self.node.get_logger().info(f"Updated proximity_radius: {self.proximity_radius}")
            elif p.name == "weights.source_balance":
                self.source_balance = max(0.0, min(1.0, float(p.value)))
                self.node.get_logger().info(f"Updated source_balance: {self.source_balance}")
            elif p.name == "weights.costmap_weight":
                self.costmap_weight = max(0.0, min(1.0, float(p.value)))
                self.node.get_logger().info(f"Updated costmap_weight: {self.costmap_weight}")
            elif p.name == "weights.costmap_radius":
                self.costmap_radius = max(0.1, float(p.value))
                self.node.get_logger().info(f"Updated costmap_radius: {self.costmap_radius}")
            elif p.name == "weights.debug_costmap":
                self.debug_costmap = bool(p.value)
                self.node.get_logger().info(f"Updated debug_costmap: {self.debug_costmap}")
        res = SetParametersResult()
        res.successful = True
        return res
    
    def _publish_costmap_debug_markers(self, nodes):
        """Visualize costmap penalties for each node as colored spheres."""
        if not self.debug_costmap or not nodes:
            return

        marker_array = MarkerArray()
        t_now = self.node.get_clock().now().to_msg()

        # Clear old markers
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = t_now
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for idx, n in enumerate(nodes):
            penalty = self._get_costmap_penalty(n.position.x, n.position.y)

            # Map penalty to color: red (bad) → blue (good)
            r = 1.0 - penalty
            g = 0.0
            b = penalty
            a = 0.9

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = t_now
            m.ns = "costmap_penalty"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = n.position
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            m.lifetime.sec = 1
            marker_array.markers.append(m)

        self.costmap_marker_pub.publish(marker_array)

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
    # Parse costmap
    # ----------------------------------------------------------------------
    def _cb_costmap(self, msg: OccupancyGrid):
        try:
            self.costmap_info = msg.info
            data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            # Flip because costmap origin is bottom-left
            # self.costmap = np.flipud(data)
            self.costmap = data
        except Exception as e:
            self.node.get_logger().warn(f"Failed to parse costmap: {e}")

    # ----------------------------------------------------------------------
    # Costmap penalty sampling
    # ----------------------------------------------------------------------
    def _get_costmap_penalty(self, x, y):
        if self.costmap is None or self.costmap_info is None:
            return 1.0

        res = self.costmap_info.resolution
        ox = self.costmap_info.origin.position.x
        oy = self.costmap_info.origin.position.y
        gx = int(round((x - ox) / res))
        gy = int(round((y - oy) / res))
        h, w = self.costmap.shape
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return 1.0

        radius_cells = int(self.costmap_radius / res)
        x0, x1 = max(0, gx - radius_cells), min(w, gx + radius_cells)
        y0, y1 = max(0, gy - radius_cells), min(h, gy + radius_cells)
        region = self.costmap[y0:y1, x0:x1]

        if region.size == 0:
            return 1.0

        # Build coordinate grid centered on the node
        ys, xs = np.mgrid[y0:y1, x0:x1]
        dx = (xs - gx) * res
        dy = (ys - gy) * res
        dist2 = dx * dx + dy * dy
        mask = region >= 0  # ignore unknown cells

        if not np.any(mask):
            return 1.0

        weights = np.exp(-0.5 * dist2 / (self.costmap_radius * 0.5) ** 2)
        weights *= mask
        weighted_cost = np.sum(region * weights) / np.sum(weights)
        norm_cost = np.clip(weighted_cost / 100.0, 0.0, 1.0)

        penalty = 1.0 - self.costmap_weight * norm_cost
        penalty = np.clip(penalty, 0.3, 1.0)
        return float(penalty)

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
    def weight_nodes(self, exploration_nodes, exploitation_nodes, detection_nodes, robot_pose):
        fused_exploration, fused_detection = [], []

        for n in exploration_nodes:
            if n.score < 0.1:
                n.score = 0.1

            proximity_factor = self._get_proximity_persistence(
                n.position.x, n.position.y, robot_pose,
                gain=self.proximity_weight, radius=self.proximity_radius
            )

            costmap_penalty = self._get_costmap_penalty(n.position.x, n.position.y)

            n.score *= self.source_balance * proximity_factor * costmap_penalty

        for n in exploitation_nodes:
            n.score *= (1.0 - self.source_balance)

        all_exp = exploration_nodes + exploitation_nodes
        all_exp.sort(key=lambda n: n.score, reverse=True)
        fused_exploration.extend(all_exp)

        for n in detection_nodes:
            s_det = max(0.0, min(1.0, n.score))
            s_map = self._get_score_from_value_map(n.position.x, n.position.y)
            s_mem = self._get_score_from_memory(n.position.x, n.position.y, exploitation_nodes)
            s_fused = 1.0 - (1.0 - self.det_weight * s_det) \
                            * (1.0 - self.map_weight * s_map) \
                            * (1.0 - self.mem_weight * s_mem)
            n.score = min(max(s_fused, 0.0), 1.0)

        fused_detection = sorted(detection_nodes, key=lambda n: n.score, reverse=True)

        if self.debug_costmap:
            # visualize exploration nodes (since those are affected by costmap)
            self._publish_costmap_debug_markers(exploration_nodes)

        return fused_exploration, fused_detection

    # ----------------------------------------------------------------------
    # Directional persistence term (favor front-facing frontiers)
    # ----------------------------------------------------------------------
    def _get_proximity_persistence(self, x_node, y_node, robot_pose, gain=0.3, radius=2.0):
        """
        Returns a weighting factor favoring nodes near the robot.
        gain: maximum boost (0–1)
        radius: influence radius [m]
        """
        x, y, _ = robot_pose
        dist = math.hypot(x_node - x, y_node - y)
        # Gaussian falloff: strong near robot, quickly decreases
        weight = math.exp(-0.5 * (dist / radius) ** 2)
        return 1.0 + gain * weight


# ===============================================================
# Main orchestrator node
# ===============================================================
class GraphNodeFusion(Node):
    def __init__(self):
        super().__init__("graph_node_fusion")

        # --- Parameters ---
        self.declare_parameter("debug_distance", True)
        self.declare_parameter("exploration_topic", "/exploration_graph_nodes/graph_nodes")
        self.declare_parameter("exploitation_topic", "/exploitation_graph_nodes/graph_nodes")
        self.declare_parameter("detection_topic", "/detection_graph_nodes/graph_nodes")
        self.declare_parameter("fused_topics.detection_graph_nodes", "/fused/detection_graph_nodes/graph_nodes")
        self.declare_parameter("fused_topics.exploration_graph_nodes", "/fused/exploration_graph_nodes/graph_nodes")
        self.declare_parameter("timer_period", 0.1)

        gp = self.get_parameter
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
        self.weighter = NodeWeighter(self)

        # --- State ---
        self.exploration_nodes = []
        self.exploitation_nodes = []
        self.detection_nodes = []

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
        robot_pose = self.robot.get_pose()
        if robot_pose is None:
            return

        fused_exp, fused_det = self.weighter.weight_nodes(
            self.exploration_nodes, 
            self.exploitation_nodes, 
            self.detection_nodes,
            robot_pose
        )

        # fused_exp = [n for n in fused_exp if not self._is_visited(n)]
        self._publish_arrays(fused_exp, fused_det)
        self.markers.publish(fused_exp, fused_det, robot_pose)

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