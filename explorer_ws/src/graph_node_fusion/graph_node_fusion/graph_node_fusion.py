#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from graph_node_msgs.msg import GraphNodeArray

BOLD="\033[1m"; GREEN="\033[92m"; RESET="\033[0m"

class GraphNodeFusion(Node):
    def __init__(self):
        super().__init__('graph_node_fusion')
        self._declare_params(); self._read_params()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subscriptions
        self.create_subscription(GraphNodeArray, '/exploration_graph_nodes/graph_nodes',
                                 self.cb_exploration, qos)
        self.create_subscription(GraphNodeArray, '/exploitation_graph_nodes/graph_nodes',
                                 self.cb_exploitation, qos)
        self.create_subscription(GraphNodeArray, '/detection_graph_nodes/graph_nodes',
                                 self.cb_detection, qos)

        # Publisher
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', qos)

        # State
        self.exploration_nodes = []
        self.exploitation_nodes = []
        self.detection_nodes = []
        self.current_goal = None
        self.object_found = False
        self.get_logger().info('Graph Node Fusion initialized')

        self.create_timer(self.plan_rate, self.timer_cb)

    # --- Callbacks ---
    def cb_exploration(self, msg): self.exploration_nodes = sorted(list(msg.nodes), key=lambda n: n.score, reverse=True)
    def cb_exploitation(self, msg): self.exploitation_nodes = sorted(list(msg.nodes), key=lambda n: n.score, reverse=True)
    def cb_detection(self, msg): self.detection_nodes = sorted(list(msg.nodes), key=lambda n: n.score, reverse=True)

    # --- Planning ---
    def timer_cb(self):
        self._read_params()

        # 1) Detection gating: if found, keep navigating to the LAST goal (no goal changes).
        top_det = self.detection_nodes[0] if self.detection_nodes else None
        if top_det and top_det.score >= self.detection_found_threshold:
            if not self.object_found:
                self.object_found = True
                # If no goal was ever sent, publish one now and then latch.
                if self.current_goal is None:
                    chosen, src, wscore = self._choose_best_no_distance()
                    if chosen:
                        self._publish_goal(chosen)
                        self.get_logger().info(
                            f"{BOLD}{GREEN}Object FOUND{RESET} (score={top_det.score:.3f}). "
                            f"Latching current goal from {src} raw={chosen.score:.3f} weighted={wscore:.3f}")
                else:
                    self.get_logger().info(
                        f"{BOLD}{GREEN}Object FOUND{RESET} (score={top_det.score:.3f}). "
                        f"Continue to last goal (latched).")
            return  # do not change goal while found

        # Not found -> free to select best goal
        if self.object_found:
            self.object_found = False  # reset latch

        chosen, src, wscore = self._choose_best_no_distance()
        if not chosen:
            self.get_logger().warn('No exploration or exploitation nodes available.')
            return

        if not self._same_goal(chosen):
            self._publish_goal(chosen)
            self.get_logger().info(
                f"{BOLD}{GREEN}New goal ->{RESET} src={src} raw={chosen.score:.3f} weighted={wscore:.3f}")

    # --- Helpers ---
    def _choose_best_no_distance(self):
        best_exp = self.exploration_nodes[0] if self.exploration_nodes else None
        best_expl = self.exploitation_nodes[0] if self.exploitation_nodes else None
        if not best_exp and not best_expl:
            return None, None, None
        w_exp = self.exploration_weight
        w_expl = 1.0 - self.exploration_weight
        score_exp = (best_exp.score * w_exp) if best_exp else -1.0
        score_expl = (best_expl.score * w_expl) if best_expl else -1.0
        if score_exp >= score_expl:
            return best_exp, 'exploration', score_exp
        else:
            return best_expl, 'exploitation', score_expl

    def _publish_goal(self, node):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map_frame
        goal.pose.position = node.position
        goal.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.goal_pub.publish(goal)
        self.current_goal = goal

    def _same_goal(self, node) -> bool:
        if self.current_goal is None or node is None:
            return False
        cg = self.current_goal.pose.position
        ng = node.position
        tol = self.goal_change_tolerance_m
        return abs(cg.x - ng.x) < tol and abs(cg.y - ng.y) < tol and abs(cg.z - ng.z) < tol

    # --- Params ---
    def _declare_params(self):
        self.declare_parameter('exploration_weight', 0.7)            # 0..1
        self.declare_parameter('detection_found_threshold', 0.50)    # detector threshold
        self.declare_parameter('goal_change_tolerance_m', 0.10)
        self.declare_parameter('plan_rate', 1.0)
        self.declare_parameter('robot.map_frame', 'map')

    def _read_params(self):
        gp = self.get_parameter
        self.exploration_weight = gp('exploration_weight').get_parameter_value().double_value
        self.detection_found_threshold = gp('detection_found_threshold').get_parameter_value().double_value
        self.goal_change_tolerance_m = gp('goal_change_tolerance_m').get_parameter_value().double_value
        self.plan_rate = gp('plan_rate').get_parameter_value().double_value
        self.map_frame = gp('robot.map_frame').get_parameter_value().string_value or 'map'
