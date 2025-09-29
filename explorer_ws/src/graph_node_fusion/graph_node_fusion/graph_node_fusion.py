import rclpy
from rclpy.node import Node
from graph_node_msgs.msg import GraphNode, GraphNodeArray
from geometry_msgs.msg import PoseStamped
# from graph_node_fusion.graph_node_fusion import BLUE, GREEN, RED, YELLOW, BOLD, RESET

# ANSI color codes
BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"

class GraphNodeFusion(Node):
    def __init__(self):
        super().__init__('graph_node_fusion')
        self.get_logger().info('Graph Node Fusion has been initialized')

        self._declare_params()
        self._read_params()

        # Subscribers
        self.exploration_graph_node_sub = self.create_subscription(
            msg_type=GraphNodeArray,
            topic='/exploration_graph_nodes/graph_nodes',
            callback=self.exploration_graph_node_callback,
            qos_profile=10
        )

        self.exploitation_graph_node_sub = self.create_subscription(
            msg_type=GraphNodeArray,
            topic='/exploitation_graph_nodes/graph_nodes',
            callback=self.exploitation_graph_node_callback,
            qos_profile=10
        )

        self.detection_graph_node_sub = self.create_subscription(
            msg_type=GraphNodeArray,
            topic='/detection_graph_nodes/graph_nodes',
            callback=self.detection_graph_node_callback,
            qos_profile=10
        )

        # Publishers
        self.goal_pose_publisher = self.create_publisher(
            msg_type=PoseStamped,
            topic='goal_pose',
            qos_profile=10
        )

        # Timers
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Member Variables
        self.current_goal = None
        self.fused_graph_nodes = []
        self.exploration_graph_nodes = []
        self.exploitation_graph_nodes = []
        self.detection_graph_nodes = []

    def exploration_graph_node_callback(self, msg):

        # Sorting the nodes based on score
        sorted_nodes = sorted(msg.nodes, key=lambda node: node.score, reverse=True)
        self.exploration_graph_nodes.extend(sorted_nodes)

    def exploitation_graph_node_callback(self, msg):

        # Sorting the nodes based on score
        sorted_nodes = sorted(msg.nodes, key=lambda node: node.score, reverse=True)
        self.exploitation_graph_nodes.extend(sorted_nodes)

    def detection_graph_node_callback(self, msg):

        # Sorting the nodes based on score
        sorted_nodes = sorted(msg.nodes, key=lambda node: node.score, reverse=True)
        self.detection_graph_nodes.extend(sorted_nodes)

    def timer_callback(self):
        pass

    def _declare_params(self):
        self.declare_parameter('exploration_weight', 0.8)

    def _read_params(self):
        p = self.get_parameter
        self.exploration_weight = p('exploration_weight').get_parameter_value().double_value

    def update_weights(self):
        self.exploitation_weight = 1.0 - self.exploration_weight

        self.exploration_graph_nodes.clear()
        for node in self.exploration_graph_nodes:
            node.score *= self.exploration_weight
            self.exploration_graph_nodes.append(node)
        
        self.exploitation_graph_nodes.clear()
        for node in self.exploitation_graph_nodes:
            node.score *= self.exploitation_weight
            self.exploitation_graph_nodes.append(node)