import rclpy
from rclpy.node import Node
from graph_node_msgs.msg import GraphNode, GraphNodeArray
from geometry_msgs.msg import PoseStamped

class GraphNodeFusion(Node):
    def __init__(self):
        super().__init__('graph_node_fusion')
        self.get_logger().info('Graph Node Fusion has been initialized')

        # Subscribers
        self.exploration_graph_node_sub = self.create_subscription(
            msg_type=GraphNodeArray,
            topic='/semantic_frontier_exploration/graph_nodes',
            callback=self.exploration_graph_node_callback,
            qos_profile=10
        )

        self.exploitation_graph_node_sub = self.create_subscription(
            msg_type=GraphNodeArray,
            topic='/semantic_frontier_exploration/graph_nodes',
            callback=self.exploitation_graph_node_callback,
            qos_profile=10
        )

        # Publishers
        self.goal_pose_publisher = self.create_publisher(
            msg_type=PoseStamped,
            topic='/semantic_frontier_exploration/goal_pose',
            qos_profile=10
        )

        # Timers
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Member Variables
        self.current_goal = None
        self.fused_graph_nodes = []
        self.exploration_graph_nodes = []
        self.exploitation_graph_nodes = []

    def exploration_graph_node_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.nodes)} graph nodes')

        # Sorting the nodes based on score
        sorted_nodes = sorted(msg.nodes, key=lambda node: node.score, reverse=True)

        # Log the sorted nodes
        for node in sorted_nodes:
            self.get_logger().info(f'Node ID: {node.id}, Score: {node.score}')

    def exploitation_graph_node_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.nodes)} graph nodes')

        # Sorting the nodes based on score
        sorted_nodes = sorted(msg.nodes, key=lambda node: node.score, reverse=True)

        # Log the sorted nodes
        for node in sorted_nodes:
            self.get_logger().info(f'Node ID: {node.id}, Score: {node.score}')

    def timer_callback(self):
        self.get_logger().info('Timer triggered')