import rclpy
from rclpy.node import Node
from graph_node_msgs.msg import GraphNode, GraphNodeArray

class GraphNodeFusion(Node):
    def __init__(self):
        super().__init__('graph_node_fusion')
        self.get_logger().info('Graph Node Fusion has been initialized')

        # Subscribers
        self.graph_node_subscriber = self.create_subscription(
            msg_type=GraphNodeArray,
            topic='/semantic_frontier_exploration/graph_nodes',
            callback=self.graph_node_callback,
            qos_profile=10
        )

    def graph_node_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.nodes)} graph nodes')

        # Sorting the nodes based on score
        sorted_nodes = sorted(msg.nodes, key=lambda node: node.score, reverse=True)

        # Log the sorted nodes
        for node in sorted_nodes:
            self.get_logger().info(f'Node ID: {node.id}, Score: {node.score}')