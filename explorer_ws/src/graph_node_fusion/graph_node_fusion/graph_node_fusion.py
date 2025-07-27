import rclpy
from rclpy.node import Node

class GraphNodeFusion(Node):
    def __init__(self):
        super().__init__('graph_node_fusion')
        self.get_logger().info('Graph Node Fusion has been initialized')