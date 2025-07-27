import rclpy
from graph_node_fusion.graph_node_fusion import GraphNodeFusion

def main(args=None):
    rclpy.init(args=args)
    node = GraphNodeFusion()
    rclpy.spin(node)
    rclpy.shutdown()
