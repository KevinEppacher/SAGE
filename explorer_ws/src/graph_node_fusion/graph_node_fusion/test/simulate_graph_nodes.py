#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from graph_node_msgs.msg import GraphNode, GraphNodeArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt

class SimulateGraphNodes(Node):
    def __init__(self):
        super().__init__('simulate_graph_nodes')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub_exploration_graph_nodes = self.create_publisher(GraphNodeArray, '/test/exploration_graph_nodes/graph_nodes', qos)
        self.pub_detection_graph_nodes = self.create_publisher(GraphNodeArray, '/test/detection_graph_nodes/graph_nodes', qos)

        # Hardcoded nodes
        self.exploration_nodes = [
            # GraphNode(id=1, position=Point(x=1.8, y=-1.2, z=0.0), score=0.9, is_visited=False),
            GraphNode(id=1, position=Point(x=5.0, y=0.0, z=0.0), score=0.9, is_visited=False),
            # GraphNode(id=1, position=Point(x=0.0, y=0.0, z=0.0), score=0.9, is_visited=False),
            # GraphNode(id=2, position=Point(x=3.0, y=-1.0, z=0.0), score=0.7, is_visited=False),
        ]

        self.detection_nodes = [
            # GraphNode(id=1, position=Point(x=3.0, y=0.0, z=0.0), score=0.9, is_visited=False),
            # GraphNode(id=1, position=Point(x=0.0, y=0.0, z=0.0), score=0.9, is_visited=False),
            # GraphNode(id=2, position=Point(x=3.0, y=-1.0, z=0.0), score=0.7, is_visited=False),
        ]

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Simulating static graph nodes at 10 Hz")

    def timer_callback(self):
        msg = GraphNodeArray()
        msg.nodes = self.exploration_nodes
        self.pub_exploration_graph_nodes.publish(msg)

        msg = GraphNodeArray()
        msg.nodes = self.detection_nodes
        self.pub_detection_graph_nodes.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulateGraphNodes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
