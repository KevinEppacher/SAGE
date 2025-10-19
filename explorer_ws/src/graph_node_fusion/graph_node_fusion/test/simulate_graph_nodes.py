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

        self.pub_graph_nodes = self.create_publisher(GraphNodeArray, '/fused/exploration_graph_nodes/graph_nodes', qos)
        self.pub_markers = self.create_publisher(MarkerArray, '/graph_node_markers', qos)

        # Hardcoded nodes
        self.nodes = [
            GraphNode(id=1, position=Point(x=1.0, y=2.0, z=0.0), score=0.9, is_visited=False),
            GraphNode(id=2, position=Point(x=1.0, y=-1.0, z=0.0), score=0.7, is_visited=False),
        ]

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Simulating static graph nodes at 1 Hz")

    def timer_callback(self):
        msg = GraphNodeArray()
        msg.nodes = self.nodes
        self.pub_graph_nodes.publish(msg)
        self.publish_markers(self.nodes)

    def publish_markers(self, nodes):
        marker_array = MarkerArray()

        # Delete all old markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        # Normalize scores for color scaling
        scores = [n.score for n in nodes] or [0.0]
        norm = plt.Normalize(vmin=min(scores), vmax=max(scores))
        cmap = plt.get_cmap("viridis")

        def score_to_color(score):
            r, g, b, a = cmap(norm(score))
            return ColorRGBA(r=r, g=g, b=b, a=a)

        def score_to_scale(score):
            return 0.1 + 0.3 * norm(score)

        idx = 0
        for node in nodes:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "graph_nodes"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = node.position
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = score_to_scale(node.score)
            m.color = score_to_color(node.score)
            m.lifetime.sec = 1
            marker_array.markers.append(m)
            idx += 1

        self.pub_markers.publish(marker_array)


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
