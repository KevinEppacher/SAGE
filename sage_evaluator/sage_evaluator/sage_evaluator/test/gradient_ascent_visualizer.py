#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from graph_node_msgs.msg import GraphNodeArray

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata

from tf2_ros import Buffer, TransformListener
from matplotlib import cm


class ValueMapSurfaceVisualizer(Node):

    def __init__(self):
        super().__init__("value_map_surface_visualizer")

        # TF for robot tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Storage
        self.graph_x = None
        self.graph_y = None
        self.graph_score = None

        self.robot_x = None
        self.robot_y = None

        # Subscribers
        self.graph_sub = self.create_subscription(
            GraphNodeArray,
            "/fused/exploration_graph_nodes/graph_nodes",
            self.graph_callback,
            10
        )

        # Matplotlib setup
        plt.ion()
        self.fig = plt.figure(figsize=(10, 7))
        self.ax = self.fig.add_subplot(111, projection="3d")

        # No lines yet—the plot will create them after first update
        self.surface = None
        self.robot_marker = None

        self.timer = self.create_timer(0.01, self.update_plot)
        self.get_logger().info("3D Value Surface Visualizer started.")


    # -------------------------------
    # CALLBACKS
    # -------------------------------
    def graph_callback(self, msg: GraphNodeArray):
        xs = []
        ys = []
        scores = []

        for n in msg.nodes:
            xs.append(n.position.x)
            ys.append(n.position.y)
            scores.append(n.score)

        self.graph_x = np.array(xs)
        self.graph_y = np.array(ys)
        self.graph_score = np.array(scores)

        # Get robot position when graph nodes update
        self.update_robot_pose()


    def update_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            self.robot_x = tf.transform.translation.x
            self.robot_y = tf.transform.translation.y
        except Exception:
            pass


    # -------------------------------
    # VISUALIZATION
    # -------------------------------
    def update_plot(self):

        if self.graph_x is None:
            return

        self.ax.clear()

        # --- 1. Create landscape surface ---
        grid_x, grid_y = np.mgrid[
            np.min(self.graph_x):np.max(self.graph_x):200j,
            np.min(self.graph_y):np.max(self.graph_y):200j,
        ]

        grid_z = griddata(
            (self.graph_x, self.graph_y),
            self.graph_score,
            (grid_x, grid_y),
            method="cubic"
        )

        self.ax.plot_surface(
            grid_x,
            grid_y,
            grid_z,
            cmap="viridis",
            alpha=0.7,
            linewidth=0,
            antialiased=True
        )

        # --- 2. Draw graph nodes as round markers ---
        for x, y, s in zip(self.graph_x, self.graph_y, self.graph_score):

            self.ax.scatter3D(
                x,
                y,
                s + 0.01,
                s=100,        # size scales with score
                color=cm.viridis(s),
                alpha=1.0,
                depthshade=True       # gives spherical shading
            )

        # --- 3. Draw robot marker ---
        if self.robot_x is not None:
            self.ax.scatter3D(
                self.robot_x,
                self.robot_y,
                np.max(self.graph_score) + 0.02,
                s=300,
                color="red",
                alpha=1.0,
                depthshade=True
            )

        # --- Labels ---
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_zlabel("Semantic Score")
        self.ax.set_title("Graph Nodes + Robot on Semantic Value Landscape")

        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = ValueMapSurfaceVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
