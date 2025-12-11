#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from graph_node_msgs.msg import GraphNodeArray
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

import numpy as np
from scipy.interpolate import Rbf, griddata
from tf2_ros import Buffer, TransformListener
from matplotlib import cm


class ValueMapSurfaceVisualizer(Node):

    def __init__(self):
        super().__init__("value_map_surface_visualizer")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # graph node arrays
        self.graph_x = None
        self.graph_y = None
        self.graph_score = None

        # scale semantic score → height in meters
        self.z_scale = 10.0

        # subscribe
        self.graph_sub = self.create_subscription(
            GraphNodeArray,
            "/fused/exploration_graph_nodes/graph_nodes",
            self.graph_callback,
            10
        )

        # publish
        self.pc_pub = self.create_publisher(PointCloud2,
                                            "/gradient_ascent/value_surface",
                                            10)

        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info("Value surface pointcloud publisher started.")


    # -------------------------------
    # Graph callback
    # -------------------------------
    def graph_callback(self, msg: GraphNodeArray):

        if len(msg.nodes) == 0:
            self.get_logger().warn("Received empty GraphNodeArray")
            return

        xs, ys, scores = [], [], []
        for n in msg.nodes:
            xs.append(n.position.x)
            ys.append(n.position.y)
            scores.append(n.score)

        self.graph_x = np.array(xs)
        self.graph_y = np.array(ys)
        self.graph_score = np.array(scores)


    # -------------------------------
    # Safe plane interpolation
    # -------------------------------
    def compute_surface(self):

        x = self.graph_x
        y = self.graph_y
        z = self.graph_score

        # check for invalid input
        if len(np.unique(x)) < 2 or len(np.unique(y)) < 2:
            self.get_logger().warn("Graph nodes degenerate (all x or all y equal) → using flat plane.")
            grid_x, grid_y = np.mgrid[
                np.min(x):np.max(x):200j,
                np.min(y):np.max(y):200j
            ]
            return grid_x, grid_y, np.zeros_like(grid_x)

        # main RBF
        try:
            rbf = Rbf(x, y, z, function="gaussian", epsilon=1.0)
            grid_x, grid_y = np.mgrid[
                np.min(x):np.max(x):200j,
                np.min(y):np.max(y):200j
            ]
            grid_z = rbf(grid_x, grid_y)

            # if RBF fails (NaN explosion)
            if np.isnan(grid_z).any():
                raise FloatingPointError

        except Exception:
            self.get_logger().warn("RBF failed → using linear griddata fallback.")

            grid_x, grid_y = np.mgrid[
                np.min(x):np.max(x):200j,
                np.min(y):np.max(y):200j
            ]

            grid_z = griddata(
                np.column_stack((x, y)),
                z,
                (grid_x, grid_y),
                method="linear"
            )

            # fill NaN with nearest
            nan_mask = np.isnan(grid_z)
            if nan_mask.any():
                grid_z[nan_mask] = griddata(
                    np.column_stack((x, y)),
                    z,
                    (grid_x[nan_mask], grid_y[nan_mask]),
                    method="nearest"
                )

        return grid_x, grid_y, grid_z


    # -------------------------------
    # RGB packer
    # -------------------------------
    def rgb_to_float(self, rgb):
        r = int(rgb[0] * 255)
        g = int(rgb[1] * 255)
        b = int(rgb[2] * 255)
        rgb_int = (r << 16) | (g << 8) | b
        return np.float32(rgb_int).view(np.float32)


    # -------------------------------
    # MAIN UPDATE LOOP
    # -------------------------------
    def update(self):

        if self.graph_x is None:
            return

        grid_x, grid_y, grid_z_raw = self.compute_surface()

        # convert score → height in meters
        grid_z = grid_z_raw * self.z_scale

        # flatten for PCL
        X = grid_x.flatten()
        Y = grid_y.flatten()
        Z = grid_z.flatten()

        # color based on raw score
        S = grid_z_raw.flatten()
        cmap = cm.get_cmap("viridis")

        points = []
        for x, y, z, s in zip(X, Y, Z, S):
            s_clamped = float(np.clip(s, 0.0, 1.0))
            rgb = cmap(s_clamped)
            rgb_packed = self.rgb_to_float(rgb)
            points.append([float(x), float(y), float(z), rgb_packed])

        # prepare PCL
        header = point_cloud2.Header()
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()

        cloud = point_cloud2.create_cloud(
            header,
            fields=[
                PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            points=points
        )

        self.pc_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ValueMapSurfaceVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
