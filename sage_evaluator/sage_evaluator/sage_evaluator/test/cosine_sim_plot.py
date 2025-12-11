#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from tf2_ros import Buffer, TransformListener


class GradientAscentVisualizer(Node):

    def __init__(self):
        super().__init__("gradient_ascent_visualizer")

        # TF Buffer + Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.score_sub = self.create_subscription(
            Float32,
            "/value_map/cosine_similarity",
            self.score_callback,
            10
        )

        # Data storage
        self.raw_scores = []
        self.scores = []   # smoothed scores
        self.xs = []
        self.ys = []

        # Smoothing length
        self.smooth_window = 10

        # Matplotlib setup
        plt.ion()
        self.fig = plt.figure(figsize=(10, 6))
        self.ax3d = self.fig.add_subplot(121, projection='3d')
        self.ax2d = self.fig.add_subplot(122)

        # Create persistent line artists
        self.line3d, = self.ax3d.plot([], [], [], color='blue', linewidth=2)
        self.line2d, = self.ax2d.plot([], [], color='red')

        # Labels always stay
        self.ax3d.set_xlabel("X [m]")
        self.ax3d.set_ylabel("Y [m]")
        self.ax3d.set_zlabel("Cosine Similarity")
        self.ax3d.set_title("Semantic Gradient-Ascent Trajectory")

        self.ax2d.set_xlabel("Timestep")
        self.ax2d.set_ylabel("Cosine Similarity")
        self.ax2d.set_title("Semantic Reward Over Time")

        # Timer for plot updates
        self.timer = self.create_timer(0.3, self.update_plot)
        self.get_logger().info("Gradient Ascent Visualizer started.")

        self.current_score = None


    # --------------------------
    # SMOOTHING FUNCTION (correct)
    # --------------------------
    def smooth(self, data):
        n = len(data)
        if n < self.smooth_window:
            return np.array(data)

        kernel = np.ones(self.smooth_window) / self.smooth_window
        smoothed = np.convolve(data, kernel, mode='valid')

        pad_length = n - len(smoothed)
        pad = np.array(data[:pad_length])

        return np.concatenate([pad, smoothed])


    # --------------------------
    # CALLBACKS
    # --------------------------
    def score_callback(self, msg: Float32):
        self.current_score = max(0.0, min(1.0, float(msg.data)))
        self.record_tf_pose()


    def record_tf_pose(self):
        if self.current_score is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())

            x = tf.transform.translation.x
            y = tf.transform.translation.y

            self.xs.append(x)
            self.ys.append(y)

            # add raw score
            self.raw_scores.append(self.current_score)

            # recompute smoothed
            self.scores = self.smooth(self.raw_scores)

        except Exception:
            pass


    # --------------------------
    # PLOT UPDATING (with proper autoscaling)
    # --------------------------
    def update_plot(self):
        if len(self.scores) < 2:
            return

        xs = np.array(self.xs)
        ys = np.array(self.ys)
        zs = np.array(self.scores)

        # --- Update 3D line ---
        self.line3d.set_data(xs, ys)
        self.line3d.set_3d_properties(zs)

        # IMPORTANT: dynamic autoscale for 3D
        self.ax3d.set_xlim(np.min(xs), np.max(xs))
        self.ax3d.set_ylim(np.min(ys), np.max(ys))
        self.ax3d.set_zlim(np.min(zs), np.max(zs) + 0.05)

        # --- Update 2D plot ---
        self.line2d.set_xdata(np.arange(len(zs)))
        self.line2d.set_ydata(zs)

        self.ax2d.relim()
        self.ax2d.autoscale_view()

        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = GradientAscentVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
