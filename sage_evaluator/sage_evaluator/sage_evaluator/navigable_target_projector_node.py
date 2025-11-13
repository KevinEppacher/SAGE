#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math


class NavigableTargetProjector(Node):
    """Find nearest reachable points for semantic centroids using OccupancyGrid."""

    def __init__(self):
        super().__init__('navigable_target_projector')

        # Subscribers
        self.create_subscription(PoseArray, '/evaluator/semantic_centroid_targets',
                                 self._centroid_cb, 10)
        self.create_subscription(OccupancyGrid, '/evaluator/global_costmap/costmap',
                                 self._costmap_cb, 10)

        # Publisher
        self.navigable_pub = self.create_publisher(PoseArray, '/evaluator/navigable_targets', 10)

        # Internal state
        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_origin = None
        self.map_resolution = None

        self.get_logger().info('📡 NavigableTargetProjector initialized (OccupancyGrid mode).')

    # ------------------------------------------------------------------
    def _costmap_cb(self, msg: OccupancyGrid):
        """Receive OccupancyGrid and store as NumPy array."""
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(self.map_height, self.map_width)
        self.get_logger().debug(f'Updated costmap: {self.map_width}x{self.map_height}')

    # ------------------------------------------------------------------
    def _centroid_cb(self, msg: PoseArray):
        if self.map_data is None:
            self.get_logger().warn('No costmap received yet.')
            return
        if not msg.poses:
            return

        navigable_poses = PoseArray()
        navigable_poses.header = msg.header

        for i, centroid in enumerate(msg.poses):
            cx = centroid.position.x
            cy = centroid.position.y
            nav_point = self._find_nearest_free_point((cx, cy))

            if nav_point is not None:
                pose = Pose()
                pose.position.x = nav_point[0]
                pose.position.y = nav_point[1]
                pose.orientation = Quaternion(w=1.0)
                navigable_poses.poses.append(pose)
            else:
                self.get_logger().warn(f'No navigable point found for centroid {i}.')

        if navigable_poses.poses:
            self.navigable_pub.publish(navigable_poses)
            self.get_logger().info(f'✅ Published {len(navigable_poses.poses)} navigable targets.')

    # ------------------------------------------------------------------
    def _find_nearest_free_point(self, centroid, max_radius=2.0, step=0.05, n_rays=36):
        cx, cy = centroid
        best_point = None
        best_dist = float('inf')

        for angle in np.linspace(0, 2 * math.pi, n_rays):
            for r in np.arange(0, max_radius, step):
                x = cx + r * math.cos(angle)
                y = cy + r * math.sin(angle)
                if self._is_free(x, y):
                    dist = math.hypot(x - cx, y - cy)
                    if dist < best_dist:
                        best_dist = dist
                        best_point = (x, y)
                    break
        return best_point

    # ------------------------------------------------------------------
    def _is_free(self, x, y, free_thresh=50):
        if self.map_data is None:
            return False

        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)

        if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
            return False

        cost = self.map_data[my, mx]
        return 10 < cost < free_thresh


def main(args=None):
    rclpy.init(args=args)
    node = NavigableTargetProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
