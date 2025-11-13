#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math


class NavigableTargetProjector(Node):
    """Structured radial sampling around target centroids, mirroring ApproachPoseAdjustor logic."""

    def __init__(self):
        super().__init__("navigable_target_projector")

        # ---------------- Parameters ----------------
        self.declare_parameter("robot_radius", 0.35)
        self.declare_parameter("cost_threshold", 50)
        self.declare_parameter("allow_unknown", False)
        self.declare_parameter("angular_step_deg", 10.0)
        self.declare_parameter("radial_samples", 6)
        self.declare_parameter("line_step", 0.05)
        self.declare_parameter("search_radius", 2.5)

        self.robot_radius = self.get_parameter("robot_radius").value
        self.cost_threshold = self.get_parameter("cost_threshold").value
        self.allow_unknown = self.get_parameter("allow_unknown").value
        self.angular_step_deg = self.get_parameter("angular_step_deg").value
        self.radial_samples = self.get_parameter("radial_samples").value
        self.line_step = self.get_parameter("line_step").value
        self.search_radius = self.get_parameter("search_radius").value

        # ---------------- Subscribers / Publishers ----------------
        self.create_subscription(PoseArray, "/evaluator/semantic_centroid_targets", self._centroid_cb, 10)
        self.create_subscription(OccupancyGrid, "/evaluator/global_costmap/costmap", self._costmap_cb, 10)
        self.pub = self.create_publisher(PoseArray, "/evaluator/navigable_targets", 10)

        # ---------------- Internal State ----------------
        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin = None

        self.get_logger().info("📡 NavigableTargetProjector initialized with structured sampling.")

    # ------------------------------------------------------------------
    def _costmap_cb(self, msg: OccupancyGrid):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(self.map_height, self.map_width)

    # ------------------------------------------------------------------
    def _centroid_cb(self, msg: PoseArray):
        if self.map_data is None:
            self.get_logger().warn("No costmap received yet.")
            return

        reachable_targets = PoseArray()
        reachable_targets.header = msg.header

        for centroid in msg.poses:
            target = (centroid.position.x, centroid.position.y)
            reachable = self.find_reachable_point(target)
            if reachable is not None:
                pose = Pose()
                pose.position.x, pose.position.y = reachable
                pose.orientation = Quaternion(w=1.0)
                reachable_targets.poses.append(pose)

        if reachable_targets.poses:
            self.pub.publish(reachable_targets)
            self.get_logger().info(f" Published {len(reachable_targets.poses)} reachable targets.")

    # ------------------------------------------------------------------
    def find_reachable_point(self, target):
        """Sample concentric rings and angles to find a reachable approach pose."""
        if self.map_data is None:
            return None

        tx, ty = target
        base_angle = 0.0  # Could derive from robot pose if available
        step_rad = math.radians(self.angular_step_deg)

        # Generate radii and angle samples
        radii = [self.search_radius * (i + 1) / float(self.radial_samples)
                 for i in range(self.radial_samples)]
        angles = [base_angle + k * step_rad for k in range(-int(math.pi / step_rad), int(math.pi / step_rad))]

        for r in radii:
            for ang in angles:
                cx = tx + r * math.cos(ang)
                cy = ty + r * math.sin(ang)
                if not self.is_footprint_free(cx, cy):
                    continue
                # Optional ray check can be added if robot pose is known
                return (cx, cy)
        return None

    # ------------------------------------------------------------------
    def is_footprint_free(self, x, y):
        """Check all cells within robot radius for obstacle/unknown."""
        if self.map_data is None:
            return False

        res = self.map_resolution
        radius = self.robot_radius
        r2 = radius * radius

        mx0 = int((x - self.map_origin[0]) / res)
        my0 = int((y - self.map_origin[1]) / res)
        span = int(math.ceil(radius / res))

        for dy in range(-span, span + 1):
            for dx in range(-span, span + 1):
                if dx * dx + dy * dy > (radius / res) ** 2:
                    continue
                mx = mx0 + dx
                my = my0 + dy
                if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
                    return False
                cost = self.map_data[my, mx]
                if cost < 0:  # unknown
                    if not self.allow_unknown:
                        return False
                elif cost >= self.cost_threshold:
                    return False
        return True

    # ------------------------------------------------------------------
    def ray_path_acceptable(self, x0, y0, x1, y1):
        """Check if line between two points is traversable."""
        if self.map_data is None:
            return False
        res = self.map_resolution
        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)
        steps = max(1, int(dist / max(self.line_step, res * 0.5)))
        ux, uy = dx / steps, dy / steps

        for i in range(steps + 1):
            wx = x0 + i * ux
            wy = y0 + i * uy
            mx = int((wx - self.map_origin[0]) / res)
            my = int((wy - self.map_origin[1]) / res)
            if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
                return False
            cost = self.map_data[my, mx]
            if cost < 0 and not self.allow_unknown:
                return False
            if cost >= self.cost_threshold:
                return False
        return True


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


if __name__ == "__main__":
    main()
