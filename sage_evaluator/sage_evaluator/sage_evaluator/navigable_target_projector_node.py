#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros

import numpy as np
import math

class NavigableTargetProjector(Node):
    def __init__(self):
        super().__init__("navigable_target_projector")

        # Parameters
        self.declare_parameter("robot_radius", 0.35)
        self.declare_parameter("cost_threshold", 25)
        self.declare_parameter("allow_unknown", False)
        self.declare_parameter("angular_step_deg", 10.0)
        self.declare_parameter("radial_samples", 6)
        self.declare_parameter("line_step", 0.05)
        self.declare_parameter("search_radius", 2.5)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")

        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.cost_threshold = int(self.get_parameter("cost_threshold").value)
        self.allow_unknown = bool(self.get_parameter("allow_unknown").value)
        self.angular_step_deg = float(self.get_parameter("angular_step_deg").value)
        self.radial_samples = int(self.get_parameter("radial_samples").value)
        self.line_step = float(self.get_parameter("line_step").value)
        self.search_radius = float(self.get_parameter("search_radius").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers / Publishers
        self.create_subscription(
            PoseArray,
            "/evaluator/semantic_centroid_targets",
            self.centroid_cb,
            10)

        self.create_subscription(
            OccupancyGrid,
            "/evaluator/global_costmap/costmap",
            self.costmap_cb,
            10)

        self.target_pub = self.create_publisher(
            PoseArray,
            "navigable_targets",
            10)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "debug_sampling_markers",
            10)

        # Map state
        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin = None

    # -----------------------------------------------------

    def costmap_cb(self, msg: OccupancyGrid):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (
            msg.info.origin.position.x,
            msg.info.origin.position.y
        )
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            self.map_height, self.map_width)

    # -----------------------------------------------------

    def get_robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time())
            return (
                tf.transform.translation.x,
                tf.transform.translation.y
            )
        except Exception:
            return None

    # -----------------------------------------------------

    def centroid_cb(self, msg: PoseArray):
        if self.map_data is None:
            return

        robot_xy = self.get_robot_position()
        if robot_xy is None:
            return

        rx, ry = robot_xy

        out = PoseArray()
        out.header = msg.header

        for pose in msg.poses:
            target = (pose.position.x, pose.position.y)

            chosen, samples, is_valid = self.find_reachable_point(target, rx, ry)

            # Visualization (same markers, reachable may be fallback)
            self.publish_markers(target, samples, chosen, rx, ry)

            # Always publish exactly one pose
            p = Pose()
            p.position.x = chosen[0]
            p.position.y = chosen[1]
            p.orientation = Quaternion(w=1.0)
            out.poses.append(p)

        # Always publish (even if only fallbacks)
        self.target_pub.publish(out)

    # -----------------------------------------------------

    def find_reachable_point(self, target, rx, ry):
        tx, ty = target
        samples = []

        base_angle = math.atan2(ry - ty, rx - tx)
        step_rad = math.radians(self.angular_step_deg)

        # Radii (identical ordering to C++)
        radii = [0.0]
        for i in range(max(1, self.radial_samples)):
            radii.append(
                self.search_radius * (i + 1) / float(self.radial_samples)
            )

        # Angles (base, +Δ, −Δ, +2Δ, −2Δ, ...)
        angles = [base_angle]
        max_k = int(math.pi / step_rad) + 1
        for k in range(1, max_k + 1):
            angles.append(base_angle + k * step_rad)
            angles.append(base_angle - k * step_rad)

        best_fallback = None
        best_cost = float("inf")

        for r in radii:
            for ang in angles:
                cx = tx + r * math.cos(ang)
                cy = ty + r * math.sin(ang)

                cx, cy = self.clamp_to_map(cx, cy)
                samples.append((cx, cy))

                # --- cost lookup for fallback ---
                mx = int((cx - self.map_origin[0]) / self.map_resolution)
                my = int((cy - self.map_origin[1]) / self.map_resolution)

                if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                    cost = self.map_data[my, mx]
                    if cost >= 0 and cost < best_cost:
                        best_cost = cost
                        best_fallback = (cx, cy)

                # --- strict validity checks ---
                if not self.is_footprint_free(cx, cy):
                    continue

                if not self.ray_path_acceptable(rx, ry, cx, cy):
                    continue

                # Fully valid pose found → return immediately
                return (cx, cy), samples, True

        # No valid pose → return lowest-cost fallback
        if best_fallback is not None:
            return best_fallback, samples, False

        # Absolute failure (should be extremely rare)
        return (tx, ty), samples, False

    # -----------------------------------------------------

    def is_footprint_free(self, x, y):
        res = self.map_resolution
        radius = self.robot_radius

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
                if cost < 0 and not self.allow_unknown:
                    return False
                if cost >= self.cost_threshold:
                    return False

        return True

    # -----------------------------------------------------

    def ray_path_acceptable(self, x0, y0, x1, y1):
        res = self.map_resolution
        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)

        steps = max(1, int(dist / max(self.line_step, res * 0.5)))
        ux = dx / steps
        uy = dy / steps

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

    # -----------------------------------------------------

    def clamp_to_map(self, x, y):
        res = self.map_resolution
        margin = self.robot_radius + res

        min_x = self.map_origin[0] + margin
        min_y = self.map_origin[1] + margin
        max_x = self.map_origin[0] + (self.map_width - 1) * res - margin
        max_y = self.map_origin[1] + (self.map_height - 1) * res - margin

        return (
            min(max(x, min_x), max_x),
            min(max(y, min_y), max_y)
        )

    # -----------------------------------------------------

    def publish_markers(self, target, samples, reachable, rx, ry):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        def mk(i, t, ns):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = ns
            m.id = i
            m.type = t
            m.action = Marker.ADD
            m.lifetime.sec = 2
            return m

        m = mk(1, Marker.SPHERE, "target")
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color = ColorRGBA(r=1.0, a=0.9)
        m.pose.position.x = target[0]
        m.pose.position.y = target[1]
        arr.markers.append(m)

        idx = 10
        for x, y in samples:
            s = mk(idx, Marker.SPHERE, "samples")
            idx += 1
            s.scale.x = s.scale.y = s.scale.z = 0.05
            s.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.5)
            s.pose.position.x = x
            s.pose.position.y = y
            s.pose.position.z = 0.05
            arr.markers.append(s)

        if reachable:
            r = mk(2, Marker.SPHERE, "reachable")
            r.scale.x = r.scale.y = r.scale.z = 0.25
            r.color = ColorRGBA(g=1.0, a=0.9)
            r.pose.position.x = reachable[0]
            r.pose.position.y = reachable[1]
            arr.markers.append(r)

            l = mk(3, Marker.LINE_STRIP, "ray")
            l.scale.x = 0.03
            l.color = ColorRGBA(b=1.0, a=0.8)
            l.points = [
                Point(x=rx, y=ry, z=0.05),
                Point(x=reachable[0], y=reachable[1], z=0.05)
            ]
            arr.markers.append(l)

            f = mk(4, Marker.CYLINDER, "footprint")
            f.scale.x = f.scale.y = self.robot_radius * 2.0
            f.scale.z = 0.05
            f.color = ColorRGBA(r=1.0, g=1.0, a=0.5)
            f.pose.position.x = reachable[0]
            f.pose.position.y = reachable[1]
            f.pose.position.z = 0.01
            arr.markers.append(f)

        self.marker_pub.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = NavigableTargetProjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()