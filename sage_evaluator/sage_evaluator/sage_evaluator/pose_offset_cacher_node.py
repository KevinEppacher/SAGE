#!/usr/bin/env python3
import os
import json
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf_transformations import euler_from_quaternion
import tf2_ros

CACHE_PATH = "/app/data/robot_alignment_cache.json"

class RobotAlignmentCache(Node):
    def __init__(self):
        super().__init__("robot_alignment_cache")

        self.declare_parameter("pose_topic", "/initialpose")  # could also use /initialpose
        self.declare_parameter("translation_threshold", 0.1)
        self.declare_parameter("rotation_threshold_deg", 5.0)
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "robot_original_pose_at_scan")
        self.declare_parameter("cache_path", CACHE_PATH)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.trans_thresh = self.get_parameter("translation_threshold").value
        self.rot_thresh = np.deg2rad(self.get_parameter("rotation_threshold_deg").value)
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.cache_path = self.get_parameter("cache_path").value

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.latest_pose = None
        self.cached_pose = self._load_cached_pose()

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            10,
        )

        if self.cached_pose is not None:
            self.publish_static_transform(self.cached_pose)
            self.get_logger().info(
                f"Loaded cached alignment and published static TF → {self.child_frame}"
            )
        else:
            self.get_logger().info(
                "No cached alignment found. Waiting for manual /initialpose..."
            )

    # ----------------------------------------------------------------------
    def _load_cached_pose(self):
        if not os.path.exists(self.cache_path):
            return None
        try:
            with open(self.cache_path, "r") as f:
                data = json.load(f)
            return data
        except Exception as e:
            self.get_logger().warn(f"Failed to load cache: {e}")
            return None

    def _save_cached_pose(self, pose):
        os.makedirs(os.path.dirname(self.cache_path), exist_ok=True)
        with open(self.cache_path, "w") as f:
            json.dump(pose, f, indent=2)
        self.get_logger().info(f"Saved alignment cache → {self.cache_path}")

    # ----------------------------------------------------------------------
    def pose_callback(self, msg):
        """Callback on /amcl_pose or /initialpose — detect significant movement."""
        pose = msg.pose.pose
        new_pose = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "qx": float(pose.orientation.x),
            "qy": float(pose.orientation.y),
            "qz": float(pose.orientation.z),
            "qw": float(pose.orientation.w),
        }

        if self.cached_pose is None:
            self._save_cached_pose(new_pose)
            self.publish_static_transform(new_pose)
            self.cached_pose = new_pose
            return

        if not self._pose_changed_significantly(new_pose, self.cached_pose):
            return

        self._save_cached_pose(new_pose)
        self.publish_static_transform(new_pose)
        self.cached_pose = new_pose
        self.get_logger().info(
            "Robot moved significantly → updated static transform and cache."
        )

    # ----------------------------------------------------------------------
    def _pose_changed_significantly(self, new, old):
        trans_diff = np.linalg.norm(
            [new["x"] - old["x"], new["y"] - old["y"], new["z"] - old["z"]]
        )
        yaw_old = euler_from_quaternion([old["qx"], old["qy"], old["qz"], old["qw"]])[2]
        yaw_new = euler_from_quaternion([new["qx"], new["qy"], new["qz"], new["qw"]])[2]
        rot_diff = abs(yaw_new - yaw_old)
        return trans_diff > self.trans_thresh or rot_diff > self.rot_thresh

    # ----------------------------------------------------------------------
    def publish_static_transform(self, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = pose["x"]
        t.transform.translation.y = pose["y"]
        t.transform.translation.z = pose["z"]
        t.transform.rotation.x = pose["qx"]
        t.transform.rotation.y = pose["qy"]
        t.transform.rotation.z = pose["qz"]
        t.transform.rotation.w = pose["qw"]
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published static transform: {self.parent_frame} → {self.child_frame}")


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RobotAlignmentCache()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
