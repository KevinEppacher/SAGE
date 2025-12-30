#!/usr/bin/env python3
from nav_msgs.msg import Path
import tf2_ros
import rclpy
from geometry_msgs.msg import PoseStamped
import tf_transformations
import numpy as np

def transform_path(tf_buffer: tf2_ros.Buffer, target_frame: str, path: Path) -> Path:
    out = Path()
    out.header.stamp = path.header.stamp
    out.header.frame_id = target_frame
    for pose in path.poses:
        out.poses.append(transform_pose(tf_buffer, target_frame, pose))
    return out

def transform_pose(tf_buffer: tf2_ros.Buffer, target_frame: str, pose: PoseStamped) -> PoseStamped:
    tf = tf_buffer.lookup_transform(
        target_frame,
        pose.header.frame_id,
        rclpy.time.Time()
    )

    # translation
    t = tf.transform.translation
    # rotation
    q = tf.transform.rotation
    T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])

    p = np.array([pose.pose.position.x,
                  pose.pose.position.y,
                  pose.pose.position.z,
                  1.0])

    p_t = T @ p
    p_t[:3] += np.array([t.x, t.y, t.z])

    out = PoseStamped()
    out.header.stamp = pose.header.stamp
    out.header.frame_id = target_frame
    out.pose = pose.pose
    out.pose.position.x = p_t[0]
    out.pose.position.y = p_t[1]
    out.pose.position.z = p_t[2]

    return out