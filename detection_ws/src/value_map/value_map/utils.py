import numpy as np
import math
import geometry_msgs.msg as Pose
from tf_transformations import euler_from_quaternion


def normalize_angle(angle_rad: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad

def get_yaw_angle(pose: Pose) -> float:
    q = pose.orientation
    quaternion = [q.x, q.y, q.z, q.w]

    roll, pitch, yaw = euler_from_quaternion(quaternion)
    return yaw