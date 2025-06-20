import numpy as np
import math
import geometry_msgs.msg as Pose
import matplotlib.pyplot as plt
if not hasattr(np, 'float'):
    np.float = float
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

def value_to_inferno_rgb(value: float, vmin=0.0, vmax=1.0) -> tuple[int, int, int]:
    """Map value to RGB using inferno colormap from matplotlib."""
    norm = (value - vmin) / (vmax - vmin)
    norm = max(0.0, min(1.0, norm))  # clamp to [0, 1]
    rgba = plt.get_cmap('inferno')(norm)
    r, g, b = int(rgba[0]*255), int(rgba[1]*255), int(rgba[2]*255)
    return r, g, b
