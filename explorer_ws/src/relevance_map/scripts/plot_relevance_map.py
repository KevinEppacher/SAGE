#!/usr/bin/env python3
"""
3D visualization of the Relevance Map (as in RelevanceMapNode::integrateRelevance)
showing the combined radial and angular Gaussian confidence distribution.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def relevance_map(width=300, height=300,
                  res=0.05,
                  fov_deg=70.0,
                  max_range=8.0,
                  radial_sharpness=0.5,
                  angular_sharpness=0.25,
                  base_increment_rate=0.1):
    """Generate a 2D relevance field."""
    # Robot pose in the center
    robot_x, robot_y = width // 2, height // 2
    yaw = 0.0  # facing along +x axis

    # Gaussian parameters
    sigma_r = max_range * radial_sharpness
    sigma_a = (fov_deg * np.pi / 180.0) * angular_sharpness
    half_fov = (fov_deg * np.pi / 180.0) / 2.0

    # Coordinate grid
    y_idx, x_idx = np.indices((height, width))
    dx = (x_idx - robot_x) * res
    dy = (y_idx - robot_y) * res

    dist = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx)
    angle_diff = np.abs(((angle - yaw + np.pi) % (2*np.pi)) - np.pi)

    # Masks for FOV and range
    in_fov = angle_diff <= half_fov
    in_range = dist <= max_range
    mask = in_fov & in_range

    # Gaussian weights
    radial_weight = np.exp(-(dist**2) / (2 * sigma_r**2))
    angular_weight = np.exp(-0.5 * (angle_diff / sigma_a)**2)

    relevance = base_increment_rate * radial_weight * angular_weight * mask
    return relevance, (dx, dy), mask, fov_deg, max_range


def plot_relevance_map_3d():
    relevance, (dx, dy), mask, fov_deg, max_range = relevance_map()

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    # Downsample for speed
    step = 4
    X = dx[::step, ::step]
    Y = dy[::step, ::step]
    Z = relevance[::step, ::step]

    # 3D surface plot
    surf = ax.plot_surface(X, Y, Z, cmap='inferno', linewidth=0, antialiased=True)

    # Highlight robot position
    ax.scatter(0, 0, np.max(Z)*1.1, c='b', s=50, label='Robot position')

    # FOV wedge lines
    half_fov_rad = np.deg2rad(fov_deg / 2)
    for sign in [-1, 1]:
        ax.plot(
            [0, max_range * np.cos(sign * half_fov_rad)],
            [0, max_range * np.sin(sign * half_fov_rad)],
            [0, 0],
            'w--', linewidth=1.5
        )

    fontsize = 25  # choose a consistent size

    ax.set_xlabel('X [m]', fontsize=fontsize, labelpad = 20)
    ax.set_ylabel('Y [m]', fontsize=fontsize, labelpad=20)
    ax.set_zlabel('Relevance Value', fontsize=fontsize, labelpad=20)
    ax.set_title('3D Relevance Map – Combined Radial & Angular Gaussian', fontsize=fontsize)

    # Colorbar with same label size
    cbar = fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, pad=0.1)
    cbar.set_label("Relevance Value", fontsize=fontsize)

    # Also scale tick label fonts
    ax.tick_params(axis='both', which='major', labelsize=fontsize - 2)
    ax.tick_params(axis='z', which='major', labelsize=fontsize - 2)
    cbar.ax.tick_params(labelsize=fontsize - 2)

    plt.legend(loc='upper right')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_relevance_map_3d()
