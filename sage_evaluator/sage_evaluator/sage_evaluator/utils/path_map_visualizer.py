#!/usr/bin/env python3
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose
import numpy as np

class PathMapVisualizer:
    """
    Visualizes:
    - OccupancyGrid
    - Actual path
    - Geodesic (shortest) path
    - Start / End / Target points

    Output: static PNG (matplotlib)
    """

    def __init__(self, node):
        self.node = node
        self.map = None  # nav_msgs/OccupancyGrid

    # ------------------------------------------------------------
    def set_map(self, occupancy_grid: OccupancyGrid):
        self.map = occupancy_grid
        self.node.get_logger().info(
            f"Map received: {occupancy_grid.info.width}x{occupancy_grid.info.height}, "
            f"res={occupancy_grid.info.resolution}"
        )

    # ------------------------------------------------------------
    def save_plot(
        self,
        output_path: str,
        actual_path: Path,
        geodesic_path: Path,
        start_pose: Pose,
        end_pose: Pose,
        target_pose: Pose
    ):
        if self.map is None:
            self.node.get_logger().error("No map set, cannot visualize")
            return

        grid = self._occupancy_to_numpy(self.map)

        fig, ax = plt.subplots(figsize=(8, 6))
        ax.imshow(
            grid,
            cmap="gray",
            origin="lower"
        )

        # Paths
        self._plot_path(ax, actual_path, "tab:blue", "Actual path", draw_arrows=True)
        self._plot_path(ax, geodesic_path, "tab:green", "Geodesic path", draw_arrows=True)

        # Points
        self._plot_point(ax, start_pose, "cyan", "Start")
        self._plot_point(ax, end_pose, "magenta", "End")
        self._plot_point(ax, target_pose, "red", "Target")

        ax.legend()
        ax.set_title("Navigation Evaluation")
        ax.set_xlabel("Map X")
        ax.set_ylabel("Map Y")

        plt.tight_layout()
        plt.savefig(output_path, dpi=200)
        plt.close(fig)

        self.node.get_logger().info(f"Saved path visualization → {output_path}")

    # ============================================================
    # Internal helpers
    # ============================================================

    def _occupancy_to_numpy(self, map_msg: OccupancyGrid):
        data = np.array(map_msg.data, dtype=np.int8)
        data = data.reshape(
            map_msg.info.height,
            map_msg.info.width
        )

        # Normalize:
        #  -1 unknown -> 0.5
        #  0 free     -> 1.0
        # 100 occ     -> 0.0
        grid = np.ones_like(data, dtype=np.float32)
        grid[data == 100] = 0.0
        grid[data == -1]  = 0.5

        return grid

    def _plot_path(self, ax, path: Path, color: str, label: str, draw_arrows: bool = True):
        if path is None or not path.poses or len(path.poses) < 2:
            return

        xs, ys = [], []
        for p in path.poses:
            mx, my = self._world_to_map(
                p.pose.position.x,
                p.pose.position.y
            )
            xs.append(mx)
            ys.append(my)

        ax.plot(xs, ys, color=color, linewidth=2, label=label)

        if draw_arrows:
            self._plot_flow_arrows(ax, xs, ys, color)

    def _plot_point(self, ax, pose: Pose, color: str, label: str):
        if pose is None:
            return

        mx, my = self._world_to_map(
            pose.position.x,
            pose.position.y
        )

        ax.scatter(mx, my, c=color, s=80, marker="x", label=label)

    def _plot_flow_arrows(
        self,
        ax,
        xs,
        ys,
        color: str,
        step: int = 8,
        arrow_len: float = 6.0,
    ):
        """
        Draw direction arrows along a path.
        step: draw one arrow every N segments
        arrow_len: visual length in map pixels
        """
        xs = np.asarray(xs)
        ys = np.asarray(ys)

        dx = np.diff(xs)
        dy = np.diff(ys)

        if len(dx) == 0:
            return

        # Subsample
        idx = np.arange(0, len(dx), step)

        # Normalize direction
        norms = np.hypot(dx[idx], dy[idx])
        norms[norms == 0.0] = 1.0

        ux = dx[idx] / norms * arrow_len
        uy = dy[idx] / norms * arrow_len

        ax.quiver(
            xs[idx],
            ys[idx],
            ux,
            uy,
            angles="xy",
            scale_units="xy",
            scale=1,
            color=color,
            width=0.006,
            headwidth=4,
            headlength=6,
            headaxislength=5,
            alpha=0.9,
            zorder=5,   # <-- IMPORTANT
        )

    def _world_to_map(self, x: float, y: float):
        info = self.map.info
        mx = (x - info.origin.position.x) / info.resolution
        my = (y - info.origin.position.y) / info.resolution
        return mx, my

