# visualization.py
"""
Module for creating visualizations of robot paths with limited visibility.
"""

import numpy as np
from typing import List, Tuple
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image
import io
import os


def get_5x5_cells(pos: Tuple[int, int]) -> List[Tuple[int, int]]:
    """Return all cells occupied by a 5x5 robot centered at pos."""
    if pos is None:
        return []
    x, y = pos
    return [(x + dx, y + dy) for dy in range(-2, 3) for dx in range(-2, 3)]


def create_single_robot_animation(
    map_grid: np.ndarray,
    a_path: List[Tuple[int, int]],
    d_path: List[Tuple[int, int]],
    start: Tuple[int, int],
    goal: Tuple[int, int],
    map_id: int,
    vision_radius: int = 10
):
    """
    Create a GIF animation comparing A* and D* Lite paths on a single map
    with limited visibility.
    """
    max_frames = max(len(a_path), len(d_path)) + 20  # buffer frames
    frames = []

    height, width = map_grid.shape

    for frame in range(max_frames):
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))

        for ax, algo_name, path in zip(axes, ['A*', 'D* Lite'], [a_path, d_path]):
            ax.imshow(map_grid, cmap='gray_r', alpha=0.7, origin='upper')

            # Start and Goal
            for cell in get_5x5_cells(start):
                ax.add_patch(patches.Rectangle((cell[0] - 0.5, cell[1] - 0.5), 1, 1,
                                               facecolor='lightgreen', edgecolor='green', alpha=0.8))
            for cell in get_5x5_cells(goal):
                ax.add_patch(patches.Rectangle((cell[0] - 0.5, cell[1] - 0.5), 1, 1,
                                               facecolor='lightcoral', edgecolor='red', alpha=0.8))

            # Current path and robot position
            if frame < len(path):
                path_so_far = path[:frame + 1]
                xs, ys = zip(*path_so_far)
                color = 'orange' if algo_name == 'A*' else 'cyan'
                ax.plot(xs, ys, '-', color=color, linewidth=3, alpha=0.9)
                current_pos = path_so_far[-1]

                # Robot (5x5)
                for cell in get_5x5_cells(current_pos):
                    ax.add_patch(patches.Rectangle((cell[0] - 0.5, cell[1] - 0.5), 1, 1,
                                                   facecolor='lightblue', edgecolor=color, alpha=0.8))

                # Visibility circle
                ax.add_patch(patches.Circle(current_pos, vision_radius,
                                            color=color, alpha=0.15, fill=True))
                ax.add_patch(patches.Circle(current_pos, vision_radius,
                                            color=color, alpha=0.6, fill=False, linewidth=2))

            ax.set_xlim(0, width)
            ax.set_ylim(height, 0)
            ax.set_aspect('equal')
            ax.set_title(f"{algo_name} - Frame {frame}", fontsize=14)
            ax.grid(True, alpha=0.3)

        plt.suptitle(f"Map {map_id}: Comparison of A* and D* Lite\n"
                     f"Limited visibility (radius {vision_radius} cells)", fontsize=16)
        plt.tight_layout()

        # Save frame to buffer
        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=100, bbox_inches='tight')
        buf.seek(0)
        frames.append(Image.open(buf))
        plt.close(fig)

    # Save GIF
    if frames:
        os.makedirs("robot_visualizations_blind", exist_ok=True)
        filename = f"robot_visualizations_blind/map{map_id}_comparison.gif"
        frames[0].save(
            filename,
            save_all=True,
            append_images=frames[1:],
            duration=300,  # ms between frames
            loop=0
        )
        print(f"Animation for single robot saved: {filename}")