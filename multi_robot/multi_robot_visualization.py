# multi_robot/multi_robot_visualization.py
"""
Visualization module for multi-robot comparison.
Creates a GIF animation showing A* and D* Lite paths with a moving secondary robot as dynamic obstacle.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image
import io
import os
from typing import List, Tuple, Dict


def get_5x5_cells(pos: Tuple[int, int]) -> List[Tuple[int, int]]:
    """Return all grid cells occupied by a 5x5 robot centered at pos."""
    if pos is None:
        return []
    x, y = pos
    return [(x + dx, y + dy) for dy in range(-2, 3) for dx in range(-2, 3)]


def create_multi_robot_animation(
    map_grid: np.ndarray,
    a_result: Dict,
    d_result: Dict,
    secondary_path: List[Tuple[int, int]],
    filename: str
):
    """
    Create a GIF animation comparing A* and D* Lite in a dynamic environment
    with a moving secondary robot.
    
    Args:
        map_grid: The environment map.
        a_result: Result dictionary from A* simulation.
        d_result: Result dictionary from D* Lite simulation.
        secondary_path: List of positions of the secondary (moving obstacle) robot.
        filename: Output GIF file path.
    """
    if not a_result.get('success', False) and not d_result.get('success', False):
        print("No successful results to visualize")
        return

    max_frames = max(
        len(a_result.get('path', [])),
        len(d_result.get('path', [])),
        len(secondary_path)
    ) + 10

    height, width = map_grid.shape
    frames = []

    start = a_result.get('start') or d_result.get('start')
    goal = a_result.get('goal') or d_result.get('goal')
    vision_radius = a_result.get('vision_radius', 10)

    colors = {
        'A*': {'robot': 'orange', 'vision': 'orange', 'path': 'gold'},
        'D* Lite': {'robot': 'cyan', 'vision': 'deepskyblue', 'path': 'deepskyblue'}
    }

    for frame in range(max_frames):
        fig, axes = plt.subplots(1, 2, figsize=(18, 9))

        for ax, algo_name, result in zip(axes, ['A*', 'D* Lite'], [a_result, d_result]):
            ax.imshow(map_grid, cmap='gray_r', alpha=0.6, origin='upper')

            # Start position
            if start:
                for cell in get_5x5_cells(start):
                    ax.add_patch(patches.Rectangle((cell[0] - 0.5, cell[1] - 0.5), 1, 1,
                                                   facecolor='lightgreen', edgecolor='green', alpha=0.7))
                ax.plot(start[0], start[1], 'o', color='green', markersize=12)

            # Goal position
            if goal:
                for cell in get_5x5_cells(goal):
                    ax.add_patch(patches.Rectangle((cell[0] - 0.5, cell[1] - 0.5), 1, 1,
                                                   facecolor='lightcoral', edgecolor='red', alpha=0.7))
                ax.plot(goal[0], goal[1], 'o', color='red', markersize=12)

            # Secondary (moving obstacle) robot
            sec_idx = min(frame, len(secondary_path) - 1)
            sec_pos = secondary_path[sec_idx]
            for cell in get_5x5_cells(sec_pos):
                ax.add_patch(patches.Rectangle((cell[0] - 0.5, cell[1] - 0.5), 1, 1,
                                               facecolor='lightpink', edgecolor='magenta', alpha=0.9))

            # Primary robot path and current position
            path = result.get('path', [])
            if result.get('success', False) and frame < len(path):
                path_so_far = path[:frame + 1]
                xs, ys = zip(*path_so_far)
                color = colors[algo_name]
                ax.plot(xs, ys, '-', color=color['path'], linewidth=3, alpha=0.9)

                current_pos = path_so_far[-1]
                for cell in get_5x5_cells(current_pos):
                    ax.add_patch(patches.Rectangle((cell[0] - 0.5, cell[1] - 0.5), 1, 1,
                                                   facecolor='lightblue', edgecolor=color['robot'], alpha=0.8))

                # Vision circle
                ax.add_patch(patches.Circle(current_pos, vision_radius,
                                            color=color['vision'], alpha=0.15, fill=True))
                ax.add_patch(patches.Circle(current_pos, vision_radius,
                                            color=color['vision'], alpha=0.5, fill=False, linewidth=2))

            ax.set_xlim(0, width)
            ax.set_ylim(height, 0)
            ax.set_aspect('equal')
            status = 'Success' if result.get('success', False) else 'Failed'
            ax.set_title(f"{algo_name} - {status} - Frame {frame}")

        plt.suptitle("A* vs D* Lite: Dynamic Obstacle (Moving Robot)\n"
                     "Vision radius = 10 cells", fontsize=16)
        plt.tight_layout()

        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=100, bbox_inches='tight')
        buf.seek(0)
        frames.append(Image.open(buf))
        plt.close(fig)

    # Save animation
    if frames:
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        frames[0].save(
            filename,
            save_all=True,
            append_images=frames[1:],
            duration=300,
            loop=0
        )
        print(f"Animation saved: {filename}")