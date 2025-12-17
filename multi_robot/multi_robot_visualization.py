# multi_robot/multi_robot_visualization.py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D
from PIL import Image
import io
import os

def create_multi_robot_animation(map_grid, a_result, d_result, secondary_path, filename):
    if not a_result.get('success', False) and not d_result.get('success', False):
        print("⚠️ No successful results to visualize")
        return
    
    max_steps_a = len(a_result['path']) if a_result.get('success') else 0
    max_steps_d = len(d_result['path']) if d_result.get('success') else 0
    max_secondary_steps = len(secondary_path)
    total_frames = max(max_steps_a, max_steps_d, max_secondary_steps) + 10

    if total_frames <= 10:
        print("⚠️ Not enough path data to create a meaningful animation.")
        return

    height, width = map_grid.shape
    frames = []

    algo_colors = {
        'A*': {'path': 'gold', 'point': 'orange', 'robot': 'orange', 'vision': 'orange'},
        'D* Lite': {'path': 'deepskyblue', 'point': 'cyan', 'robot': 'cyan', 'vision': 'cyan'}
    }

    def get_5x5_cells(pos):
        if pos is None:
            return []
        x, y = pos
        cells = []
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                cells.append((x + dx, y + dy))
        return cells

    start_pos = a_result.get('start') or d_result.get('start')
    goal_pos = a_result.get('goal') or d_result.get('goal')
    vision_radius = a_result.get('vision_radius', 12)

    for frame_idx in range(total_frames):
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))
        
        for ax_idx, (ax, algorithm, result) in enumerate(zip(axes, ['A*', 'D* Lite'], [a_result, d_result])):
            ax.imshow(map_grid, cmap='gray_r', vmin=0, vmax=1, alpha=0.6)
            
            # Draw Start (Green)
            if start_pos:
                start_cells = get_5x5_cells(start_pos)
                for x, y in start_cells:
                    rect = patches.Rectangle((x-0.5, y-0.5), 1, 1,
                                           facecolor='lightgreen', edgecolor='green',
                                           alpha=0.7, linewidth=2)
                    ax.add_patch(rect)
                ax.plot(start_pos[0], start_pos[1], 'go', markersize=12, 
                        markerfacecolor='green', markeredgecolor='darkgreen', markeredgewidth=2)

            # Draw Goal (Red)
            if goal_pos:
                goal_cells = get_5x5_cells(goal_pos)
                for x, y in goal_cells:
                    rect = patches.Rectangle((x-0.5, y-0.5), 1, 1,
                                           facecolor='lightcoral', edgecolor='red',
                                           alpha=0.7, linewidth=2)
                    ax.add_patch(rect)
                ax.plot(goal_pos[0], goal_pos[1], 'ro', markersize=12,
                        markerfacecolor='red', markeredgecolor='darkred', markeredgewidth=2)

            # Draw Secondary Robot (Pink)
            if frame_idx < len(secondary_path):
                sec_pos = secondary_path[frame_idx]
                sec_cells = get_5x5_cells(sec_pos)
                for x, y in sec_cells:
                    rect = patches.Rectangle((x-0.5, y-0.5), 1, 1,
                                           facecolor='lightpink', edgecolor='darkred',
                                           alpha=0.8, linewidth=2)
                    ax.add_patch(rect)

            # Draw Primary Robot Path and Position
            if result and result.get('success', False):
                current_path_len = min(frame_idx + 1, len(result['path']))
                if current_path_len > 0:
                    path_points = result['path'][:current_path_len]
                    path_x = [p[0] for p in path_points]
                    path_y = [p[1] for p in path_points]
                    
                    ax.plot(path_x, path_y, '-', color=algo_colors[algorithm]['path'], linewidth=3, alpha=0.8)
                    ax.plot(path_x, path_y, 'o', color=algo_colors[algorithm]['point'], markersize=5, alpha=0.6)
                    
                    current_pos = path_points[-1]
                    robot_cells = get_5x5_cells(current_pos)
                    for x, y in robot_cells:
                        rect = patches.Rectangle((x-0.5, y-0.5), 1, 1,
                                               facecolor='lightblue', edgecolor=algo_colors[algorithm]['robot'],
                                               alpha=0.8, linewidth=2)
                        ax.add_patch(rect)
                    
                    # Vision Circle
                    vision_circle = patches.Circle(
                        current_pos, vision_radius,
                        fill=True, color=algo_colors[algorithm]['vision'], alpha=0.15
                    )
                    ax.add_patch(vision_circle)
                    vision_outline = patches.Circle(
                        current_pos, vision_radius,
                        fill=False, color=algo_colors[algorithm]['vision'], linewidth=2, alpha=0.5
                    )
                    ax.add_patch(vision_outline)

            ax.set_xlim(-1, width)
            ax.set_ylim(height, -1)
            ax.set_aspect('equal')
            success_status = "✅ Success" if result.get('success') else "❌ Failed"
            ax.set_title(f"{algorithm} - {success_status}\nStep {min(frame_idx+1, len(result.get('path', [])))}", fontsize=12)
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.grid(True, alpha=0.3, linestyle='--')

        plt.suptitle("Multi-Robot Path Planning: A* vs D* Lite\n"
                     "Primary Robot (Blue) | Moving Obstacle (Pink) | Vision Radius (Faint Circle)", 
                     fontsize=14, fontweight='bold')
        plt.tight_layout()

        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=100, bbox_inches='tight')
        buf.seek(0)
        frame_img = Image.open(buf)
        frames.append(frame_img)
        plt.close()

    if frames:
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        try:
            frames[0].save(filename, save_all=True, append_images=frames[1:],
                          duration=200, loop=0, optimize=True)
            print(f"✅ Saved animation with vision circles: {filename}")
        except Exception as e:
            print(f"❌ Error saving animation: {e}")