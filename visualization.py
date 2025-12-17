"""
Visualization and animation functions using Robot class.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from matplotlib.lines import Line2D
from typing import List, Tuple, Dict
from PIL import Image
import io
import os
import math

def save_gif(frames, filename, duration=100):
    """Save list of PIL Images as GIF."""
    if not frames:
        print(f"⚠️  No frames to save for {filename}")
        return
    
    try:
        if isinstance(frames[0], Image.Image):
            frames[0].save(
                filename,
                save_all=True,
                append_images=frames[1:],
                duration=duration,
                loop=0,
                optimize=True
            )
        else:
            pil_frames = []
            for frame in frames:
                if isinstance(frame, np.ndarray):
                    if frame.dtype == np.float32 or frame.dtype == np.float64:
                        frame = (np.clip(frame, 0, 1) * 255).astype(np.uint8)
                    elif frame.dtype == bool:
                        frame = (frame * 255).astype(np.uint8)
                    
                    if len(frame.shape) == 2:
                        frame = np.stack([frame, frame, frame], axis=-1)
                    
                    pil_frame = Image.fromarray(frame, 'RGB')
                else:
                    pil_frame = frame
                
                pil_frames.append(pil_frame)
            
            pil_frames[0].save(
                filename,
                save_all=True,
                append_images=pil_frames[1:],
                duration=duration,
                loop=0,
                optimize=True
            )
        
        print(f"✅ Saved GIF: {filename} ({len(frames)} frames)")
        
    except Exception as e:
        print(f"❌ Error saving GIF {filename}: {e}")

def create_frame_from_figure(fig):
    """Convert matplotlib figure to PIL Image."""
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=80, bbox_inches='tight', pad_inches=0.1)
    buf.seek(0)
    img = Image.open(buf)
    plt.close(fig)
    return img

def create_robot_animation_with_vision(map_grid: np.ndarray, path: List[Tuple[int, int]], 
                                     start: Tuple[int, int], goal: Tuple[int, int],
                                     title: str, filename: str, algorithm_name: str = "Algorithm",
                                     vision_radius: int = 10):
    """
    Create animation of robot moving along path with vision area.
    """
    if not path:
        print(f"⚠️  No path to visualize for {algorithm_name}")
        return []
    
    height, width = map_grid.shape
    
    print(f"🎬 Creating {algorithm_name} animation for {len(path)}-step path...")
    
    # Color schemes
    colors = {
        'A*': ('yellow', 'gold', 'blue', 'orange'),
        'D* Lite': ('cyan', 'deepskyblue', 'blue', 'lightblue'),
        'default': ('yellow', 'gold', 'blue', 'orange')
    }
    
    algo_color = colors.get(algorithm_name, colors['default'])
    path_color, point_color, robot_color, vision_color = algo_color
    
    frames = []
    
    # Pre-calculate robot cells for start and goal to avoid creating Robot objects in loop
    start_cells = get_robot_cells(start, robot_size=5)
    goal_cells = get_robot_cells(goal, robot_size=5)
    
    # Create animation frames
    for frame_idx in range(len(path) + 10):  # +10 for pause at end
        fig, ax = plt.subplots(figsize=(12, 12))
        
        # Display map with obstacles
        ax.imshow(map_grid, cmap='gray_r', vmin=0, vmax=1, alpha=0.6)
        
        # Draw start and goal areas (fast, without Robot class)
        draw_robot_cells(ax, start_cells, 'Start', 'green', 'lightgreen')
        draw_robot_cells(ax, goal_cells, 'Goal', 'red', 'lightcoral')
        
        # Draw start and goal centers
        ax.plot(start[0], start[1], 'go', markersize=12, 
                markerfacecolor='green', markeredgecolor='darkgreen',
                markeredgewidth=2, label='Start Center')
        ax.plot(goal[0], goal[1], 'ro', markersize=12,
                markerfacecolor='red', markeredgecolor='darkred',
                markeredgewidth=2, label='Goal Center')
        
        # Draw path up to current position
        current_pos_idx = min(frame_idx, len(path))
        if current_pos_idx > 0:
            draw_path_visualization(ax, path[:current_pos_idx], path_color, point_color, 
                                  algorithm_name, vision_radius)
        
        # Draw robot at current position with vision
        if 0 < current_pos_idx <= len(path):
            robot_pos = path[current_pos_idx - 1]
            draw_robot_visualization(ax, robot_pos, robot_color, vision_color, 
                                   vision_radius, frame_idx)
        
        # Configure plot
        configure_robot_plot(ax, height, width, title, algorithm_name, 
                           current_pos_idx, len(path), vision_radius)
        
        # Convert to frame
        frame_img = create_frame_from_figure(fig)
        frames.append(frame_img)
    
    # Save GIF
    save_gif(frames, filename, duration=150)
    return frames

def get_robot_cells(position: Tuple[int, int], robot_size: int = 5) -> List[Tuple[int, int]]:
    """Get all cells occupied by a 5×5 robot at given position."""
    x, y = position
    robot_radius = robot_size // 2
    cells = []
    
    for dy in range(-robot_radius, robot_radius + 1):
        for dx in range(-robot_radius, robot_radius + 1):
            cells.append((x + dx, y + dy))
    
    return cells

def draw_robot_cells(ax, cells: List[Tuple[int, int]], label: str, 
                    edge_color: str, face_color: str):
    """Draw robot cells without creating Robot objects."""
    for x, y in cells:
        rect = patches.Rectangle((x-0.5, y-0.5), 1, 1,
                               facecolor=face_color, edgecolor=edge_color,
                               alpha=0.7, linewidth=2)
        ax.add_patch(rect)

def draw_path_visualization(ax, path: List[Tuple[int, int]], 
                          line_color: str, point_color: str, 
                          label: str, vision_radius: int):
    """Draw path with visibility circles."""
    if len(path) < 2:
        return
    
    path_x = [p[0] for p in path]
    path_y = [p[1] for p in path]
    
    # Path line
    ax.plot(path_x, path_y, '-', color=line_color, linewidth=3, alpha=0.8, 
           label=f'{label} Path')
    
    # Path points
    ax.plot(path_x, path_y, 'o', color=point_color, markersize=5, alpha=0.6)

def draw_robot_visualization(ax, position: Tuple[int, int], 
                           robot_color: str, vision_color: str,
                           vision_radius: int, frame_idx: int):
    """Draw robot with 5×5 body and vision area."""
    x, y = position
    
    # Draw robot body (5×5 cells)
    robot_cells = get_robot_cells(position, robot_size=5)
    for cell_x, cell_y in robot_cells:
        rect = patches.Rectangle((cell_x-0.5, cell_y-0.5), 1, 1,
                               facecolor='lightblue', edgecolor=robot_color,
                               alpha=0.8, linewidth=2)
        ax.add_patch(rect)
    
    # Robot center
    ax.plot(x, y, 'o', color=robot_color, markersize=10,
           markerfacecolor=robot_color, markeredgecolor='darkblue',
           markeredgewidth=2, label='Robot Center')
    
    # Draw vision area
    vision_circle = patches.Circle((x, y), vision_radius,
                                 fill=True, color=vision_color,
                                 alpha=0.15)
    ax.add_patch(vision_circle)
    
    # Vision circle outline
    vision_outline = patches.Circle((x, y), vision_radius,
                                  fill=False, color=vision_color,
                                  linewidth=2, alpha=0.5, linestyle='-')
    ax.add_patch(vision_outline)
    
    # Pulsing effect for vision area
    pulse_radius = vision_radius * (0.8 + 0.2 * math.sin(frame_idx * 0.3))
    pulse_circle = patches.Circle((x, y), pulse_radius,
                                fill=False, color=vision_color,
                                linewidth=1, alpha=0.3, linestyle=':')
    ax.add_patch(pulse_circle)

def configure_robot_plot(ax, height, width, title, algorithm_name, 
                        current_step, total_steps, vision_radius):
    """Configure plot appearance for robot visualization."""
    ax.set_xlim(-1, width)
    ax.set_ylim(height, -1)  # Inverted y-axis
    ax.set_aspect('equal')
    
    # Title
    ax.set_title(f"{title} ({algorithm_name})\n"
                f"Step {current_step}/{total_steps} | Robot: 5×5 cells | Vision: {vision_radius} cells",
                fontsize=14, fontweight='bold')
    
    ax.set_xlabel("X coordinate", fontsize=11)
    ax.set_ylabel("Y coordinate", fontsize=11)
    ax.grid(True, alpha=0.2, linestyle='--')
    
    # Custom legend
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='green', 
               markersize=10, label='Start Center'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
               markersize=10, label='Goal Center'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', 
               markersize=10, label='Robot Center'),
        patches.Patch(facecolor='lightblue', edgecolor='blue', alpha=0.8, 
                     label='Robot Body (5×5)'),
        patches.Patch(facecolor='cyan', alpha=0.15, label='Vision Area'),
        Line2D([0], [0], color='yellow', linewidth=3, label='Path'),
    ]
    
    ax.legend(handles=legend_elements, loc='upper right', fontsize=9, 
              framealpha=0.9, ncol=2)
    
    # Info box
    info_text = f"Path length: {total_steps-1} steps\n"
    info_text += f"Vision radius: {vision_radius} cells\n"
    info_text += f"Robot size: 5×5 cells"
    
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

def create_robot_comparison_plot(map_grid: np.ndarray, paths: dict, 
                                start: Tuple[int, int], goal: Tuple[int, int],
                                title: str, filename: str, vision_radius: int = 10):
    """Create comparison plot with robot visualization."""
    height, width = map_grid.shape
    
    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    
    # Colors
    colors = {
        'A*': ('yellow', 'gold', 'blue', 'orange'),
        'D* Lite': ('cyan', 'deepskyblue', 'blue', 'lightblue')
    }
    
    # Pre-calculate robot cells
    start_cells = get_robot_cells(start, robot_size=5)
    goal_cells = get_robot_cells(goal, robot_size=5)
    
    # Plot 1: Map with start/goal
    ax1 = axes[0]
    ax1.imshow(map_grid, cmap='gray_r', vmin=0, vmax=1, alpha=0.6)
    draw_robot_cells(ax1, start_cells, 'Start', 'green', 'lightgreen')
    draw_robot_cells(ax1, goal_cells, 'Goal', 'red', 'lightcoral')
    ax1.plot(start[0], start[1], 'go', markersize=8, label='Start')
    ax1.plot(goal[0], goal[1], 'ro', markersize=8, label='Goal')
    
    # Add vision area at start
    start_circle = patches.Circle(start, vision_radius,
                                fill=True, color='lightgreen',
                                alpha=0.1, label='Start Vision')
    ax1.add_patch(start_circle)
    
    ax1.set_title("Map with Start/Goal Areas\n(Robot Vision Radius Shown)")
    ax1.set_xlabel("X coordinate")
    ax1.set_ylabel("Y coordinate")
    ax1.grid(True, alpha=0.2)
    ax1.legend(loc='upper right')
    
    # Plot 2: A* path
    ax2 = axes[1]
    ax2.imshow(map_grid, cmap='gray_r', vmin=0, vmax=1, alpha=0.4)
    
    if 'A*' in paths and paths['A*']:
        a_star_path = paths['A*']
        path_x = [p[0] for p in a_star_path]
        path_y = [p[1] for p in a_star_path]
        
        # Draw path
        ax2.plot(path_x, path_y, '-', color=colors['A*'][0], linewidth=3, 
                alpha=0.8, label='A* Path')
        ax2.plot(path_x, path_y, 'o', color=colors['A*'][1], markersize=4, alpha=0.6)
        
        # Draw key robot positions
        for i, (x, y) in enumerate(a_star_path):
            if i % 10 == 0 or i == len(a_star_path) - 1:
                # Draw vision area at key points
                vision = patches.Circle((x, y), vision_radius,
                                      fill=True, color=colors['A*'][3],
                                      alpha=0.1)
                ax2.add_patch(vision)
    
    # Draw start and goal
    draw_robot_cells(ax2, start_cells, 'Start', 'green', 'lightgreen')
    draw_robot_cells(ax2, goal_cells, 'Goal', 'red', 'lightcoral')
    ax2.plot(start[0], start[1], 'go', markersize=8)
    ax2.plot(goal[0], goal[1], 'ro', markersize=8)
    
    a_star_len = len(paths.get('A*', [])) - 1 if paths.get('A*') else 'N/A'
    ax2.set_title(f"A* Path\nLength: {a_star_len} | Vision Radius: {vision_radius}")
    ax2.set_xlabel("X coordinate")
    ax2.set_ylabel("Y coordinate")
    ax2.grid(True, alpha=0.2)
    ax2.legend(loc='upper right')
    
    # Plot 3: D* Lite path
    ax3 = axes[2]
    ax3.imshow(map_grid, cmap='gray_r', vmin=0, vmax=1, alpha=0.4)
    
    if 'D* Lite' in paths and paths['D* Lite']:
        d_star_path = paths['D* Lite']
        path_x = [p[0] for p in d_star_path]
        path_y = [p[1] for p in d_star_path]
        
        # Draw path
        ax3.plot(path_x, path_y, '-', color=colors['D* Lite'][0], linewidth=3, 
                alpha=0.8, label='D* Lite Path')
        ax3.plot(path_x, path_y, 'o', color=colors['D* Lite'][1], markersize=4, alpha=0.6)
        
        # Draw key robot positions
        for i, (x, y) in enumerate(d_star_path):
            if i % 10 == 0 or i == len(d_star_path) - 1:
                vision = patches.Circle((x, y), vision_radius,
                                      fill=True, color=colors['D* Lite'][3],
                                      alpha=0.1)
                ax3.add_patch(vision)
    
    # Draw start and goal
    draw_robot_cells(ax3, start_cells, 'Start', 'green', 'lightgreen')
    draw_robot_cells(ax3, goal_cells, 'Goal', 'red', 'lightcoral')
    ax3.plot(start[0], start[1], 'go', markersize=8)
    ax3.plot(goal[0], goal[1], 'ro', markersize=8)
    
    d_star_len = len(paths.get('D* Lite', [])) - 1 if paths.get('D* Lite') else 'N/A'
    ax3.set_title(f"D* Lite Path\nLength: {d_star_len} | Vision Radius: {vision_radius}")
    ax3.set_xlabel("X coordinate")
    ax3.set_ylabel("Y coordinate")
    ax3.grid(True, alpha=0.2)
    ax3.legend(loc='upper right')
    
    plt.suptitle(f"{title}\nRobot Size: 5×5 cells | Vision Radius: {vision_radius} cells", 
                 fontsize=16, y=1.02)
    plt.tight_layout()
    
    # Save plot
    os.makedirs("robot_visualizations_blind", exist_ok=True)
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✅ Saved robot comparison plot: {filename}")

def demonstrate_robot_vision(map_grid: np.ndarray, start: Tuple[int, int], 
                            goal: Tuple[int, int], filename: str = "robot_vision_demo.gif"):
    """Create demonstration of robot vision capabilities."""
    print("🎬 Creating robot vision demonstration...")
    
    # Import Robot class here to avoid circular imports
    from robot_class import Robot
    
    # Create robot
    robot = Robot(map_grid.copy())
    
    if not robot.set_position(start[0], start[1]):
        print("❌ Could not place robot at start position")
        return
    
    frames = []
    vision_radius = 10
    
    # Create demonstration frames
    for frame_idx in range(20):  # 20 frames for demo
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Frame 1: Map with robot and vision
        ax1.imshow(map_grid, cmap='gray_r', vmin=0, vmax=1, alpha=0.7)
        
        # Draw robot using Robot class methods
        occupied = robot.get_occupied_cells()
        for x, y in occupied:
            rect = patches.Rectangle((x-0.5, y-0.5), 1, 1,
                                   facecolor='lightblue', edgecolor='blue',
                                   alpha=0.8, linewidth=2)
            ax1.add_patch(rect)
        
        # Draw vision area
        vision_circle = patches.Circle(robot.get_position(), vision_radius,
                                     fill=True, color='cyan', alpha=0.2)
        ax1.add_patch(vision_circle)
        
        # Draw vision outline
        vision_outline = patches.Circle(robot.get_position(), vision_radius,
                                      fill=False, color='blue', linewidth=2, alpha=0.5)
        ax1.add_patch(vision_outline)
        
        # Draw goal
        ax1.plot(goal[0], goal[1], 'ro', markersize=15, 
                markerfacecolor='red', markeredgecolor='darkred',
                markeredgewidth=2, label='Goal')
        
        ax1.set_title(f"Robot with Vision Area (Radius: {vision_radius} cells)")
        ax1.set_xlabel("X coordinate")
        ax1.set_ylabel("Y coordinate")
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper right')
        
        # Frame 2: Vision mask (using Robot's method)
        vision_mask = robot.get_vision_mask(vision_radius)
        ax2.imshow(vision_mask, cmap='Blues', vmin=0, vmax=1, alpha=0.8)
        ax2.imshow(map_grid, cmap='gray_r', vmin=0, vmax=1, alpha=0.3)
        
        # Overlay robot position
        robot_pos = robot.get_position()
        ax2.plot(robot_pos[0], robot_pos[1], 'bo', markersize=12,
                markerfacecolor='blue', markeredgecolor='darkblue',
                markeredgewidth=2, label='Robot')
        
        ax2.set_title("Robot Vision Mask\n(Visible area in blue)")
        ax2.set_xlabel("X coordinate")
        ax2.set_ylabel("Y coordinate")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper right')
        
        plt.suptitle(f"Robot Vision System Demonstration\nFrame {frame_idx+1}/20", 
                     fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        # Convert to frame
        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=80, bbox_inches='tight')
        buf.seek(0)
        frame_img = Image.open(buf)
        frames.append(frame_img)
        plt.close()
    
    # Save demonstration GIF
    os.makedirs("robot_visualizations_blind", exist_ok=True)
    save_gif(frames, f"robot_visualizations_blind/{filename}", duration=200)
    print(f"✅ Created robot vision demonstration: {filename}")