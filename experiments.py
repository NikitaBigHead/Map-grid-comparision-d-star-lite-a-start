import os
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

from map_generator import MapGenerator
from grid_map import GridMap
from arena import Arena
from robot import Robot
from planners import AStarPlanner, DStarLitePlanner
import random 
import numpy as np
from tqdm import tqdm
from utils import sample_center_unique, center_ok

# === CONFIG ===
os.makedirs("results", exist_ok=True)
rng = np.random.default_rng(52)
robot_size = 5
speed = 1
max_steps = 2000
frame_ms = 120
exp_robot_counts =  [1,2, 3,4,5]
clearance = 2
vision_radius = 20
seed = 3

np.random.seed(seed)
random.seed(seed)

# === HELPERS ===
def merge_frames_side_by_side(frames_left, frames_right):
    merged = []
    n = min(len(frames_left), len(frames_right))
    for i in range(n):
        left = frames_left[i].convert("RGB")
        right = frames_right[i].convert("RGB")
        w, h = left.size
        new_img = Image.new("RGB", (w * 2, h))
        new_img.paste(left, (0, 0))
        new_img.paste(right, (w, 0))
        merged.append(new_img)
    return merged


def compute_path_length(history):
    if len(history) < 2:
        return 0.0
    pts = np.array(history)
    diffs = np.diff(pts, axis=0)
    return float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))


def render_frame(grid, robots, history_list, title=""):
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.imshow(grid, cmap="gray_r", origin="upper")

    for r, hist in zip(robots, history_list):
        # --- 1) ПЛАН до цели (полупрозрачный) ---
        if hasattr(r, "last_plan") and r.last_plan is not None and r.last_plan.ok:
            path = r.last_plan.path
            if path is not None and len(path) >= 2:
                xs, ys = zip(*path)
                ax.plot(
                    xs, ys,
                    "-",
                    color=r.color_rgb,
                    alpha=0.25,         
                    linewidth=6.0,
                    solid_capstyle="round",
                    zorder=2,
                )

        # --- 2) ПРОЕХАННАЯ ТРАЕКТОРИЯ (непрозрачная) ---
        if len(hist) > 1:
            xs, ys = zip(*hist)
            ax.plot(xs, ys, "-", color=r.color_rgb, linewidth=2.5, alpha=1.0, zorder=3)

        # --- 3) ТЕКУЩАЯ ПОЗИЦИЯ робота ---
        ax.plot(
            r.pos[0], r.pos[1],
            "s",
            color=r.color_rgb,
            markersize=12,
            markeredgecolor="black",
            markeredgewidth=1.0,
            zorder=4,
        )

        # --- 4) GOAL ---
        ax.plot(
            r.goal[0], r.goal[1],
            "x",
            color=r.color_rgb,
            markersize=12,
            markeredgewidth=3.0,
            zorder=5,
        )

    ax.set_title(title, fontsize=12)
    ax.set_xlim(-0.5, grid.shape[1] - 0.5)
    ax.set_ylim(grid.shape[0] - 0.5, -0.5)
    ax.set_aspect("equal")
    ax.axis("off")

    fig.tight_layout(pad=0.1)
    canvas = fig.canvas
    canvas.draw()
    buf = canvas.buffer_rgba()
    w, h = canvas.get_width_height()
    img = Image.frombuffer("RGBA", (w, h), buf, "raw", "RGBA", 0, 1).convert("RGB")
    plt.close(fig)
    return img



# === MAIN LOOP ===
for exp_idx, N in enumerate(exp_robot_counts, start=1):
    print(f"\n--- Experiment {exp_idx}: {N} robot(s) ---")

    gen = MapGenerator(200, 200)
    grid = gen.generate_obstacles(0.25)
    gm = GridMap(grid)
    

        # =========================
    # STARTS: уникальные, без пересечений
    # =========================
    used_starts: list[tuple[int, int]] = []
    starts: list[tuple[int, int]] = []

    for _ in range(N):
        s = sample_center_unique(
            gm,
            size=robot_size,
            clearance=clearance,
            rng=rng,
            used_centers=used_starts,
        )
        used_starts.append(s)
        starts.append(s)

    # =========================
    # GOALS: уникальные, не на стартах, рядом с партнёром
    # =========================
    used_goals: list[tuple[int, int]] = []

    def random_point_near_safe(center, max_dist, rng):

        for _ in range(2000):
            dx = int(rng.integers(-max_dist, max_dist + 1))
            dy = int(rng.integers(-max_dist, max_dist + 1))
            g = (center[0] + dx, center[1] + dy)

            if not gm.center_valid_for_robot_with_clearance(g, robot_size, clearance):
                continue
            if g in starts:
                continue
            if not center_ok(g, robot_size + 2 * clearance, used_goals):
                continue

            return g

        # fallback
        return sample_center_unique(
            gm,
            size=robot_size,
            clearance=clearance,
            rng=rng,
            used_centers=used_goals + starts,
        )

    goals: list[tuple[int, int]] = []

    for i in range(N):
        if i % 2 == 0 and i + 1 < N:
            g = random_point_near_safe(starts[i + 1], 15, rng)
        elif i % 2 == 1:
            g = random_point_near_safe(starts[i - 1], 15, rng)
        else:
            g = random_point_near_safe(starts[i], 25, rng)

        used_goals.append(g)
        goals.append(g)


    # Цвета
    colors = [plt.cm.tab10(i) for i in range(N)]
    colors = [c[:3] for c in colors]

    # Роботы
    robots_astar, robots_dstar = [], []
    for i in range(N):
        robots_astar.append(Robot(
            name=f"R{i}", start=starts[i], goal=goals[i], speed=speed,
            planner=AStarPlanner(), size=robot_size, color_rgb=colors[i], vision_radius = vision_radius
        ))
        robots_dstar.append(Robot(
            name=f"R{i}", start=starts[i], goal=goals[i], speed=speed,
            planner=DStarLitePlanner(), size=robot_size, color_rgb=colors[i], vision_radius = vision_radius
        ))

    # === A* ===
    arena_astar = Arena(map=gm)
    for r in robots_astar: arena_astar.add_robot(r)
    history_astar = [[r.pos] for r in robots_astar]
    frames_astar = []
    steps_astar = 0
    start_time = time.perf_counter()
    for i, step in tqdm(enumerate(range(max_steps)), disable = True):
        if arena_astar.all_reached():
            steps_astar = step
            break
        arena_astar.step()

        for i, r in enumerate(robots_astar):
            history_astar[i].append(r.pos)
        if step % 2 == 0:
            frames_astar.append(render_frame(grid, robots_astar, history_astar, "A*"))
    time_astar = time.perf_counter() - start_time
    if steps_astar == 0 and not arena_astar.all_reached():
        steps_astar = max_steps

    # === D* Lite ===
    arena_dstar = Arena(map=gm)
    for r in robots_dstar: arena_dstar.add_robot(r)
    history_dstar = [[r.pos] for r in robots_dstar]
    frames_dstar = []
    steps_dstar = 0
    start_time = time.perf_counter()
    for step in range(max_steps):
        if arena_dstar.all_reached():
            steps_dstar = step
            break
        arena_dstar.step()
        for i, r in enumerate(robots_dstar):
            history_dstar[i].append(r.pos)
        if step % 2 == 0:
            frames_dstar.append(render_frame(grid, robots_dstar, history_dstar, "D* Lite"))
    time_dstar = time.perf_counter() - start_time
    if steps_dstar == 0 and not arena_dstar.all_reached():
        steps_dstar = max_steps

    # === SAVE ===
    if frames_astar and frames_dstar:
        merged = merge_frames_side_by_side(frames_astar, frames_dstar)
        gif_path = f"results/exp{exp_idx}_comparison.gif"
        merged[0].save(gif_path, save_all=True, append_images=merged[1:], duration=frame_ms, loop=0)
        print(f"  Saved GIF: {gif_path}")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
    for ax, robots, history_list, title in zip([ax1, ax2], [robots_astar, robots_dstar], [history_astar, history_dstar], ["A*", "D* Lite"]):
        ax.imshow(grid, cmap="gray_r", origin="upper")
        for r, hist in zip(robots, history_list):
            if len(hist) > 1:
                xs, ys = zip(*hist)
                ax.plot(xs, ys, "-", color=r.color_rgb, linewidth=2.5)
            ax.plot(r.goal[0], r.goal[1], 'x', color=r.color_rgb, markersize=12, markeredgewidth=3)
        ax.set_title(title, fontsize=12)
        ax.set_xlim(-0.5, grid.shape[1] - 0.5)
        ax.set_ylim(grid.shape[0] - 0.5, -0.5)
        ax.set_aspect("equal")
        ax.axis('off')
    plt.tight_layout(pad=1.0)
    png_path = f"results/exp{exp_idx}_final_comparison.png"
    plt.savefig(png_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved final image: {png_path}")

    # === METRICS ===
    astar_success = sum(1 for r in robots_astar if r.pos == r.goal)
    dstar_success = sum(1 for r in robots_dstar if r.pos == r.goal)
    astar_path = sum(compute_path_length(h) for h in history_astar)
    dstar_path = sum(compute_path_length(h) for h in history_dstar)
    print(f"  A*: success={astar_success}/{N}, path={astar_path:.1f}, time={time_astar:.3f}s, steps={steps_astar}")
    print(f"  D*Lite: success={dstar_success}/{N}, path={dstar_path:.1f}, time={time_dstar:.3f}s, steps={steps_dstar}")

print("\n✅ All experiments completed! Results in 'results/' folder.")