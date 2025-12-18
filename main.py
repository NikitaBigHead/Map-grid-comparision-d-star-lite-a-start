from map_generator import MapGenerator
from grid_map import GridMap
from arena import Arena
from robot import Robot
from planners import DStarLitePlanner
import numpy as np
import matplotlib.cm as cm

rng = np.random.default_rng(0)

# --- общий размер для ВСЕХ роботов ---
robot_size = 10  # <-- меняй тут

gen = MapGenerator(200, 200)
grid = gen.generate_obstacles(0.3)
gm = GridMap(grid)
arena = Arena(map=gm)

N = 5
cmap = cm.get_cmap("hsv", N)

for i in range(N):
    s = gm.random_valid_center(robot_size, rng)
    g = gm.random_valid_center_not_equal(robot_size, s, rng)

    color = cmap(i)[:3]  # RGB floats in [0,1]

    arena.add_robot(
        Robot(
            name=f"bot_{i}",
            start=s,          # центр
            goal=g,           # центр
            speed=1,
            planner=DStarLitePlanner(),
            size=robot_size,  # одинаковый для всех
            color_rgb=color,
        )
    )

arena.start_recording(gif_path="sim.gif", frame_ms=60, record_every=1)
arena.record_frame()

max_steps = 500
for _ in range(max_steps):
    if arena.all_reached():
        break
    arena.step()

arena.stop_and_save_gif()
