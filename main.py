from map_generator import MapGenerator
from grid_map import GridMap
from arena import Arena
from robot import Robot
from planners import DStarLitePlanner

import numpy as np
import matplotlib.cm as cm
import argparse
import random
from datetime import datetime


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Multi-robot arena with D* Lite and limited vision"
    )

    parser.add_argument("--map-size", type=int, default=200,
                        help="Map width and height (square map)")
    parser.add_argument("--obs-density", type=float, default=0.25,
                        help="Obstacle density in map generator")

    parser.add_argument("--n-robots", type=int, default=25,
                        help="Number of robots")
    parser.add_argument("--robot-size", type=int, default=5,
                        help="Robot square side length (cells)")
    parser.add_argument("--vision-radius", type=int, default=20,
                        help="Vision radius in cells (Manhattan distance)")

    parser.add_argument("--max-steps", type=int, default=500,
                        help="Maximum simulation steps")
    parser.add_argument("--frame-ms", type=int, default=60,
                        help="Frame duration in milliseconds for GIF")
    parser.add_argument("--gif", type=str, default="sim.gif",
                        help="Output GIF filename")

    parser.add_argument("--seed", type=int, default=0,
                        help="Random seed")
    
    parser.add_argument("--clearance", type=int, default=2,
                    help="The clearance betweem robots and obstacles")

    return parser.parse_args()


def main():
    args = parse_args()
    np.random.seed(args.seed)
    random.seed(args.seed)

    rng = np.random.default_rng(args.seed)

    gen = MapGenerator(args.map_size, args.map_size)
    grid = gen.generate_obstacles(args.obs_density)
    gm = GridMap(grid)
    arena = Arena(map=gm, clearance = args.clearance)

    cmap = cm.get_cmap("hsv", args.n_robots)

    for i in range(args.n_robots):
        s = gm.random_valid_center(args.robot_size, rng)
        g = gm.random_valid_center_not_equal(args.robot_size, s, rng)

        color = cmap(i)[:3]

        arena.add_robot(
            Robot(
                name=f"bot_{i}",
                start=s,
                goal=g,
                speed=1,
                planner=DStarLitePlanner(),
                size=args.robot_size,
                color_rgb=color,
                vision_radius=args.vision_radius,
            )
        )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    gif_name = args.gif.replace(".gif", f"_{timestamp}.gif")

    arena.start_recording(
        gif_path=gif_name,
        frame_ms=args.frame_ms,
        record_every=1,
    )

    arena.record_frame()
    try:
        for _ in range(args.max_steps):
            if arena.all_reached():
                break
            arena.step()
    except KeyboardInterrupt:
        print("\nInterrupted (Ctrl+C). Saving partial GIF...")

    # ВАЖНО: сохраняем в любом случае (если есть кадры)
    try:
        arena.stop_and_save_gif()
    except RuntimeError as e:
        # если вдруг кадров нет
        print(f"Could not save GIF: {e}")


if __name__ == "__main__":
    main()
