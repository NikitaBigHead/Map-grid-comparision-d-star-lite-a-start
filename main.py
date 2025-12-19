from map_generator import MapGenerator
from grid_map import GridMap
from arena import Arena
from robot import Robot
from planners import DStarLitePlanner, AStarPlanner

import numpy as np
import matplotlib.cm as cm
import argparse
import random
from datetime import datetime
from tqdm import tqdm
from utils import center_ok, sample_center_unique




def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Multi-robot arena with D* Lite and limited vision"
    )

    parser.add_argument("--map-size", type=int, default=200,
                        help="Map width and height (square map)")
    
    parser.add_argument("--obs-density", type=float, default=0.25,
                        help="Obstacle density in map generator")

    parser.add_argument("--n-robots", type=int, default=5,
                        help="Number of robots")
    
    parser.add_argument("--robot-size", type=int, default=5,
                        help="Robot square side length (cells)")
    
    parser.add_argument("--vision-radius", type=int, default=30,
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
    
    parser.add_argument(
        "--planner",
        type=str,
        choices=["dstar", "astar"],
        default="dstar",
        help="Planner type: dstar or astar",
    )


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

    used_starts: list[tuple[int, int]] = []
    used_goals: list[tuple[int, int]] = []

    for i in range(args.n_robots):
        s = sample_center_unique(gm, args.robot_size, args.clearance, rng, used_starts)
        used_starts.append(s)

        g = sample_center_unique(gm, args.robot_size, args.clearance, rng, used_goals)
        while g == s:
            g = sample_center_unique(gm, args.robot_size, args.clearance, rng, used_goals)

        used_goals.append(g)

        color = cmap(i)[:3]

        arena.add_robot(
            Robot(
                name=f"bot_{i}",
                start=s,
                goal=g,
                speed=1,
                planner = DStarLitePlanner() if args.planner == "dstar" else AStarPlanner(),
                size=args.robot_size,
                color_rgb=color,
                vision_radius=args.vision_radius,
            )
        )
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    planner_tag = "DStarLite" if args.planner == "dstar" else "AStar"
    obs_pct = int(round(args.obs_density * 100))

    # base name без .gif
    base = args.gif[:-4] if args.gif.lower().endswith(".gif") else args.gif

    gif_name = f"{base}_{planner_tag}_N{args.n_robots}_OBS{obs_pct}_SEED{args.seed}_{timestamp}.gif"
    title = f"{planner_tag} | N={args.n_robots} | obs={obs_pct}% | seed={args.seed}"



    arena.start_recording(
        gif_path=gif_name,
        frame_ms=args.frame_ms,
        record_every=1,
        title = title
    )

    arena.record_frame()
    try:
        for _ in tqdm(range(args.max_steps)):
            if arena.all_reached():
                break
            arena.step()
    except KeyboardInterrupt:
        print("\nInterrupted (Ctrl+C). Saving partial GIF...")

    try:
        arena.stop_and_save_gif()
    except RuntimeError as e:
        # если вдруг кадров нет
        print(f"Could not save GIF: {e}")


if __name__ == "__main__":
    main()
