import os
import time
import json
import random
import numpy as np
import pandas as pd

from map_generator import MapGenerator
from grid_map import GridMap
from arena import Arena
from robot import Robot
from planners import AStarPlanner, DStarLitePlanner

from utils import sample_center_unique, center_ok  # твои функции

# ---------------- config ----------------
OUT_DIR = "results"
os.makedirs(OUT_DIR, exist_ok=True)

TASK_N_ROBOTS = [1, 2, 4, 8]
DENSITIES = [0.10, 0.20, 0.25]
N_MAPS_PER_DENSITY = 10  # => 30 экспериментов на задачу

MAP_SIZE = 200
ROBOT_SIZE = 5
CLEARANCE = 2
VISION_RADIUS = 20
SPEED = 1
MAX_STEPS = 2000

MASTER_SEED = 3  # общий сид пайплайна

# ---------------- helpers ----------------
def compute_path_length(history):
    if len(history) < 2:
        return 0.0
    pts = np.array(history, dtype=float)
    diffs = np.diff(pts, axis=0)
    return float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))

def make_map(map_seed: int, density: float):
    # Важно: MapGenerator использует random, а твой код ещё и np.random
    np.random.seed(map_seed)
    random.seed(map_seed)

    gen = MapGenerator(MAP_SIZE, MAP_SIZE)
    grid = gen.generate_obstacles(density)
    return grid

def sample_starts_goals(gm: GridMap, n_robots: int, rng: np.random.Generator):
    # STARTS: уникальные и не пересекаются
    used_starts = []
    starts = []
    for _ in range(n_robots):
        s = sample_center_unique(
            gm,
            size=ROBOT_SIZE,
            clearance=CLEARANCE,
            rng=rng,
            used_centers=used_starts,
        )
        used_starts.append(s)
        starts.append(s)

    # GOALS: уникальные, не на стартах
    used_goals = []

    def random_point_near_safe(center, max_dist):
        for _ in range(2000):
            dx = int(rng.integers(-max_dist, max_dist + 1))
            dy = int(rng.integers(-max_dist, max_dist + 1))
            g = (center[0] + dx, center[1] + dy)

            if not gm.center_valid_for_robot_with_clearance(g, ROBOT_SIZE, CLEARANCE):
                continue
            if g in starts:
                continue
            if not center_ok(g, ROBOT_SIZE + 2 * CLEARANCE, used_goals):
                continue
            return g

        # fallback
        return sample_center_unique(
            gm,
            size=ROBOT_SIZE,
            clearance=CLEARANCE,
            rng=rng,
            used_centers=used_goals + starts,
        )

    goals = []
    for i in range(n_robots):
        # можно оставить твою логику “к партнёру”
        if i % 2 == 0 and i + 1 < n_robots:
            g = random_point_near_safe(starts[i + 1], 15)
        elif i % 2 == 1:
            g = random_point_near_safe(starts[i - 1], 15)
        else:
            g = random_point_near_safe(starts[i], 25)

        used_goals.append(g)
        goals.append(g)

    return starts, goals

def run_one_algorithm(
    algo_name: str,
    planner_factory,
    gm: GridMap,
    starts,
    goals,
    seed_for_colors: int,
):
    arena = Arena(map=gm)

    # цвета фиксируем (чтобы A* и D* были одинаковыми)
    rng_colors = np.random.default_rng(seed_for_colors)
    colors = [tuple(rng_colors.random(3).tolist()) for _ in range(len(starts))]

    robots = []
    for i, (s, g) in enumerate(zip(starts, goals)):
        r = Robot(
            name=f"R{i}",
            start=s,
            goal=g,
            speed=SPEED,
            planner=planner_factory(),
            size=ROBOT_SIZE,
            color_rgb=colors[i],
            vision_radius=VISION_RADIUS,
        )
        robots.append(r)
        arena.add_robot(r)

    history = [[r.pos] for r in robots]

    t0 = time.perf_counter()
    steps = 0
    for step in range(MAX_STEPS):
        if arena.all_reached():
            steps = step
            break
        arena.step()
        for i, r in enumerate(robots):
            history[i].append(r.pos)
    runtime = time.perf_counter() - t0

    # если не дошли до конца
    if steps == 0 and not arena.all_reached():
        steps = MAX_STEPS

    # --- metrics ---
    path_sum = sum(compute_path_length(h) for h in history)

    # visited states: нужна stats в planner (см. часть 2)
    visited_sum = 0
    for r in robots:
        if hasattr(r.planner, "stats") and hasattr(r.planner.stats, "expanded"):
            visited_sum += int(r.planner.stats.expanded)

    success_count = sum(1 for r in robots if r.pos == r.goal)
    deadlock_count = sum(1 for r in robots if getattr(r, "deadlocked", False))

    return {
        "algo": algo_name,
        "n_robots": len(robots),
        "steps": int(steps),
        "runtime_sec": float(runtime),
        "path_length_sum": float(path_sum),
        "visited_states_sum": int(visited_sum),
        "success_count": int(success_count),
        "deadlock_count": int(deadlock_count),
    }

# ---------------- main pipeline ----------------
def main():
    # общий генератор для starts/goals (чтобы воспроизводимо)
    master_rng = np.random.default_rng(MASTER_SEED)

    raw_rows = []

    exp_id = 0
    for n_robots in TASK_N_ROBOTS:
        for density in DENSITIES:
            for map_idx in range(1, N_MAPS_PER_DENSITY + 1):
                exp_id += 1

                # задаём seed карты так, чтобы он зависел от (N, density, map_idx)
                # (можно любое стабильное хеширование)
                map_seed = int(10_000 * density) * 1000 + n_robots * 100 + map_idx + MASTER_SEED * 999

                grid = make_map(map_seed, density)
                gm = GridMap(grid)

                # starts/goals для этой карты фиксируем отдельным seed
                sg_seed = map_seed + 12345
                sg_rng = np.random.default_rng(sg_seed)
                starts, goals = sample_starts_goals(gm, n_robots, sg_rng)

                # запускаем оба алгоритма на одинаковом сценарии
                for algo_name, factory in [
                    ("A*", lambda: AStarPlanner()),
                    ("D*Lite", lambda: DStarLitePlanner()),
                ]:
                    res = run_one_algorithm(
                        algo_name=algo_name,
                        planner_factory=factory,
                        gm=gm,
                        starts=starts,
                        goals=goals,
                        seed_for_colors=sg_seed,  # одинаковые цвета
                    )
                    res.update({
                        "task_n_robots": n_robots,
                        "density": density,
                        "map_idx": map_idx,
                        "map_seed": map_seed,
                        "sg_seed": sg_seed,
                        "exp_id": exp_id,
                    })
                    raw_rows.append(res)

                print(f"[{exp_id:03d}] N={n_robots} density={density:.2f} map={map_idx}/10 done")

    df = pd.DataFrame(raw_rows)
    raw_path = os.path.join(OUT_DIR, "raw_runs.csv")
    df.to_csv(raw_path, index=False)
    print(f"Saved raw runs: {raw_path}")

    # --- агрегаты по задаче и алгоритму ---
    group_cols = ["task_n_robots", "algo"]
    agg = df.groupby(group_cols).agg(
        runs=("exp_id", "count"),

        path_length_mean=("path_length_sum", "mean"),
        path_length_std=("path_length_sum", "std"),

        visited_states_mean=("visited_states_sum", "mean"),
        visited_states_std=("visited_states_sum", "std"),

        runtime_mean=("runtime_sec", "mean"),
        runtime_std=("runtime_sec", "std"),

        success_sum=("success_count", "sum"),
        deadlock_sum=("deadlock_count", "sum"),
        robots_total=("n_robots", "sum"),
    ).reset_index()

    agg["success_rate_pct"] = 100.0 * agg["success_sum"] / agg["robots_total"]
    agg["deadlock_rate_pct"] = 100.0 * agg["deadlock_sum"] / agg["robots_total"]

    summary_csv = os.path.join(OUT_DIR, "summary_by_task.csv")
    agg.to_csv(summary_csv, index=False)
    print(f"Saved summary CSV: {summary_csv}")

    summary_json = os.path.join(OUT_DIR, "summary_by_task.json")
    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(agg.to_dict(orient="records"), f, ensure_ascii=False, indent=2)
    print(f"Saved summary JSON: {summary_json}")

if __name__ == "__main__":
    main()
