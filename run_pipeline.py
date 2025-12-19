# run_pipeline.py
from __future__ import annotations

import os
import csv
import time
import random
from contextlib import contextmanager
from dataclasses import asdict
from typing import Callable, Dict, List, Tuple

import numpy as np

from seed_manager import SeedManager
from map_generator import MapGenerator
from grid_map import GridMap
from arena import Arena
from robot import Robot
from planners import AStarPlanner, DStarLitePlanner

from utils import sample_center_unique, center_ok
import json

# =========================
# CONFIG
# =========================
OUT_DIR = "results"
os.makedirs(OUT_DIR, exist_ok=True)

TASK_ROBOT_COUNTS = [1, 2, 2, 4, 8]
DENSITIES = [0.10, 0.20, 0.25]
N_MAPS = 10  # 10 карт на плотность => 30 экспериментов на задачу

MAP_SIZE = 150
ROBOT_SIZE = 5
CLEARANCE = 2
VISION_RADIUS = 30
SPEED = 1
MAX_STEPS = 2000

MASTER_SEED = 3  # общий seed, который задаёт ВСЁ


# =========================
# RNG isolation for MapGenerator
# =========================
@contextmanager
def temp_seed(py_seed: int, np_seed: int):
    py_state = random.getstate()
    np_state = np.random.get_state()
    random.seed(py_seed)
    np.random.seed(np_seed)
    try:
        yield
    finally:
        random.setstate(py_state)
        np.random.set_state(np_state)


def generate_map(map_seed: int, density: float) -> GridMap:
    # Важно: если MapGenerator использует random/np.random — фиксируем их только на время генерации
    with temp_seed(py_seed=map_seed, np_seed=map_seed):
        gen = MapGenerator(MAP_SIZE, MAP_SIZE)
        grid = gen.generate_obstacles(density)
    return GridMap(grid)


# =========================
# Scenario (starts/goals)
# =========================
def sample_starts_goals(
    gm: GridMap,
    n_robots: int,
    rng: np.random.Generator,
) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
    # STARTS: уникальные, без пересечений
    used_starts: List[Tuple[int, int]] = []
    starts: List[Tuple[int, int]] = []
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

    # GOALS: уникальные, не на стартах, иногда рядом с партнёром
    used_goals: List[Tuple[int, int]] = []
    eff_size = ROBOT_SIZE + 2 * CLEARANCE

    def random_point_near_safe(center: Tuple[int, int], max_dist: int) -> Tuple[int, int]:
        cx, cy = center
        for _ in range(2000):
            dx = int(rng.integers(-max_dist, max_dist + 1))
            dy = int(rng.integers(-max_dist, max_dist + 1))
            g = (cx + dx, cy + dy)

            if not gm.center_valid_for_robot_with_clearance(g, ROBOT_SIZE, CLEARANCE):
                continue
            if g in starts:
                continue
            if not center_ok(g, eff_size, used_goals):
                continue
            return g

        # fallback (гарантированно найдёт, или упадёт с ошибкой)
        return sample_center_unique(
            gm,
            size=ROBOT_SIZE,
            clearance=CLEARANCE,
            rng=rng,
            used_centers=used_goals + starts,
        )

    goals: List[Tuple[int, int]] = []
    for i in range(n_robots):
        if i % 2 == 0 and i + 1 < n_robots:
            g = random_point_near_safe(starts[i + 1], 15)
        elif i % 2 == 1:
            g = random_point_near_safe(starts[i - 1], 15)
        else:
            g = random_point_near_safe(starts[i], 25)

        used_goals.append(g)
        goals.append(g)

    return starts, goals


# =========================
# Metrics
# =========================
def path_len(points: List[Tuple[int, int]]) -> float:
    if len(points) < 2:
        return 0.0
    pts = np.asarray(points, dtype=float)
    d = np.diff(pts, axis=0)
    return float(np.sum(np.hypot(d[:, 0], d[:, 1])))


def run_one(
    algo_name: str,
    planner_factory: Callable[[], object],
    gm: GridMap,
    starts: List[Tuple[int, int]],
    goals: List[Tuple[int, int]],
    colors_seed: int,
) -> Dict:
    arena = Arena(map=gm)

    # одинаковые цвета между A* и D* для одного эксперимента
    rngc = np.random.default_rng(colors_seed)
    colors = [tuple(rngc.random(3).tolist()) for _ in range(len(starts))]

    robots: List[Robot] = []
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
    if steps == 0 and not arena.all_reached():
        steps = MAX_STEPS

    # длина пути: считаем среднюю длину на робота (чтобы сравнение N было честным)
    per_robot_paths = [path_len(h) for h in history]
    mean_path_per_robot = float(np.mean(per_robot_paths)) if per_robot_paths else 0.0

    # успех/дедлок по роботам
    success_count = sum(1 for r in robots if r.pos == r.goal)
    deadlock_count = sum(1 for r in robots if getattr(r, "deadlocked", False))

    return dict(
        algo=algo_name,
        n_robots=len(robots),
        steps=int(steps),
        runtime_sec=float(runtime),
        mean_path_per_robot=float(mean_path_per_robot),
        success_count=int(success_count),
        deadlock_count=int(deadlock_count),
    )


# =========================
# Aggregation
# =========================
def mean_std(xs: List[float]) -> Tuple[float, float]:
    if not xs:
        return 0.0, 0.0
    a = np.asarray(xs, dtype=float)
    return float(a.mean()), float(a.std(ddof=1)) if len(a) > 1 else 0.0


def aggregate_by_task(rows: List[Dict]) -> List[Dict]:
    """
    task = фиксированный N роботов (30 экспериментов: 10 карт × 3 плотности)
    """
    out = []
    # группируем по (task_n_robots, algo)
    keys = sorted(set((r["task_n_robots"], r["algo"]) for r in rows))
    for n, algo in keys:
        block = [r for r in rows if r["task_n_robots"] == n and r["algo"] == algo]

        paths = [r["mean_path_per_robot"] for r in block]
        times = [r["runtime_sec"] for r in block]

        path_mean, path_std = mean_std(paths)
        time_mean, time_std = mean_std(times)

        # успех/дедлок: считаем по роботам через все эксперименты задачи
        success_sum = sum(r["success_count"] for r in block)
        deadlock_sum = sum(r["deadlock_count"] for r in block)
        robots_total = sum(r["n_robots"] for r in block)

        success_pct = 100.0 * success_sum / robots_total if robots_total else 0.0
        deadlock_pct = 100.0 * deadlock_sum / robots_total if robots_total else 0.0

        out.append(dict(
            task_n_robots=n,
            algo=algo,
            runs=len(block),

            mean_path_per_robot_mean=path_mean,
            mean_path_per_robot_std=path_std,

            runtime_sec_mean=time_mean,
            runtime_sec_std=time_std,

            success_rate_pct=success_pct,
            deadlock_rate_pct=deadlock_pct,
        ))
    return out


def aggregate_by_density(rows: List[Dict]) -> List[Dict]:
    """
    Доп. сводка: по (N, density, algo) — удобно строить графики “качество vs плотность”.
    """
    out = []
    keys = sorted(set((r["task_n_robots"], r["density"], r["algo"]) for r in rows))
    for n, d, algo in keys:
        block = [r for r in rows if r["task_n_robots"] == n and r["density"] == d and r["algo"] == algo]

        paths = [r["mean_path_per_robot"] for r in block]
        times = [r["runtime_sec"] for r in block]
        path_mean, path_std = mean_std(paths)
        time_mean, time_std = mean_std(times)

        success_sum = sum(r["success_count"] for r in block)
        deadlock_sum = sum(r["deadlock_count"] for r in block)
        robots_total = sum(r["n_robots"] for r in block)
        success_pct = 100.0 * success_sum / robots_total if robots_total else 0.0
        deadlock_pct = 100.0 * deadlock_sum / robots_total if robots_total else 0.0

        out.append(dict(
            task_n_robots=n,
            density=float(d),
            algo=algo,
            runs=len(block),

            mean_path_per_robot_mean=path_mean,
            mean_path_per_robot_std=path_std,

            runtime_sec_mean=time_mean,
            runtime_sec_std=time_std,

            success_rate_pct=success_pct,
            deadlock_rate_pct=deadlock_pct,
        ))
    return out

def _atomic_write_text(path: str, text: str) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
    os.replace(tmp, path)


def _atomic_write_csv(path: str, rows: List[Dict]) -> None:
    if not rows:
        return
    tmp = path + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, path)


def save_all(
    raw_rows: List[Dict],
    scenarios: List[Dict],
    map_cache: Dict[Tuple[float, int], GridMap],
    sm: SeedManager,
    interrupted: bool,
) -> None:
    os.makedirs(OUT_DIR, exist_ok=True)

    suffix = "_partial" if interrupted else ""

    # 1) raw
    raw_path = os.path.join(OUT_DIR, f"raw_runs{suffix}.csv")
    _atomic_write_csv(raw_path, raw_rows)
    print("Saved:", raw_path)

    # 2) summaries (если хоть что-то есть)
    if raw_rows:
        task_summary = aggregate_by_task(raw_rows)
        task_path = os.path.join(OUT_DIR, f"summary_by_task{suffix}.csv")
        _atomic_write_csv(task_path, task_summary)
        print("Saved:", task_path)

        dens_summary = aggregate_by_density(raw_rows)
        dens_path = os.path.join(OUT_DIR, f"summary_by_density{suffix}.csv")
        _atomic_write_csv(dens_path, dens_summary)
        print("Saved:", dens_path)

    # 3) scenarios
    scenarios_path = os.path.join(OUT_DIR, f"scenarios{suffix}.json")
    payload = dict(
        meta=dict(
            master_seed=int(MASTER_SEED),
            map_size=int(MAP_SIZE),
            robot_size=int(ROBOT_SIZE),
            clearance=int(CLEARANCE),
            vision_radius=int(VISION_RADIUS),
            max_steps=int(MAX_STEPS),
            densities=[float(d) for d in DENSITIES],
            task_robot_counts=[int(n) for n in TASK_ROBOT_COUNTS],
            n_maps=int(N_MAPS),
            interrupted=bool(interrupted),
            runs_completed=int(len(raw_rows) // 2),  # 2 алгоритма на exp
        ),
        scenarios=scenarios,
    )
    _atomic_write_text(scenarios_path, json.dumps(payload, ensure_ascii=False, indent=2))
    print("Saved:", scenarios_path)

    # 4) maps (только те, что реально есть в кэше)
    maps_dir = os.path.join(OUT_DIR, f"maps_npz{suffix}")
    os.makedirs(maps_dir, exist_ok=True)

    for (density, map_idx), gm in map_cache.items():
        ms = sm.map_seed(density, map_idx)
        out_npz = os.path.join(
            maps_dir,
            f"map_d{int(round(density*100)):02d}_i{map_idx:02d}_seed{ms}.npz",
        )
        np.savez_compressed(
            out_npz,
            grid=gm.grid.astype(np.uint8),
            density=float(density),
            map_idx=int(map_idx),
            map_seed=int(ms),
        )

    print("Saved maps to:", maps_dir)


# =========================
# Main
# =========================
def main():
    sm = SeedManager(MASTER_SEED)
    raw_rows: List[Dict] = []
    scenarios: List[Dict] = []
    map_cache: Dict[Tuple[float, int], GridMap] = {}

    interrupted = False
    exp_id = 0

    try:
        for n_robots in TASK_ROBOT_COUNTS:
            for density in DENSITIES:
                for map_idx in range(1, N_MAPS + 1):
                    exp_id += 1

                    # 1) карта — одинаковая между задачами
                    key = (float(density), int(map_idx))
                    if key not in map_cache:
                        ms = sm.map_seed(density, map_idx)
                        gm = generate_map(ms, density)
                        map_cache[key] = gm
                    else:
                        gm = map_cache[key]
                        ms = sm.map_seed(density, map_idx)

                    # 2) сценарий
                    ss = sm.scenario_seed(density, map_idx, n_robots)
                    rng = np.random.default_rng(ss)
                    starts, goals = sample_starts_goals(gm, n_robots, rng)

                    # 3) цвета
                    cs = sm.colors_seed(density, map_idx, n_robots)

                    scenarios.append(dict(
                        exp_id=exp_id,
                        density=float(density),
                        map_idx=int(map_idx),
                        n_robots=int(n_robots),
                        map_seed=int(ms),
                        scenario_seed=int(ss),
                        colors_seed=int(cs),
                        starts=[list(p) for p in starts],
                        goals=[list(p) for p in goals],
                    ))

                    for algo_name, factory in [
                        ("A*", lambda: AStarPlanner()),
                        ("D*Lite", lambda: DStarLitePlanner()),
                    ]:
                        res = run_one(algo_name, factory, gm, starts, goals, cs)
                        res.update(dict(
                            exp_id=exp_id,
                            task_n_robots=n_robots,
                            density=float(density),
                            map_idx=int(map_idx),
                            map_seed=int(ms),
                            scenario_seed=int(ss),
                        ))
                        raw_rows.append(res)

                    print(f"[{exp_id:03d}] N={n_robots} density={density:.2f} map={map_idx}/10 done")

    except KeyboardInterrupt:
        interrupted = True
        print("\nInterrupted (Ctrl+C). Saving partial results...")

    finally:
        # сохраняем ВСЁ, что успели собрать
        save_all(raw_rows, scenarios, map_cache, sm, interrupted=interrupted)

        if interrupted:
            print("✅ Partial results saved.")
        else:
            print("✅ Pipeline completed. See results/ folder.")


if __name__ == "__main__":
    main()
