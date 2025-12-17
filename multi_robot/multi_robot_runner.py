# multi_robot/multi_robot_runner.py
"""
Simulation and comparison of A* and D* Lite with a moving secondary robot
acting as a dynamic obstacle.
"""

import time
import numpy as np
import os
from typing import Dict, List, Tuple

from a_star import AStar
from d_star_lite import DStarLite
from visibility_manager import VisibilityManager
from multi_robot.scenarios import create_multi_robot_scenario
from multi_robot.multi_robot_visualization import create_multi_robot_animation


def is_robot_position_valid(map_grid: np.ndarray, position: Tuple[int, int], robot_size: int = 5) -> bool:
    """Check if a 5x5 robot can be placed at the given center position."""
    robot_radius = robot_size // 2
    x, y = position
    height, width = map_grid.shape

    if (x < robot_radius or x >= width - robot_radius or
        y < robot_radius or y >= height - robot_radius):
        return False

    for dy in range(-robot_radius, robot_radius + 1):
        for dx in range(-robot_radius, robot_radius + 1):
            if map_grid[y + dy, x + dx] == 1:
                return False
    return True


def add_robot_as_obstacle(map_grid: np.ndarray, robot_pos: Tuple[int, int], robot_size: int = 5):
    """Mark cells occupied by the robot as obstacles on the map."""
    robot_radius = robot_size // 2
    x, y = robot_pos
    for dy in range(-robot_radius, robot_radius + 1):
        for dx in range(-robot_radius, robot_radius + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < map_grid.shape[1] and 0 <= ny < map_grid.shape[0]:
                map_grid[ny, nx] = 1


def remove_robot_from_obstacle(map_grid: np.ndarray, robot_pos: Tuple[int, int], robot_size: int = 5):
    """Clear cells previously occupied by the robot (assuming no static obstacle underneath)."""
    robot_radius = robot_size // 2
    x, y = robot_pos
    for dy in range(-robot_radius, robot_radius + 1):
        for dx in range(-robot_radius, robot_radius + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < map_grid.shape[1] and 0 <= ny < map_grid.shape[0]:
                map_grid[ny, nx] = 0


def simulate_with_moving_obstacle(
    full_map: np.ndarray,
    primary_start: Tuple[int, int],
    primary_goal: Tuple[int, int],
    secondary_path: List[Tuple[int, int]],
    algorithm_name: str = "A*"
) -> Dict:
    """
    Simulate primary robot navigation with a moving secondary robot as dynamic obstacle.
    Limited visibility (radius 10) is used.
    """
    true_map = full_map.copy()  # true environment with moving obstacle
    primary_pos = primary_start
    primary_path = [primary_start]
    secondary_idx = 0
    current_secondary_pos = secondary_path[0]

    vision_radius = 10
    replan_count = 0
    total_time = 0.0
    max_steps = 200

    planner = None  # single instance for D* Lite

    for step in range(max_steps):
        # Move secondary robot every 15 steps of primary
        if secondary_idx < len(secondary_path) - 1 and step % 15 == 0:
            remove_robot_from_obstacle(true_map, current_secondary_pos)
            secondary_idx += 1
            current_secondary_pos = secondary_path[secondary_idx]
            add_robot_as_obstacle(true_map, current_secondary_pos)
            print(f"  [Step {step}] Secondary robot moved to {current_secondary_pos}")

        # Update primary robot visibility
        vis_manager = VisibilityManager(true_map, vision_radius)
        vis_manager.update_visibility(primary_pos)
        known_map = vis_manager.get_known_map()

        # Planning
        if algorithm_name == "A*":
            planner = AStar(known_map, robot_size=5)
            start_time = time.perf_counter()
            result = planner.find_path(primary_pos, primary_goal)
            total_time += time.perf_counter() - start_time
            replan_count += 1
        else:  # D* Lite
            if step == 0:
                planner = DStarLite(known_map, robot_size=5)
                planner.initialize(primary_pos, primary_goal)
                planner.compute_shortest_path()
                replan_count += 1
            else:
                old_grid = planner.grid.copy()
                planner.grid = known_map.copy()

                changed_cells = [(x, y) for y in range(known_map.shape[0])
                                 for x in range(known_map.shape[1])
                                 if old_grid[y, x] != known_map[y, x]]

                if changed_cells:
                    planner.move_to(primary_pos)
                    planner.update_obstacles(changed_cells)
                    start_time = time.perf_counter()
                    planner.compute_shortest_path()
                    total_time += time.perf_counter() - start_time
                    replan_count += 1

            result = {'path': planner.extract_path()}

        if not result or len(result['path']) < 2:
            print(f"  [{algorithm_name}] No path found at step {step}")
            break

        # Move primary robot
        next_pos = result['path'][1]
        if is_robot_position_valid(true_map, next_pos):
            primary_path.append(next_pos)
            primary_pos = next_pos
        else:
            print(f"  [{algorithm_name}] Collision prevented move to {next_pos}")
            break

        if primary_pos == primary_goal:
            print(f"  [{algorithm_name}] Goal reached")
            break

    success = (primary_pos == primary_goal)
    return {
        'path': primary_path,
        'secondary_path': secondary_path[:secondary_idx + 1],
        'time': total_time,
        'replans': replan_count,
        'success': success,
        'steps': len(primary_path) - 1,
        'start': primary_start,
        'goal': primary_goal,
        'vision_radius': vision_radius
    }


def run_multi_robot_comparison():
    """Run comparison between A* and D* Lite with a moving obstacle."""
    print("\nMulti-Robot Comparison: A* vs D* Lite (with limited vision)")
    print("=" * 70)

    os.makedirs("multi_robot_visualizations", exist_ok=True)

    scenario = create_multi_robot_scenario()

    print(f"Primary robot: {scenario['primary_start']} to {scenario['primary_goal']}")
    print(f"Secondary robot path length: {len(scenario['secondary_path'])}")

    print("\n--- A* (full replanning) ---")
    a_result = simulate_with_moving_obstacle(
        scenario['map'].copy(),
        scenario['primary_start'],
        scenario['primary_goal'],
        scenario['secondary_path'].copy(),
        "A*"
    )

    print("\n--- D* Lite (incremental replanning) ---")
    d_result = simulate_with_moving_obstacle(
        scenario['map'].copy(),
        scenario['primary_start'],
        scenario['primary_goal'],
        scenario['secondary_path'].copy(),
        "D* Lite"
    )

    create_multi_robot_animation(
        scenario['map'],
        a_result,
        d_result,
        scenario['secondary_path'],
        filename="multi_robot_visualizations/multi_robot_comparison.gif"
    )

    print("\nResults")
    print("=" * 70)
    print(f"{'Algorithm':<12} {'Success':<10} {'Time(s)':<10} {'Replans':<10} {'Steps':<8}")
    print("-" * 70)
    for name, res in [("A*", a_result), ("D* Lite", d_result)]:
        status = "Success" if res['success'] else "Failed"
        print(f"{name:<12} {status:<10} {res['time']:<10.4f} {res['replans']:<10} {res['steps']:<8}")

    print("\nMulti-robot test completed")