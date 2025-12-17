# # multi_robot/multi_robot_runner.py
# import time
# import numpy as np
# import os
# from typing import Dict

# from a_star import AStar
# from d_star_lite import DStarLite
# from visibility_manager import VisibilityManager
# from multi_robot.scenarios import create_multi_robot_scenario
# from multi_robot.multi_robot_visualization import create_multi_robot_animation

# def is_robot_position_valid(map_grid, position, robot_size=5):
#     """Check if robot can be placed at position."""
#     robot_radius = robot_size // 2
#     x, y = position
#     height, width = map_grid.shape
    
#     if (x < robot_radius or x >= width - robot_radius or
#         y < robot_radius or y >= height - robot_radius):
#         return False
        
#     for dy in range(-robot_radius, robot_radius + 1):
#         for dx in range(-robot_radius, robot_radius + 1):
#             nx, ny = x + dx, y + dy
#             if map_grid[ny, nx] == 1:
#                 return False
#     return True

# def add_robot_as_obstacle(map_grid, robot_pos, robot_size=5):
#     """Add robot as obstacle to map."""
#     robot_radius = robot_size // 2
#     x, y = robot_pos
    
#     for dy in range(-robot_radius, robot_radius + 1):
#         for dx in range(-robot_radius, robot_radius + 1):
#             nx, ny = x + dx, y + dy
#             if 0 <= nx < map_grid.shape[1] and 0 <= ny < map_grid.shape[0]:
#                 map_grid[ny, nx] = 1

# def simulate_with_moving_obstacle(full_map, primary_start, primary_goal, 
#                                  secondary_path, algorithm_name="A*"):
#     """Simulate primary robot with moving secondary robot as dynamic obstacle."""
#     working_map = full_map.copy()
    
#     primary_pos = primary_start
#     primary_path = [primary_start]
#     secondary_step = 0
    
#     vision_radius = 12
#     replan_count = 0
#     total_time = 0.0
#     max_steps = 150  # Reduced to prevent infinite loops
    
#     # For D* Lite - keep single instance
#     dstar_instance = None
    
#     for step in range(max_steps):
#         # Update secondary robot every 12 steps (after primary has moved)
#         if secondary_step < len(secondary_path) and step >= 20 and step % 12 == 0:
#             sec_pos = secondary_path[secondary_step]
#             add_robot_as_obstacle(working_map, sec_pos)
#             secondary_step += 1
        
#         # Update primary robot's visibility
#         vis_manager = VisibilityManager(working_map, vision_radius)
#         vis_manager.update_visibility(primary_pos)
#         known_map = vis_manager.get_known_map()
        
#         # Plan path
#         if algorithm_name == "A*":
#             astar = AStar(known_map, robot_size=5)
#             start_time = time.perf_counter()
#             result = astar.find_path(primary_pos, primary_goal)
#             total_time += time.perf_counter() - start_time
#             replan_count += 1
#         else:  # D* Lite
#             if step == 0:
#                 dstar_instance = DStarLite(known_map, robot_size=5)
#                 start_time = time.perf_counter()
#                 result = dstar_instance.find_path(primary_pos, primary_goal)
#                 total_time += time.perf_counter() - start_time
#                 replan_count += 1
#             else:
#                 # Update existing D* Lite instance
#                 old_grid = dstar_instance.grid.copy()
#                 dstar_instance.grid = known_map.copy()
                
#                 # Find changed cells only if map actually changed
#                 if not np.array_equal(old_grid, known_map):
#                     changed_cells = []
#                     for y in range(known_map.shape[0]):
#                         for x in range(known_map.shape[1]):
#                             if old_grid[y, x] != known_map[y, x]:
#                                 changed_cells.append((x, y))
                    
#                     if changed_cells:
#                         dstar_instance.update_obstacles(changed_cells)
#                         start_time = time.perf_counter()
#                         result = dstar_instance.find_path(primary_pos, primary_goal)
#                         total_time += time.perf_counter() - start_time
#                         replan_count += 1
#                     else:
#                         result = {'path': dstar_instance.extract_path()}
#                 else:
#                     result = {'path': dstar_instance.extract_path()}
        
#         # Check if planning succeeded
#         if not result or not result.get('path') or len(result['path']) < 2:
#             break
        
#         # Move primary robot one step
#         next_pos = result['path'][1]
#         if is_robot_position_valid(working_map, next_pos, robot_size=5):
#             primary_path.append(next_pos)
#             primary_pos = next_pos
#         else:
#             break
        
#         if primary_pos == primary_goal:
#             break
    
#     return {
#         'path': primary_path,
#         'secondary_path': secondary_path[:secondary_step],
#         'time': total_time,
#         'replans': replan_count,
#         'success': primary_pos == primary_goal,
#         'steps': len(primary_path) - 1
#     }

# def run_multi_robot_comparison():
#     """Run comparison between A* and D* Lite with moving obstacle."""
#     print("🤖 MULTI-ROBOT COMPARISON: A* vs D* Lite")
#     print("=" * 60)
    
#     os.makedirs("multi_robot_visualizations", exist_ok=True)
    
#     scenario = create_multi_robot_scenario()
#     print(f"🗺️  Primary: {scenario['primary_start']} → {scenario['primary_goal']}")
#     print(f"   Secondary path: {len(scenario['secondary_path'])} positions")
    
#     # Test A*
#     print("\n--- Testing A* with moving obstacle ---")
#     a_result = simulate_with_moving_obstacle(
#         scenario['map'].copy(),
#         scenario['primary_start'],
#         scenario['primary_goal'],
#         scenario['secondary_path'].copy(),
#         "A*"
#     )
    
#     # Test D* Lite
#     print("\n--- Testing D* Lite with moving obstacle ---")
#     d_result = simulate_with_moving_obstacle(
#         scenario['map'].copy(),
#         scenario['primary_start'],
#         scenario['primary_goal'],
#         scenario['secondary_path'].copy(),
#         "D* Lite"
#     )
    
#     # Create visualizations
#     try:
#         final_secondary_path = secondary_path = scenario['secondary_path']
#         create_multi_robot_animation(
#             scenario['map'],
#             a_result,
#             d_result,
#             final_secondary_path,
#             filename="multi_robot_visualizations/multi_robot_comparison.gif"
#         )
#     except Exception as e:
#         print(f"⚠️ Warning: Could not create animation: {e}")
    
#     # Print results
#     print("\n" + "=" * 60)
#     print("📊 RESULTS")
#     print("=" * 60)
#     print(f"{'Algorithm':<12} {'Success':<8} {'Time(s)':<10} {'Replans':<8} {'Steps':<8}")
#     print("-" * 60)
    
#     results = []
#     for name, result in [("A*", a_result), ("D* Lite", d_result)]:
#         if result and result.get('success'):
#             print(f"{name:<12} ✅        {result['time']:<10.4f} {result['replans']:<8} {result['steps']:<8}")
#             results.append(True)
#         else:
#             print(f"{name:<12} ❌        {'-':<10} {'-':<8} {'-':<8}")
#             results.append(False)
    
#     return a_result, d_result

# multi_robot/multi_robot_runner.py
import time
import numpy as np
import os
from typing import Dict, Optional, List, Tuple

from a_star import AStar
from d_star_lite import DStarLite
from multi_robot.scenarios import create_multi_robot_scenario
from multi_robot.multi_robot_visualization import create_multi_robot_animation


def is_robot_position_valid(map_grid: np.ndarray, position: Tuple[int, int], robot_size: int = 5) -> bool:
    robot_radius = robot_size // 2
    x, y = position
    height, width = map_grid.shape
    if (x < robot_radius or x >= width - robot_radius or
        y < robot_radius or y >= height - robot_radius):
        return False
    for dy in range(-robot_radius, robot_radius + 1):
        for dx in range(-robot_radius, robot_radius + 1):
            nx, ny = x + dx, y + dy
            if map_grid[ny, nx] == 1:
                return False
    return True


def add_robot_as_obstacle(map_grid: np.ndarray, robot_pos: Tuple[int, int], robot_size: int = 5):
    robot_radius = robot_size // 2
    x, y = robot_pos
    for dy in range(-robot_radius, robot_radius + 1):
        for dx in range(-robot_radius, robot_radius + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < map_grid.shape[1] and 0 <= ny < map_grid.shape[0]:
                map_grid[ny, nx] = 1


def simulate_with_moving_obstacle(
    full_map: np.ndarray,
    primary_start: Tuple[int, int],
    primary_goal: Tuple[int, int],
    secondary_path: List[Tuple[int, int]],
    algorithm_name: str = "A*"
) -> Dict:
    """
    Simulate with a moving secondary robot.
    The secondary robot moves along its path, and the primary robot must replan when blocked.
    """
    working_map = full_map.copy()
    primary_pos = primary_start
    primary_path = [primary_start]
    secondary_step = 0
    max_steps = 100
    vision_radius = 12

    replan_count = 0
    total_time = 0.0

    # --- PHASE 1: Initial planning on empty map ---
    print(f"  [Phase 1] Initial planning on empty map...")
    if algorithm_name == "A*":
        planner = AStar(working_map, robot_size=5)
        start_time = time.perf_counter()
        result = planner.find_path(primary_start, primary_goal)
        total_time += time.perf_counter() - start_time
        replan_count += 1
        if not result:
            return _create_failure_result(primary_start, primary_goal, vision_radius, total_time, replan_count, primary_path)
        current_plan = result['path']
    else:  # D* Lite
        planner = DStarLite(working_map, robot_size=5)
        planner.initialize(primary_start, primary_goal)
        start_time = time.perf_counter()
        planner.compute_shortest_path()
        total_time += time.perf_counter() - start_time
        replan_count += 1
        current_plan = planner.extract_path()
        if not current_plan or current_plan[0] != primary_start:
            return _create_failure_result(primary_start, primary_goal, vision_radius, total_time, replan_count, primary_path)

    # --- PHASE 2: Execute plan and move secondary robot ---
    for step in range(max_steps):
        # Move secondary robot if it's time
        if secondary_step < len(secondary_path):
            sec_pos = secondary_path[secondary_step]
            # Add the secondary robot to the map
            add_robot_as_obstacle(working_map, sec_pos)
            secondary_step += 1
            print(f"  [Step {step}] Secondary robot moved to {sec_pos}")

        # Check if we need to replan
        # We replan only if the next position in our plan is now blocked
        if current_plan and len(current_plan) > 1:
            next_pos = current_plan[1]  # Next position in the plan
            if not is_robot_position_valid(working_map, next_pos, robot_size=5):
                print(f"  [Step {step}] Path is blocked at {next_pos}! Replanning...")
                if algorithm_name == "A*":
                    planner = AStar(working_map, robot_size=5)
                    start_time = time.perf_counter()
                    result = planner.find_path(primary_pos, primary_goal)
                    total_time += time.perf_counter() - start_time
                    replan_count += 1
                    if not result:
                        break
                    current_plan = result['path']
                else:  # D* Lite
                    old_grid = planner.grid.copy()
                    planner.grid = working_map.copy()
                    planner.start = primary_pos
                    changed_cells = [(x, y) for y in range(working_map.shape[0])
                                      for x in range(working_map.shape[1])
                                      if old_grid[y, x] != working_map[y, x]]
                    if changed_cells:
                        planner.update_obstacles(changed_cells)
                        start_time = time.perf_counter()
                        planner.compute_shortest_path()
                        total_time += time.perf_counter() - start_time
                        replan_count += 1
                    current_plan = planner.extract_path()
                    if not current_plan or current_plan[0] != primary_pos:
                        break

        # Move primary robot
        if current_plan and len(current_plan) > 1:
            next_pos = current_plan[1]
            if not is_robot_position_valid(working_map, next_pos, robot_size=5):
                break
            primary_path.append(next_pos)
            primary_pos = next_pos
            current_plan = current_plan[1:]  # Remove the used position from the plan
        else:
            break

        if primary_pos == primary_goal:
            break

    success = (primary_pos == primary_goal)
    return {
        'path': primary_path,
        'secondary_path': secondary_path[:secondary_step],  # All positions the secondary robot visited
        'time': total_time,
        'replans': replan_count,
        'success': success,
        'steps': len(primary_path) - 1,
        'final_pos': primary_pos,
        'start': primary_start,
        'goal': primary_goal,
        'vision_radius': vision_radius
    }


def _create_failure_result(start, goal, vision_radius, time, replans, path):
    return {
        'path': path, 'secondary_path': [], 'time': time, 'replans': replans,
        'success': False, 'steps': len(path)-1, 'final_pos': path[-1],
        'start': start, 'goal': goal, 'vision_radius': vision_radius
    }

def run_multi_robot_comparison():
    """Run comparison between A* and D* Lite with moving obstacle."""
    print("🤖 MULTI-ROBOT COMPARISON: A* vs D* Lite")
    print("=" * 60)
    
    os.makedirs("multi_robot_visualizations", exist_ok=True)
    
    scenario = create_multi_robot_scenario()
    print(f"🗺️  Primary: {scenario['primary_start']} → {scenario['primary_goal']}")
    print(f"   Secondary path: {len(scenario['secondary_path'])} positions")
    
    print("\n--- Testing A* with moving obstacle ---")
    a_result = simulate_with_moving_obstacle(
        scenario['map'].copy(),
        scenario['primary_start'],
        scenario['primary_goal'],
        scenario['secondary_path'].copy(),
        "A*"
    )
    
    print("\n--- Testing D* Lite with moving obstacle ---")
    d_result = simulate_with_moving_obstacle(
        scenario['map'].copy(),
        scenario['primary_start'],
        scenario['primary_goal'],
        scenario['secondary_path'].copy(),
        "D* Lite"
    )
    
    try:
        create_multi_robot_animation(
            scenario['map'],
            a_result,
            d_result,
            scenario['secondary_path'],
            filename="multi_robot_visualizations/multi_robot_comparison.gif"
        )
    except Exception as e:
        print(f"⚠️ Warning: Could not create animation: {e}")
    
    print("\n" + "=" * 70)
    print("📊 RESULTS")
    print("=" * 70)
    print(f"{'Algorithm':<12} {'Success':<8} {'Time(s)':<10} {'Replans':<8} {'Steps':<8}")
    print("-" * 70)
    
    for name, result in [("A*", a_result), ("D* Lite", d_result)]:
        if result['success']:
            print(f"{name:<12} ✅        {result['time']:<10.4f} {result['replans']:<8} {result['steps']:<8}")
        else:
            print(f"{name:<12} ❌        {result['time']:<10.4f} {result['replans']:<8} {result['steps']:<8}")
    
    print("\n" + "=" * 70)
    print("✅ All tests completed!")
    return a_result, d_result