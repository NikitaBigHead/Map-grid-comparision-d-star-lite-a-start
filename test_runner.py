# """
# Test execution and algorithm comparison with Robot class visualization.
# """

# import time
# import numpy as np
# from typing import List, Tuple, Dict
# import os

# from map_utils import create_test_maps, verify_map_connectivity
# from visualization import create_robot_animation_with_vision, create_robot_comparison_plot, demonstrate_robot_vision
# from a_star import AStar
# from d_star_lite import DStarLite

# def run_single_test(map_grid: np.ndarray, map_id: int, 
#                    start: Tuple[int, int], goal: Tuple[int, int],
#                    description: str) -> Dict:
#     """Run A* and D* Lite tests on a single map."""
#     print(f"\n{'='*60}")
#     print(f"🗺️  MAP {map_id}: {description}")
#     print(f"{'='*60}")
    
#     # Verify connectivity
#     print("🔍 Verifying map connectivity...")
#     if not verify_map_connectivity(map_grid, start, goal, robot_size=5):
#         print("❌ No path exists between start and goal!")
#         return None
    
#     print("✅ Map is connected")
#     print(f"   Start: {start}, Goal: {goal}")
#     print(f"   Map size: {map_grid.shape}")
    
#     results = {}
    
#     # Test A*
#     print("\n--- Testing A* ---")
#     a_result = test_algorithm(AStar, map_grid, start, goal, "A*")
#     if a_result:
#         results['A*'] = a_result
#     else:
#         results['A*'] = None
    
#     # Test D* Lite
#     print("\n--- Testing D* Lite ---")
#     d_result = test_algorithm(DStarLite, map_grid, start, goal, "D* Lite")
#     if d_result:
#         results['D* Lite'] = d_result
#     else:
#         results['D* Lite'] = None
    
#     # Create visualizations with Robot class
#     if results['A*'] or results['D* Lite']:
#         create_visualizations_with_robot(map_grid, results, map_id, start, goal, description)
    
#     return results

# def test_algorithm(algorithm_class, map_grid: np.ndarray, 
#                   start: Tuple[int, int], goal: Tuple[int, int],
#                   algorithm_name: str) -> Dict:
#     """Test a single pathfinding algorithm."""
#     try:
#         algorithm = algorithm_class(map_grid, robot_size=5)
#         start_time = time.perf_counter()
#         result = algorithm.find_path(start, goal)
#         elapsed_time = time.perf_counter() - start_time
        
#         if result and result['path']:
#             print(f"✅ Found path!")
#             print(f"   Length: {result['path_length']} steps")
#             print(f"   Cost: {result['total_cost']:.4f}")
#             print(f"   Time: {elapsed_time:.4f}s")
#             print(f"   Visited nodes: {result['visited_count']}")
            
#             return {
#                 'path': result['path'],
#                 'time': elapsed_time,
#                 'visited': result['visited_count'],
#                 'length': result['path_length'],
#                 'cost': result['total_cost']
#             }
#         else:
#             print("❌ No path found")
#             return None
            
#     except Exception as e:
#         print(f"❌ Error testing {algorithm_name}: {e}")
#         return None

# def create_visualizations_with_robot(map_grid: np.ndarray, results: Dict, map_id: int,
#                                    start: Tuple[int, int], goal: Tuple[int, int],
#                                    description: str):
#     """Create visualizations for test results using Robot class."""
#     print("\n🎨 Creating visualizations with Robot class...")
    
#     # Create animations with robot vision
#     vision_radius = 10  # Robot vision radius from robot_class
    
#     if results['A*']:
#         print("  Creating A* animation with robot vision...")
#         create_robot_animation_with_vision(
#             map_grid, results['A*']['path'], start, goal,
#             title=f"Map {map_id}",
#             filename=f"robot_visualizations/map{map_id}_a_star.gif",
#             algorithm_name="A*",
#             vision_radius=vision_radius
#         )
    
#     if results['D* Lite']:
#         print("  Creating D* Lite animation with robot vision...")
#         create_robot_animation_with_vision(
#             map_grid, results['D* Lite']['path'], start, goal,
#             title=f"Map {map_id}",
#             filename=f"robot_visualizations/map{map_id}_d_star_lite.gif",
#             algorithm_name="D* Lite",
#             vision_radius=vision_radius
#         )
    
#     # Create comparison plot with robot visualization
#     print("  Creating robot comparison plot...")
#     paths_dict = {}
#     if results['A*']:
#         paths_dict['A*'] = results['A*']['path']
#     if results['D* Lite']:
#         paths_dict['D* Lite'] = results['D* Lite']['path']
    
#     if paths_dict:
#         create_robot_comparison_plot(
#             map_grid, paths_dict, start, goal,
#             title=f"Map {map_id}: {description}",
#             filename=f"robot_visualizations/map{map_id}_comparison.png",
#             vision_radius=vision_radius
#         )
    
#     # Optional: Create robot vision demonstration (only for first map)
#     if map_id == 1 and results.get('A*'):
#         print("  Creating robot vision demonstration...")
#         demonstrate_robot_vision(
#             map_grid, start, goal,
#             filename=f"map{map_id}_vision_demo.gif"
#         )
    
#     # Compare results if both found paths
#     if results['A*'] and results['D* Lite']:
#         compare_results(results)

# def compare_results(results: Dict):
#     """Compare A* and D* Lite results."""
#     a_path = results['A*']['path']
#     d_path = results['D* Lite']['path']
    
#     a_set = set(a_path)
#     d_set = set(d_path)
#     intersection = a_set.intersection(d_set)
#     union = a_set.union(d_set)
#     similarity = len(intersection) / len(union) if union else 0
    
#     print(f"\n📊 Comparison:")
#     print(f"   Path similarity: {similarity:.1%}")
#     print(f"   Length: A*={len(a_path)-1}, D* Lite={len(d_path)-1}")
#     print(f"   Cost: A*={results['A*']['cost']:.4f}, D* Lite={results['D* Lite']['cost']:.4f}")
#     print(f"   Time: A*={results['A*']['time']:.4f}s, D* Lite={results['D* Lite']['time']:.4f}s")

# def run_comparison_tests():
#     """Run complete comparison tests."""
#     # Create output directory
#     os.makedirs("robot_visualizations", exist_ok=True)
    
#     # Create test maps
#     print("Creating test maps...")
#     maps_info = create_test_maps()
    
#     print(f"\n✅ Created {len(maps_info)} test maps")
#     for map_info in maps_info:
#         print(f"  Map {map_info['map_id']}: {map_info['description']}")
#         print(f"     Size: {map_info['grid'].shape}, Start: {map_info['start']}, Goal: {map_info['goal']}")
    
#     print("\n" + "=" * 70)
#     print("🤖 Robot Specifications:")
#     print("   - Size: 5×5 cells")
#     print("   - Vision radius: 10 cells")
#     print("   - Algorithms consider full 5×5 footprint")
#     print("=" * 70)
    
#     # Run tests
#     all_results = []
#     for map_info in maps_info:
#         results = run_single_test(
#             map_info['grid'],
#             map_info['map_id'],
#             map_info['start'],
#             map_info['goal'],
#             map_info['description']
#         )
        
#         if results:
#             all_results.append({
#                 'map_id': map_info['map_id'],
#                 'results': results
#             })
    
#     # Print summary
#     if all_results:
#         print_summary(all_results)

# def print_summary(all_results: List[Dict]):
#     """Print test summary."""
#     print(f"\n{'='*70}")
#     print("📈 FINAL SUMMARY")
#     print(f"{'='*70}")
#     print(f"{'Map':<6} {'Algorithm':<12} {'Found':<8} {'Time(s)':<10} {'Length':<8} {'Cost':<12}")
#     print("-" * 70)
    
#     successful_tests = 0
#     for map_result in all_results:
#         map_id = map_result['map_id']
#         results = map_result['results']
        
#         for algo in ['A*', 'D* Lite']:
#             if algo in results and results[algo]:
#                 r = results[algo]
#                 print(f"{map_id:<6} {algo:<12} {'✅':<8} {r['time']:<10.4f} {r['length']:<8} {r['cost']:<12.4f}")
#                 successful_tests += 1
#             else:
#                 print(f"{map_id:<6} {algo:<12} {'❌':<8} {'-':<10} {'-':<8} {'-':<12}")
    
#     # Statistics
#     total_tests = len(all_results) * 2
#     if total_tests > 0:
#         success_rate = successful_tests / total_tests * 100
#     else:
#         success_rate = 0
    
#     print(f"\n📊 Statistics:")
#     print(f"   Total tests: {total_tests}")
#     print(f"   Successful: {successful_tests}")
#     print(f"   Success rate: {success_rate:.1f}%")
    
#     # Calculate averages for successful tests
#     a_star_times = []
#     a_star_lengths = []
#     a_star_costs = []
    
#     d_star_times = []
#     d_star_lengths = []
#     d_star_costs = []
    
#     for map_result in all_results:
#         results = map_result['results']
        
#         if 'A*' in results and results['A*']:
#             a_star_times.append(results['A*']['time'])
#             a_star_lengths.append(results['A*']['length'])
#             a_star_costs.append(results['A*']['cost'])
        
#         if 'D* Lite' in results and results['D* Lite']:
#             d_star_times.append(results['D* Lite']['time'])
#             d_star_lengths.append(results['D* Lite']['length'])
#             d_star_costs.append(results['D* Lite']['cost'])
    
#     if a_star_times:
#         print(f"\n📊 A* Performance:")
#         print(f"   Avg time: {np.mean(a_star_times):.4f}s")
#         print(f"   Avg path length: {np.mean(a_star_lengths):.1f}")
#         print(f"   Avg cost: {np.mean(a_star_costs):.4f}")
    
#     if d_star_times:
#         print(f"\n📊 D* Lite Performance:")
#         print(f"   Avg time: {np.mean(d_star_times):.4f}s")
#         print(f"   Avg path length: {np.mean(d_star_lengths):.1f}")
#         print(f"   Avg cost: {np.mean(d_star_costs):.4f}")

# if __name__ == "__main__":
#     # Can be run directly for testing
#     run_comparison_tests()

# test_runner.py
"""
Test execution and algorithm comparison with limited visibility (10 cells) for a single robot.
"""

import time
import numpy as np
from typing import List, Tuple, Dict
import os

from map_utils import create_test_maps, verify_map_connectivity
from a_star import AStar
from d_star_lite import DStarLite
from visibility_manager import VisibilityManager
from visualization import create_single_robot_animation


def is_robot_position_valid(map_grid: np.ndarray, position: Tuple[int, int], robot_size: int = 5) -> bool:
    """Check if the 5x5 robot can be placed at the given center position."""
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


def run_single_test(map_grid: np.ndarray, map_id: int,
                   start: Tuple[int, int], goal: Tuple[int, int],
                   description: str) -> Dict:
    print(f"\n{'=' * 70}")
    print(f"MAP {map_id}: {description} (vision radius = 10)")
    print(f"{'=' * 70}")

    if not verify_map_connectivity(map_grid, start, goal, robot_size=5):
        print("No path exists even on the full map!")
        return None

    print("Map is connected")
    print(f"   Start: {start}, Goal: {goal}, Size: {map_grid.shape}")

    results = {}
    vision_radius = 10

    print("\n--- A* (full replanning on obstacle discovery) ---")
    a_result = run_astar_with_visibility(map_grid, start, goal, vision_radius)
    results['A*'] = a_result

    print("\n--- D* Lite (incremental replanning) ---")
    d_result = run_dstar_with_visibility(map_grid, start, goal, vision_radius)
    results['D* Lite'] = d_result

    # Visualization of both algorithms
    try:
        create_single_robot_animation(
            map_grid,
            a_result['path'] if a_result['success'] else [],
            d_result['path'] if d_result['success'] else [],
            start, goal, map_id, vision_radius
        )
    except Exception as e:
        print(f"Visualization failed: {e}")

    return results


def run_astar_with_visibility(full_map: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], vision_radius: int) -> Dict:
    """A* with limited visibility: full replan after each visibility update."""
    vis_manager = VisibilityManager(full_map, vision_radius)
    robot_pos = start
    path = [start]
    total_time = 0.0
    replan_count = 0

    while robot_pos != goal:
        vis_manager.update_visibility(robot_pos)
        known_map = vis_manager.get_known_map()

        astar = AStar(known_map, robot_size=5)
        start_time = time.perf_counter()
        result = astar.find_path(robot_pos, goal)
        total_time += time.perf_counter() - start_time
        replan_count += 1

        if not result or len(result['path']) < 2:
            break

        next_pos = result['path'][1]
        if is_robot_position_valid(full_map, next_pos):
            path.append(next_pos)
            robot_pos = next_pos
        else:
            break

    success = (robot_pos == goal)
    return {
        'path': path,
        'time': total_time,
        'replans': replan_count,
        'success': success,
        'length': len(path) - 1,
        'cost': len(path) - 1
    }


def run_dstar_with_visibility(full_map: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], vision_radius: int) -> Dict:
    """D* Lite with limited visibility: single instance with incremental updates."""
    vis_manager = VisibilityManager(full_map, vision_radius)
    vis_manager.update_visibility(start)

    dstar = DStarLite(vis_manager.get_known_map(), robot_size=5)
    dstar.initialize(start, goal)
    dstar.compute_shortest_path()  # Critical: run search after initialization

    robot_pos = start
    path = [start]
    total_time = 0.0
    replan_count = 1  # initial planning

    while robot_pos != goal:
        vis_manager.update_visibility(robot_pos)
        known_map = vis_manager.get_known_map()

        old_grid = dstar.grid.copy()
        dstar.grid = known_map.copy()

        changed_cells = [(x, y) for y in range(known_map.shape[0])
                         for x in range(known_map.shape[1])
                         if old_grid[y, x] != known_map[y, x]]

        if changed_cells:
            dstar.move_to(robot_pos)
            dstar.update_obstacles(changed_cells)
            start_time = time.perf_counter()
            dstar.compute_shortest_path()
            total_time += time.perf_counter() - start_time
            replan_count += 1

        current_plan = dstar.extract_path()
        if len(current_plan) < 2:
            break

        next_pos = current_plan[1]
        if is_robot_position_valid(full_map, next_pos):
            path.append(next_pos)
            robot_pos = next_pos
        else:
            break

    success = (robot_pos == goal)
    return {
        'path': path,
        'time': total_time,
        'replans': replan_count,
        'success': success,
        'length': len(path) - 1,
        'cost': len(path) - 1
    }


def run_comparison_tests():
    """Run all single-robot tests."""
    os.makedirs("robot_visualizations_blind", exist_ok=True)

    maps_info = create_test_maps()

    print(f"\nCreated {len(maps_info)} test maps")
    for m in maps_info:
        print(f"   Map {m['map_id']}: {m['description']}")

    print("\n" + "=" * 70)
    print("Robot Specifications:")
    print("   - Size: 5x5 cells")
    print("   - Vision radius: 10 cells")
    print("   - A* - full replanning")
    print("   - D* Lite - incremental replanning")
    print("=" * 70)

    all_results = []
    for map_info in maps_info:
        results = run_single_test(
            map_info['grid'],
            map_info['map_id'],
            map_info['start'],
            map_info['goal'],
            map_info['description']
        )
        if results:
            all_results.append({'map_id': map_info['map_id'], 'results': results})

    # Final table
    print(f"\n{'=' * 70}")
    print("FINAL SUMMARY (single robot, limited visibility)")
    print(f"{'=' * 70}")
    print(f"{'Map':<6} {'Algorithm':<12} {'Success':<8} {'Time(s)':<12} {'Replans':<10} {'Length':<8}")
    print("-" * 80)

    successful = 0
    total_tests = len(all_results) * 2

    for res in all_results:
        map_id = res['map_id']
        for algo in ['A*', 'D* Lite']:
            r = res['results'][algo]
            status = "Success" if r['success'] else "Failed"
            print(f"{map_id:<6} {algo:<12} {status:<8} {r['time']:<12.4f} {r['replans']:<10} {r['length']:<8}")
            if r['success']:
                successful += 1

    print(f"\nSuccessful tests: {successful}/{total_tests} ({successful / total_tests * 100:.1f}%)")


if __name__ == "__main__":
    run_comparison_tests()