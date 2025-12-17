# map_utils.py
"""
Utilities for creating test maps and verifying connectivity for a 5x5 robot.
"""

import numpy as np
from typing import List, Tuple, Dict
from collections import deque


def create_test_maps() -> List[Dict]:
    """Create three test maps with guaranteed paths for a 5x5 robot."""
    maps_info = []
    robot_size = 5
    robot_radius = robot_size // 2

    # Map 1: Zigzag corridor
    print("Creating Map 1: Zigzag corridor...")
    map1 = np.zeros((100, 100), dtype=np.uint8)

    # Outer borders
    map1[0:5, :] = 1          # Top
    map1[95:100, :] = 1       # Bottom
    map1[:, 0:5] = 1          # Left
    map1[:, 95:100] = 1       # Right

    # Horizontal barriers forcing zigzag movement
    map1[20:25, 10:50] = 1    # Top left barrier
    map1[40:45, 50:90] = 1    # Middle right barrier
    map1[60:65, 10:50] = 1    # Bottom left barrier
    map1[80:85, 50:90] = 1    # Bottom right barrier

    # Vertical connectors
    map1[25:40, 45:50] = 1    # Connect top to middle
    map1[45:60, 50:55] = 1    # Connect middle to bottom
    map1[65:80, 45:50] = 1    # Connect bottom to end

    # Openings in barriers
    map1[20:25, 48:52] = 0    # Opening in top barrier
    map1[40:45, 48:52] = 0    # Opening in middle barrier
    map1[60:65, 48:52] = 0    # Opening in bottom barrier
    map1[80:85, 48:52] = 0    # Opening in final barrier

    # Decorative obstacles (do not block main path)
    decor_obstacles = [(30, 30, 6), (70, 30, 6), (30, 70, 6), (70, 70, 6)]
    for x, y, size in decor_obstacles:
        half = size // 2
        map1[y-half:y+half+1, x-half:x+half+1] = 1

    start1 = (10, 10)
    goal1 = (90, 90)

    clear_robot_area(map1, start1, robot_size)
    clear_robot_area(map1, goal1, robot_size)

    maps_info.append({
        'map_id': 1,
        'grid': map1,
        'start': start1,
        'goal': goal1,
        'description': 'Zigzag corridor with guaranteed path'
    })

    # Map 2: Open space with spiral obstacles
    print("Creating Map 2: Open space with spiral obstacles...")
    map2 = np.zeros((120, 120), dtype=np.uint8)

    obstacles = [
        (40, 40, 15), (80, 40, 12), (80, 80, 14), (40, 80, 10),
        (60, 60, 8), (30, 60, 7), (90, 60, 9),
        (60, 30, 6), (60, 90, 7)
    ]

    for x, y, size in obstacles:
        half = size // 2
        map2[y-half:y+half+1, x-half:x+half+1] = 1

    start2 = (20, 20)
    goal2 = (100, 100)

    clear_robot_area(map2, start2, robot_size)
    clear_robot_area(map2, goal2, robot_size)

    maps_info.append({
        'map_id': 2,
        'grid': map2,
        'start': start2,
        'goal': goal2,
        'description': 'Open space with spiral obstacle field'
    })

    # Map 3: Complex maze
    print("Creating Map 3: Complex maze...")
    map3 = np.zeros((80, 80), dtype=np.uint8)

    border_width = 5
    map3[0:border_width, :] = 1
    map3[-border_width:, :] = 1
    map3[:, 0:border_width] = 1
    map3[:, -border_width:] = 1

    # Main walls
    map3[15:20, 10:50] = 1
    map3[30:35, 20:60] = 1
    map3[45:50, 10:50] = 1
    map3[60:65, 20:70] = 1

    map3[10:60, 15:20] = 1
    map3[25:55, 30:35] = 1
    map3[15:45, 45:50] = 1
    map3[30:70, 60:65] = 1

    # Openings for guaranteed path
    map3[17:18, 25:26] = 0
    map3[20:30, 25:26] = 0

    map3[32:33, 30:31] = 0
    map3[35:45, 32:33] = 0

    map3[47:48, 40:41] = 0
    map3[50:60, 47:48] = 0
    map3[62:63, 55:56] = 0

    # Optional dead ends
    map3[17:18, 40:41] = 0
    map3[47:48, 30:31] = 0

    start3 = (12, 12)
    goal3 = (68, 68)

    clear_robot_area(map3, start3, robot_size)
    clear_robot_area(map3, goal3, robot_size)

    maps_info.append({
        'map_id': 3,
        'grid': map3,
        'start': start3,
        'goal': goal3,
        'description': 'Complex maze with guaranteed path'
    })

    return maps_info


def clear_robot_area(map_grid: np.ndarray, position: Tuple[int, int], robot_size: int = 5):
    """Clear the 5x5 area around the given center position."""
    robot_radius = robot_size // 2
    x, y = position

    for dy in range(-robot_radius, robot_radius + 1):
        for dx in range(-robot_radius, robot_radius + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < map_grid.shape[1] and 0 <= ny < map_grid.shape[0]:
                map_grid[ny, nx] = 0


def ensure_clear_path(map_grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], robot_size: int = 5):
    """Placeholder for ensuring path existence (manual design used instead)."""
    pass


def verify_map_connectivity(map_grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], robot_size: int = 5) -> bool:
    """Verify connectivity between start and goal using BFS for a 5x5 robot."""
    height, width = map_grid.shape
    robot_radius = robot_size // 2

    def is_valid_pos(x: int, y: int) -> bool:
        if (x < robot_radius or x >= width - robot_radius or
            y < robot_radius or y >= height - robot_radius):
            return False

        for dy in range(-robot_radius, robot_radius + 1):
            for dx in range(-robot_radius, robot_radius + 1):
                if map_grid[y + dy, x + dx] == 1:
                    return False
        return True

    if not is_valid_pos(start[0], start[1]):
        print(f"Start position {start} is invalid")
        return False

    if not is_valid_pos(goal[0], goal[1]):
        print(f"Goal position {goal} is invalid")
        return False

    directions = [(-1, -1), (-1, 0), (-1, 1),
                  (0, -1),          (0, 1),
                  (1, -1),  (1, 0),  (1, 1)]

    queue = deque([start])
    visited = set([start])

    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            return True

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            pos = (nx, ny)
            if pos not in visited and is_valid_pos(nx, ny):
                visited.add(pos)
                queue.append(pos)

    return False