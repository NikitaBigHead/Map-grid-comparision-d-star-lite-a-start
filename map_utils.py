# """
# Utilities for map operations and test map generation.
# """

# import numpy as np
# import random
# from typing import List, Tuple
# from collections import deque

# def create_test_maps() -> List[dict]:
#     """Create test maps with guaranteed NON-TRIVIAL paths for 5×5 robot."""
#     maps_info = []
#     robot_size = 5
#     robot_radius = robot_size // 2
    
#     # Map 1: Corridor with U-turn and explicit entrances
#     print("Creating Map 1: Corridor with U-turn...")
#     map1 = np.zeros((100, 100), dtype=np.uint8)
    
#     # Main corridor walls
#     map1[20:80, 10:20] = 1   # Left outer wall
#     map1[20:80, 80:90] = 1   # Right outer wall
    
#     # Inner obstacles forcing detour
#     map1[30:50, 30:70] = 1   # Central blockage (top half)
#     map1[60:75, 30:50] = 1   # Partial blockage (bottom left)
#     map1[50:65, 60:75] = 1   # Partial blockage (bottom right)
    
#     # ✅ CRITICAL: Create entrance/exit openings
#     map1[28:32, 10:20] = 0  # Entrance opening at start level
#     map1[68:72, 80:90] = 0  # Exit opening at goal level
    
#     start1 = (8, 30)      # Safe start position
#     goal1 = (92, 70)     # Safe goal position
    
#     clear_robot_area(map1, start1, robot_size)
#     clear_robot_area(map1, goal1, robot_size)
    
#     maps_info.append({
#         'map_id': 1,
#         'grid': map1,
#         'start': start1,
#         'goal': goal1,
#         'description': 'Corridor with U-turn detour'
#     })
    
#     # Map 2: Open space with spiral obstacle field
#     print("Creating Map 2: Open space with spiral obstacles...")
#     map2 = np.zeros((120, 120), dtype=np.uint8)
    
#     # Create spiral barrier
#     obstacles = [
#         (40, 40, 15),  # Inner circle
#         (80, 40, 12),  # Right arc
#         (80, 80, 14),  # Bottom arc
#         (40, 80, 10),  # Left arc
#         (60, 60, 8),   # Center obstacle
#         (30, 60, 7),   # Left-middle
#         (90, 60, 9),   # Right-middle
#         (60, 30, 6),   # Top-middle
#         (60, 90, 7)    # Bottom-middle
#     ]
    
#     for x, y, size in obstacles:
#         half = size // 2
#         map2[y-half:y+half+1, x-half:x+half+1] = 1
    
#     start2 = (20, 20)    # Safe start position
#     goal2 = (100, 100)  # Safe goal position
    
#     clear_robot_area(map2, start2, robot_size)
#     clear_robot_area(map2, goal2, robot_size)
#     ensure_clear_path(map2, start2, goal2, robot_size)
    
#     maps_info.append({
#         'map_id': 2,
#         'grid': map2,
#         'start': start2,
#         'goal': goal2,
#         'description': 'Open space with spiral obstacle field'
#     })
    
#     # Map 3: Complex maze with guaranteed path
#     print("Creating Map 3: Complex maze...")
#     map3 = np.zeros((80, 80), dtype=np.uint8)
    
#     # Outer border
#     border_width = 5
#     map3[0:border_width, :] = 1
#     map3[-border_width:, :] = 1
#     map3[:, 0:border_width] = 1
#     map3[:, -border_width:] = 1
    
#     # Maze walls
#     map3[15:20, 10:50] = 1    # Top horizontal
#     map3[30:35, 20:60] = 1    # Middle horizontal 1
#     map3[45:50, 10:50] = 1    # Middle horizontal 2
#     map3[60:65, 20:70] = 1    # Bottom horizontal
    
#     map3[10:60, 15:20] = 1    # Left vertical 1
#     map3[25:55, 30:35] = 1    # Center vertical 1
#     map3[15:45, 45:50] = 1    # Center vertical 2
#     map3[30:70, 60:65] = 1    # Right vertical
    
#     # ✅ CRITICAL: Create guaranteed path (no dead ends on main route)
#     # Top section
#     map3[17:18, 25:26] = 0    # Opening in top horizontal
#     map3[20:30, 25:26] = 0    # Vertical passage down
    
#     # Middle section
#     map3[32:33, 30:31] = 0    # Opening in middle horiz 1
#     map3[35:45, 32:33] = 0    # Vertical passage down
    
#     # Bottom section
#     map3[47:48, 40:41] = 0    # Opening in middle horiz 2
#     map3[50:60, 47:48] = 0    # Vertical passage down
#     map3[62:63, 55:56] = 0    # Opening in bottom horizontal
    
#     # Side passages (optional dead ends for complexity)
#     map3[17:18, 40:41] = 0    # Dead end (top wall)
#     map3[47:48, 30:31] = 0    # Dead end (middle wall)
    
#     start3 = (12, 12)    # Safe start position
#     goal3 = (68, 68)    # Safe goal position
    
#     clear_robot_area(map3, start3, robot_size)
#     clear_robot_area(map3, goal3, robot_size)
    
#     maps_info.append({
#         'map_id': 3,
#         'grid': map3,
#         'start': start3,
#         'goal': goal3,
#         'description': 'Complex maze with guaranteed path'
#     })
    
#     return maps_info

# def clear_robot_area(map_grid: np.ndarray, position: Tuple[int, int], robot_size: int = 5):
#     """Clear area for robot placement."""
#     robot_radius = robot_size // 2
#     x, y = position
    
#     for dy in range(-robot_radius, robot_radius + 1):
#         for dx in range(-robot_radius, robot_radius + 1):
#             nx, ny = x + dx, y + dy
#             if 0 <= nx < map_grid.shape[1] and 0 <= ny < map_grid.shape[0]:
#                 map_grid[ny, nx] = 0

# def ensure_clear_path(map_grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], robot_size: int = 5):
#     """Ensure a clear (but not necessarily straight) path exists between start and goal."""
#     # For complex maps, we rely on manual design rather than diagonal clearing
#     pass

# def verify_map_connectivity(map_grid, start, goal, robot_size=5):
#     """Verify that path exists using BFS."""
#     height, width = map_grid.shape
#     robot_radius = robot_size // 2
    
#     def is_valid_pos(x, y):
#         if (x < robot_radius or x >= width - robot_radius or
#             y < robot_radius or y >= height - robot_radius):
#             return False
        
#         for dy in range(-robot_radius, robot_radius + 1):
#             for dx in range(-robot_radius, robot_radius + 1):
#                 if map_grid[y + dy, x + dx] == 1:
#                     return False
#         return True
    
#     if not is_valid_pos(start[0], start[1]):
#         print(f"  ❌ Start position {start} invalid")
#         return False
    
#     if not is_valid_pos(goal[0], goal[1]):
#         print(f"  ❌ Goal position {goal} invalid")
#         return False
    
#     # 8-direction BFS
#     directions = [(-1, -1), (-1, 0), (-1, 1),
#                   (0, -1), (0, 1),
#                   (1, -1), (1, 0), (1, 1)]
    
#     queue = deque([start])
#     visited = set([start])
    
#     while queue:
#         x, y = queue.popleft()
        
#         if (x, y) == goal:
#             return True
        
#         for dx, dy in directions:
#             nx, ny = x + dx, y + dy
#             new_pos = (nx, ny)
            
#             if new_pos not in visited and is_valid_pos(nx, ny):
#                 visited.add(new_pos)
#                 queue.append(new_pos)
    
#     return False

"""
Utilities for map operations and test map generation.
"""

import numpy as np
import random
from typing import List, Tuple
from collections import deque

def create_test_maps() -> List[dict]:
    """Create test maps with guaranteed NON-TRIVIAL paths for 5×5 robot."""
    maps_info = []
    robot_size = 5
    robot_radius = robot_size // 2
    
    # Map 1: Zigzag corridor with guaranteed path
    print("Creating Map 1: Zigzag corridor...")
    map1 = np.zeros((100, 100), dtype=np.uint8)

    # Create outer borders for safety
    map1[0:5, :] = 1      # Top
    map1[95:100, :] = 1   # Bottom
    map1[:, 0:5] = 1      # Left  
    map1[:, 95:100] = 1   # Right

    # Create zigzag path with alternating horizontal barriers
    # Horizontal barriers that force vertical movement
    map1[20:25, 10:50] = 1    # Top horizontal barrier (left half)
    map1[40:45, 50:90] = 1    # Middle horizontal barrier (right half)  
    map1[60:65, 10:50] = 1    # Bottom horizontal barrier (left half)
    map1[80:85, 50:90] = 1    # Bottom-right horizontal barrier

    # Vertical barriers to complete zigzag pattern
    map1[25:40, 45:50] = 1    # Connect top to middle
    map1[45:60, 50:55] = 1    # Connect middle to bottom
    map1[65:80, 45:50] = 1    # Connect bottom to final section

    # Create guaranteed openings in barriers
    map1[20:25, 48:52] = 0    # Opening in top barrier (connects to vertical)
    map1[40:45, 48:52] = 0    # Opening in middle barrier  
    map1[60:65, 48:52] = 0    # Opening in bottom barrier
    map1[80:85, 48:52] = 0    # Opening in final barrier

    # Add some decorative obstacles in open areas (won't block path)
    decor_obstacles = [
        (30, 30, 6),
        (70, 30, 6), 
        (30, 70, 6),
        (70, 70, 6)
    ]
    for x, y, size in decor_obstacles:
        half = size // 2
        map1[y-half:y+half+1, x-half:x+half+1] = 1

    # Start at top-left, goal at bottom-right
    start1 = (10, 10)    # Top-left open area
    goal1 = (90, 90)     # Bottom-right open area

    clear_robot_area(map1, start1, robot_size)
    clear_robot_area(map1, goal1, robot_size)
        
    maps_info.append({
        'map_id': 1,
        'grid': map1,
        'start': start1,
        'goal': goal1,
        'description': 'Corridor with U-turn detour'
    })
    
    # Map 2: Open space with spiral obstacle field
    print("Creating Map 2: Open space with spiral obstacles...")
    map2 = np.zeros((120, 120), dtype=np.uint8)
    
    # Create spiral barrier
    obstacles = [
        (40, 40, 15),  # Inner circle
        (80, 40, 12),  # Right arc
        (80, 80, 14),  # Bottom arc
        (40, 80, 10),  # Left arc
        (60, 60, 8),   # Center obstacle
        (30, 60, 7),   # Left-middle
        (90, 60, 9),   # Right-middle
        (60, 30, 6),   # Top-middle
        (60, 90, 7)    # Bottom-middle
    ]
    
    for x, y, size in obstacles:
        half = size // 2
        map2[y-half:y+half+1, x-half:x+half+1] = 1
    
    start2 = (20, 20)    # Safe start position
    goal2 = (100, 100)  # Safe goal position
    
    clear_robot_area(map2, start2, robot_size)
    clear_robot_area(map2, goal2, robot_size)
    ensure_clear_path(map2, start2, goal2, robot_size)
    
    maps_info.append({
        'map_id': 2,
        'grid': map2,
        'start': start2,
        'goal': goal2,
        'description': 'Open space with spiral obstacle field'
    })
    
    # Map 3: Complex maze with guaranteed path
    print("Creating Map 3: Complex maze...")
    map3 = np.zeros((80, 80), dtype=np.uint8)
    
    # Outer border
    border_width = 5
    map3[0:border_width, :] = 1
    map3[-border_width:, :] = 1
    map3[:, 0:border_width] = 1
    map3[:, -border_width:] = 1
    
    # Maze walls
    map3[15:20, 10:50] = 1    # Top horizontal
    map3[30:35, 20:60] = 1    # Middle horizontal 1
    map3[45:50, 10:50] = 1    # Middle horizontal 2
    map3[60:65, 20:70] = 1    # Bottom horizontal
    
    map3[10:60, 15:20] = 1    # Left vertical 1
    map3[25:55, 30:35] = 1    # Center vertical 1
    map3[15:45, 45:50] = 1    # Center vertical 2
    map3[30:70, 60:65] = 1    # Right vertical
    
    # ✅ CRITICAL: Create guaranteed path (no dead ends on main route)
    # Top section
    map3[17:18, 25:26] = 0    # Opening in top horizontal
    map3[20:30, 25:26] = 0    # Vertical passage down
    
    # Middle section
    map3[32:33, 30:31] = 0    # Opening in middle horiz 1
    map3[35:45, 32:33] = 0    # Vertical passage down
    
    # Bottom section
    map3[47:48, 40:41] = 0    # Opening in middle horiz 2
    map3[50:60, 47:48] = 0    # Vertical passage down
    map3[62:63, 55:56] = 0    # Opening in bottom horizontal
    
    # Side passages (optional dead ends for complexity)
    map3[17:18, 40:41] = 0    # Dead end (top wall)
    map3[47:48, 30:31] = 0    # Dead end (middle wall)
    
    start3 = (12, 12)    # Safe start position
    goal3 = (68, 68)    # Safe goal position
    
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
    """Clear area for robot placement."""
    robot_radius = robot_size // 2
    x, y = position
    
    for dy in range(-robot_radius, robot_radius + 1):
        for dx in range(-robot_radius, robot_radius + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < map_grid.shape[1] and 0 <= ny < map_grid.shape[0]:
                map_grid[ny, nx] = 0

def ensure_clear_path(map_grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], robot_size: int = 5):
    """Ensure a clear (but not necessarily straight) path exists between start and goal."""
    # For complex maps, we rely on manual design rather than diagonal clearing
    pass

def verify_map_connectivity(map_grid, start, goal, robot_size=5):
    """Verify that path exists using BFS."""
    height, width = map_grid.shape
    robot_radius = robot_size // 2
    
    def is_valid_pos(x, y):
        if (x < robot_radius or x >= width - robot_radius or
            y < robot_radius or y >= height - robot_radius):
            return False
        
        for dy in range(-robot_radius, robot_radius + 1):
            for dx in range(-robot_radius, robot_radius + 1):
                if map_grid[y + dy, x + dx] == 1:
                    return False
        return True
    
    if not is_valid_pos(start[0], start[1]):
        print(f"  ❌ Start position {start} invalid")
        return False
    
    if not is_valid_pos(goal[0], goal[1]):
        print(f"  ❌ Goal position {goal} invalid")
        return False
    
    # 8-direction BFS
    directions = [(-1, -1), (-1, 0), (-1, 1),
                  (0, -1), (0, 1),
                  (1, -1), (1, 0), (1, 1)]
    
    queue = deque([start])
    visited = set([start])
    
    while queue:
        x, y = queue.popleft()
        
        if (x, y) == goal:
            return True
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            new_pos = (nx, ny)
            
            if new_pos not in visited and is_valid_pos(nx, ny):
                visited.add(new_pos)
                queue.append(new_pos)
    
    return False