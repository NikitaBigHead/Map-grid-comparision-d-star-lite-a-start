# import heapq
# import numpy as np
# from typing import List, Tuple, Optional, Dict
# import math

# class AStar:
#     def __init__(self, grid: np.ndarray, robot_size: int = 5):
#         self.grid = grid.copy()
#         self.height, self.width = grid.shape
#         self.robot_size = robot_size
#         self.robot_radius = robot_size // 2
        
#         # 8-connected directions
#         self.directions = [
#             (-1, -1), (-1, 0), (-1, 1),
#             (0, -1),           (0, 1),
#             (1, -1),  (1, 0),  (1, 1)
#         ]
        
#         # Corresponding costs
#         self.costs = [
#             math.sqrt(2), 1.0, math.sqrt(2),
#             1.0,               1.0,
#             math.sqrt(2), 1.0, math.sqrt(2)
#         ]
        
#         # Store the path for visualization
#         self.found_path = None

#     def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
#         # ✅ L1 (Manhattan) heuristic
#         return abs(a[0] - b[0]) + abs(a[1] - b[1])

#     def is_position_valid(self, pos: Tuple[int, int]) -> bool:
#         """Check if robot can be placed at this position (center)."""
#         x, y = pos
        
#         # Check boundaries (robot is 5×5, center at (x,y))
#         if (x < self.robot_radius or x >= self.width - self.robot_radius or
#             y < self.robot_radius or y >= self.height - self.robot_radius):
#             return False
        
#         # Check all cells occupied by robot
#         for dy in range(-self.robot_radius, self.robot_radius + 1):
#             for dx in range(-self.robot_radius, self.robot_radius + 1):
#                 nx, ny = x + dx, y + dy
#                 if self.grid[ny, nx] == 1:  # Obstacle
#                     return False
        
#         return True

#     def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
#         """Get valid neighboring positions for robot center with costs."""
#         neighbors = []
#         x, y = pos
        
#         for i, (dx, dy) in enumerate(self.directions):
#             new_pos = (x + dx, y + dy)
#             if self.is_position_valid(new_pos):
#                 neighbors.append((new_pos, self.costs[i]))
        
#         return neighbors

#     def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[Dict]:
#         """
#         Find path for 5×5 robot using A*.
#         ✅ Uses L1 (Manhattan) heuristic.
#         ✅ Assumes start and goal are pre-validated.
#         """
#         # Priority queue: (f, g, position, parent)
#         open_set = []
#         heapq.heappush(open_set, (0, 0, start, None))
        
#         # Dictionaries for tracking
#         g_costs = {start: 0}
#         f_costs = {start: self.heuristic(start, goal)}
#         came_from = {}
#         visited_count = 0
        
#         # For visualization
#         self.closed_set = set()
        
#         while open_set:
#             f, g, current, parent = heapq.heappop(open_set)
#             visited_count += 1
            
#             # Skip if we found a better path to this node
#             if g > g_costs.get(current, float('inf')):
#                 continue
            
#             came_from[current] = parent
            
#             if current == goal:
#                 # Reconstruct path
#                 path = []
#                 while current is not None:
#                     path.append(current)
#                     current = came_from.get(current)
#                 path.reverse()
                
#                 self.found_path = path  # Store for visualization
                
#                 return {
#                     'path': path,
#                     'visited_count': visited_count,
#                     'path_length': len(path) - 1,
#                     'total_cost': g_costs[goal]
#                 }
            
#             # Mark as visited
#             self.closed_set.add(current)
            
#             for neighbor, move_cost in self.get_neighbors(current):
#                 tentative_g = g + move_cost
                
#                 if tentative_g < g_costs.get(neighbor, float('inf')):
#                     came_from[neighbor] = current
#                     g_costs[neighbor] = tentative_g
#                     f_score = tentative_g + self.heuristic(neighbor, goal)
#                     f_costs[neighbor] = f_score
#                     heapq.heappush(open_set, (f_score, tentative_g, neighbor, current))
        
#         self.found_path = None  # No path found
#         return None  # No path found

#     def get_path(self) -> List[Tuple[int, int]]:
#         """Get the found path for visualization."""
#         return self.found_path if self.found_path else []

# a_star.py
import heapq
import numpy as np
from typing import List, Tuple, Optional, Dict
import math

class AStar:
    def __init__(self, grid: np.ndarray, robot_size: int = 5):
        self.grid = grid.copy()
        self.height, self.width = grid.shape
        self.robot_size = robot_size
        self.robot_radius = robot_size // 2

        self.directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
        self.costs = [math.sqrt(2), 1.0, math.sqrt(2), 1.0, 1.0, math.sqrt(2), 1.0, math.sqrt(2)]

        self.found_path = None

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return math.sqrt(dx*dx + dy*dy)  # Euclidean — оптимально для 8-направленного

    def is_position_valid(self, pos: Tuple[int, int]) -> bool:
        x, y = pos
        if not (self.robot_radius <= x < self.width - self.robot_radius and
                self.robot_radius <= y < self.height - self.robot_radius):
            return False
        for dy in range(-self.robot_radius, self.robot_radius + 1):
            for dx in range(-self.robot_radius, self.robot_radius + 1):
                if self.grid[y + dy, x + dx] == 1:
                    return False
        return True

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        x, y = pos
        neighbors = []
        for i, (dx, dy) in enumerate(self.directions):
            nx, ny = x + dx, y + dy
            if self.is_position_valid((nx, ny)):
                neighbors.append(((nx, ny), self.costs[i]))
        return neighbors

    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[Dict]:
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, None))

        g_costs = {start: 0}
        came_from = {}
        visited_count = 0

        while open_set:
            _, g, current, parent = heapq.heappop(open_set)
            visited_count += 1

            if g > g_costs.get(current, float('inf')):
                continue

            came_from[current] = parent

            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = came_from.get(current)
                path.reverse()
                self.found_path = path
                return {
                    'path': path,
                    'visited_count': visited_count,
                    'path_length': len(path) - 1,
                    'total_cost': g_costs[goal]
                }

            for neighbor, cost in self.get_neighbors(current):
                tentative_g = g + cost
                if tentative_g < g_costs.get(neighbor, float('inf')):
                    g_costs[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, tentative_g, neighbor, current))

        self.found_path = None
        return None

    def get_path(self) -> List[Tuple[int, int]]:
        return self.found_path if self.found_path else []