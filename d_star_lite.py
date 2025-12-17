# import heapq
# import numpy as np
# from typing import List, Tuple, Optional, Dict
# from collections import defaultdict
# import math

# class DStarLite:
#     def __init__(self, grid: np.ndarray, robot_size: int = 5):
#         self.grid = grid.copy()
#         self.height, self.width = grid.shape
#         self.robot_size = robot_size
#         self.robot_radius = robot_size // 2  # 2 для 5×5
        
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
        
#         self.reset()

#     def reset(self):
#         self.km = 0.0
#         self.start = None
#         self.goal = None
        
#         self.g = defaultdict(lambda: float('inf'))
#         self.rhs = defaultdict(lambda: float('inf'))
        
#         self.U = []
#         self.in_queue = {}
#         self.visited_count = 0

#     def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
#         return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

#     def calculate_key(self, s: Tuple[int, int]) -> tuple:
#         min_val = min(self.g[s], self.rhs[s])
#         k1 = min_val + self.heuristic(self.start, s) + self.km
#         k2 = min_val
#         return (k1, k2)

#     def is_position_valid(self, pos: Tuple[int, int]) -> bool:
#         """Check if robot can be placed at this position (center)."""
#         x, y = pos
        
#         if (x < self.robot_radius or x >= self.width - self.robot_radius or
#             y < self.robot_radius or y >= self.height - self.robot_radius):
#             return False
        
#         for dy in range(-self.robot_radius, self.robot_radius + 1):
#             for dx in range(-self.robot_radius, self.robot_radius + 1):
#                 nx, ny = x + dx, y + dy
#                 if self.grid[ny, nx] == 1:
#                     return False
        
#         return True

#     def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
#         """Get all neighboring positions."""
#         x, y = pos
#         neighbors = []
#         for dx, dy in self.directions:
#             neighbor = (x + dx, y + dy)
#             neighbors.append(neighbor)
#         return neighbors

#     def get_valid_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
#         """Get valid neighboring positions for robot."""
#         return [n for n in self.get_neighbors(pos) if self.is_position_valid(n)]

#     def get_cost(self, u: Tuple[int, int], v: Tuple[int, int]) -> float:
#         dx = v[0] - u[0]
#         dy = v[1] - u[1]
        
#         if abs(dx) + abs(dy) == 1:
#             return 1.0
#         elif abs(dx) == 1 and abs(dy) == 1:
#             return math.sqrt(2)
#         else:
#             return float('inf')

#     def insert_or_update(self, s: Tuple[int, int]):
#         key = self.calculate_key(s)
#         if s in self.in_queue:
#             self.in_queue[s] = None
        
#         self.in_queue[s] = key
#         heapq.heappush(self.U, (key[0], key[1], s))

#     def top_key(self):
#         while self.U:
#             k1, k2, s = self.U[0]
#             if self.in_queue.get(s) == (k1, k2):
#                 return (k1, k2)
#             heapq.heappop(self.U)
#         return (float('inf'), float('inf'))

#     def pop(self):
#         while self.U:
#             k1, k2, s = heapq.heappop(self.U)
#             if self.in_queue.get(s) == (k1, k2):
#                 self.in_queue[s] = None
#                 return s
#         return None

#     def update_vertex(self, u: Tuple[int, int]):
#         if u != self.goal:
#             min_rhs = float('inf')
#             for v in self.get_valid_neighbors(u):
#                 cost = self.get_cost(u, v)
#                 candidate = cost + self.g[v]
#                 if candidate < min_rhs:
#                     min_rhs = candidate
#             self.rhs[u] = min_rhs
        
#         if u in self.in_queue and self.in_queue[u] is not None:
#             self.in_queue[u] = None
        
#         if self.g[u] != self.rhs[u]:
#             self.insert_or_update(u)

#     def compute_shortest_path(self):
#         max_iterations = self.width * self.height * 10
#         iterations = 0
        
#         while self.U and iterations < max_iterations:
#             iterations += 1
            
#             top_k = self.top_key()
#             start_k = self.calculate_key(self.start)
            
#             if top_k >= start_k and self.rhs[self.start] == self.g[self.start]:
#                 break
            
#             u = self.pop()
#             if u is None:
#                 break
            
#             self.visited_count += 1
            
#             k_old = self.calculate_key(u)
            
#             if k_old < self.calculate_key(u):
#                 self.insert_or_update(u)
#             elif self.g[u] > self.rhs[u]:
#                 self.g[u] = self.rhs[u]
#                 for v in self.get_neighbors(u):
#                     if self.is_position_valid(v):
#                         self.update_vertex(v)
#             else:
#                 self.g[u] = float('inf')
#                 self.update_vertex(u)
#                 for v in self.get_neighbors(u):
#                     if self.is_position_valid(v):
#                         self.update_vertex(v)
        
#         return self.visited_count

#     def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[Dict]:
#         if not self.is_position_valid(start) or not self.is_position_valid(goal):
#             print(f"D* Lite: Invalid start or goal position for {self.robot_size}×{self.robot_size} robot")
#             return None
        
#         self.reset()
#         self.start = start
#         self.goal = goal
        
#         self.rhs[goal] = 0.0
#         self.insert_or_update(goal)
        
#         visited = self.compute_shortest_path()
        
#         if self.g[start] == float('inf'):
#             return None
        
#         path = self.extract_path()
        
#         if not path:
#             return None
        
#         return {
#             'path': path,
#             'visited_count': visited,
#             'max_queue_size': len([k for k in self.in_queue if self.in_queue[k] is not None]),
#             'path_length': len(path) - 1,
#             'total_cost': self.g[start]
#         }

#     def extract_path(self) -> List[Tuple[int, int]]:
#         if self.start is None or self.goal is None:
#             return []
        
#         if self.g[self.start] == float('inf'):
#             return []
        
#         path = [self.start]
#         current = self.start
#         max_steps = self.width * self.height
#         steps = 0
        
#         while current != self.goal and steps < max_steps:
#             steps += 1
            
#             best_next = None
#             best_cost = float('inf')
            
#             for neighbor in self.get_valid_neighbors(current):
#                 cost = self.get_cost(current, neighbor) + self.g[neighbor]
#                 if cost < best_cost:
#                     best_cost = cost
#                     best_next = neighbor
            
#             if best_next is None:
#                 break
            
#             if best_next in path[-10:]:
#                 break
            
#             path.append(best_next)
#             current = best_next
        
#         return path

# d_star_lite.py
import heapq
import numpy as np
from typing import List, Tuple, Optional, Dict
from collections import defaultdict
import math

class DStarLite:
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

        self.reset()

    def reset(self):
        """Reset internal state (used only for a completely new problem)."""
        self.km = 0.0
        self.start = None
        self.goal = None
        self.g = defaultdict(lambda: float('inf'))
        self.rhs = defaultdict(lambda: float('inf'))
        self.U = []
        self.in_queue = {}
        self.visited_count = 0

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        # ✅ L1 (Manhattan) heuristic everywhere
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def calculate_key(self, s: Tuple[int, int]) -> tuple:
        min_val = min(self.g[s], self.rhs[s])
        # ✅ FIXED: use heuristic(s, goal)
        k1 = min_val + self.heuristic(s, self.goal) + self.km
        k2 = min_val
        return (k1, k2)

    def is_position_valid(self, pos: Tuple[int, int]) -> bool:
        x, y = pos
        if (x < self.robot_radius or x >= self.width - self.robot_radius or
            y < self.robot_radius or y >= self.height - self.robot_radius):
            return False
        for dy in range(-self.robot_radius, self.robot_radius + 1):
            for dx in range(-self.robot_radius, self.robot_radius + 1):
                nx, ny = x + dx, y + dy
                if self.grid[ny, nx] == 1:
                    return False
        return True

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        x, y = pos
        neighbors = []
        for dx, dy in self.directions:
            neighbor = (x + dx, y + dy)
            neighbors.append(neighbor)
        return neighbors

    def get_valid_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        return [n for n in self.get_neighbors(pos) if self.is_position_valid(n)]

    def get_cost(self, u: Tuple[int, int], v: Tuple[int, int]) -> float:
        dx = v[0] - u[0]
        dy = v[1] - u[1]
        if abs(dx) + abs(dy) == 1:
            return 1.0
        elif abs(dx) == 1 and abs(dy) == 1:
            return math.sqrt(2)
        else:
            return float('inf')

    def insert_or_update(self, s: Tuple[int, int]):
        key = self.calculate_key(s)
        self.in_queue[s] = key
        heapq.heappush(self.U, (key[0], key[1], s))

    def top_key(self):
        while self.U:
            k1, k2, s = self.U[0]
            if self.in_queue.get(s) == (k1, k2):
                return (k1, k2)
            heapq.heappop(self.U)
        return (float('inf'), float('inf'))

    def pop(self):
        while self.U:
            k1, k2, s = heapq.heappop(self.U)
            if self.in_queue.get(s) == (k1, k2):
                self.in_queue[s] = None
                return s
        return None

    def update_vertex(self, u: Tuple[int, int]):
        if u != self.goal:
            min_rhs = float('inf')
            for v in self.get_valid_neighbors(u):
                cost = self.get_cost(u, v)
                candidate = cost + self.g[v]
                if candidate < min_rhs:
                    min_rhs = candidate
            self.rhs[u] = min_rhs

        if u in self.in_queue:
            del self.in_queue[u]

        if self.g[u] != self.rhs[u]:
            self.insert_or_update(u)

    def compute_shortest_path(self):
        max_iterations = self.width * self.height * 10
        iterations = 0
        self.visited_count = 0

        while self.U and iterations < max_iterations:
            iterations += 1
            top_k = self.top_key()
            start_k = self.calculate_key(self.start)

            if top_k >= start_k and self.rhs[self.start] == self.g[self.start]:
                break

            u = self.pop()
            if u is None:
                break

            self.visited_count += 1

            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for v in self.get_neighbors(u):
                    if self.is_position_valid(v):
                        self.update_vertex(v)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for v in self.get_neighbors(u):
                    if self.is_position_valid(v):
                        self.update_vertex(v)

        return self.visited_count

    def extract_path(self) -> List[Tuple[int, int]]:
        if self.start is None or self.goal is None or self.g[self.start] == float('inf'):
            return []

        path = [self.start]
        current = self.start
        max_steps = self.width * self.height
        steps = 0

        while current != self.goal and steps < max_steps:
            steps += 1
            best_next = None
            best_cost = float('inf')
            for neighbor in self.get_valid_neighbors(current):
                cost = self.get_cost(current, neighbor) + self.g[neighbor]
                if cost < best_cost:
                    best_cost = cost
                    best_next = neighbor
            if best_next is None or best_next in path[-10:]:
                break
            path.append(best_next)
            current = best_next

        return path

    def update_obstacles(self, changed_cells: List[Tuple[int, int]]):
        affected = set()
        for (x, y) in changed_cells:
            u = (x, y)
            affected.add(u)
            for neighbor in self.get_neighbors(u):
                affected.add(neighbor)
        for u in affected:
            self.update_vertex(u)
    
    # --- NEW METHOD FOR CLEAN INITIALIZATION ---
    def initialize(self, start: Tuple[int, int], goal: Tuple[int, int]):
        """Initialize D* Lite for a new planning problem."""
        self.reset()
        self.start = start
        self.goal = goal
        self.rhs[goal] = 0.0
        self.insert_or_update(goal)

    # --- DEPRECATED FOR DYNAMIC SCENARIOS, but kept for compatibility ---
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[Dict]:
        if not self.is_position_valid(start) or not self.is_position_valid(goal):
            print(f"D* Lite: Invalid start or goal position for {self.robot_size}×{self.robot_size} robot")
            return None

        # Do NOT reset here for dynamic scenarios! This is for static use only.
        self.start = start
        self.goal = goal
        self.rhs[goal] = 0.0
        self.insert_or_update(goal)

        visited = self.compute_shortest_path()

        if self.g[start] == float('inf'):
            return None

        path = self.extract_path()
        if not path:
            return None

        return {
            'path': path,
            'visited_count': visited,
            'max_queue_size': len([k for k in self.in_queue if self.in_queue[k] is not None]),
            'path_length': len(path) - 1,
            'total_cost': self.g[start]
        }