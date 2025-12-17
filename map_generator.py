import numpy as np
import random
import matplotlib.pyplot as plt


class MapGenerator:
    def __init__(self, width: int = 200, height: int = 200):
        """
        Initialize the map generator
        """
        self.width = width
        self.height = height
        self.map = np.zeros((height, width), dtype=np.uint8)

    def generate_obstacles(self, density: float = 0.3, seed: int = None) -> np.ndarray:
        """
        Generate obstacles on the map with optional seed
        """
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
            
        self.density = density
        self.map.fill(0)

        total_cells = self.width * self.height
        target_obstacle_cells = int(total_cells * self.density)
        current_obstacle_cells = 0

        while current_obstacle_cells < target_obstacle_cells:
            obstacle_type = random.choice(['circle', 'rectangle'])

            if obstacle_type == 'circle':
                radius = random.randint(5, 15)  # Smaller circles
                center_x = random.randint(radius, self.width - radius - 1)
                center_y = random.randint(radius, self.height - radius - 1)
                added_cells = self._add_circle(center_x, center_y, radius)

            else:  # rectangle
                rect_width = random.randint(5, 30)  # Smaller rectangles
                rect_height = random.randint(5, 30)
                x1 = random.randint(0, self.width - rect_width - 1)
                y1 = random.randint(0, self.height - rect_height - 1)
                x2 = x1 + rect_width
                y2 = y1 + rect_height
                added_cells = self._add_rectangle(x1, y1, x2, y2)

            current_obstacle_cells += added_cells

            if current_obstacle_cells > target_obstacle_cells * 1.5:
                break

        return self.map
    
    def generate_with_fixed_start_goal(self, density: float, start: tuple, goal: tuple, seed: int = None) -> np.ndarray:
        """
        Generate map with fixed start and goal positions
        """
        # Generate obstacles
        map_grid = self.generate_obstacles(density, seed)
        
        # Ensure start and goal areas are clear (5×5 area)
        robot_radius = 2
        
        # Clear area around start (9×9 area for safety)
        for dy in range(-robot_radius-2, robot_radius+3):
            for dx in range(-robot_radius-2, robot_radius+3):
                x, y = start[0] + dx, start[1] + dy
                if 0 <= x < self.width and 0 <= y < self.height:
                    map_grid[y, x] = 0
        
        # Clear area around goal
        for dy in range(-robot_radius-2, robot_radius+3):
            for dx in range(-robot_radius-2, robot_radius+3):
                x, y = goal[0] + dx, goal[1] + dy
                if 0 <= x < self.width and 0 <= y < self.height:
                    map_grid[y, x] = 0
        
        # Clear a wider corridor between start and goal
        self._clear_path_corridor(map_grid, start, goal, width=3)
        
        return map_grid
    
    def _clear_path_corridor(self, map_grid: np.ndarray, start: tuple, goal: tuple, width: int = 3):
        """Clear a corridor between start and goal for easier pathfinding."""
        x1, y1 = start
        x2, y2 = goal
        
        # Clear along a straight line with some width
        steps = max(abs(x2 - x1), abs(y2 - y1))
        if steps == 0:
            return
            
        for i in range(steps + 1):
            t = i / steps
            x = int(x1 + (x2 - x1) * t)
            y = int(y1 + (y2 - y1) * t)
            
            # Clear a square of width×width
            for dy in range(-width, width + 1):
                for dx in range(-width, width + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        map_grid[ny, nx] = 0

    def _add_circle(self, center_x: int, center_y: int, radius: int) -> int:
        added_cells = 0
        for y in range(max(0, center_y - radius), min(self.height, center_y + radius + 1)):
            for x in range(max(0, center_x - radius), min(self.width, center_x + radius + 1)):
                if (x - center_x) ** 2 + (y - center_y) ** 2 <= radius ** 2:
                    if self.map[y, x] == 0:
                        self.map[y, x] = 1
                        added_cells += 1
        return added_cells

    def _add_rectangle(self, x1: int, y1: int, x2: int, y2: int) -> int:
        added_cells = 0
        for y in range(max(0, y1), min(self.height, y2 + 1)):
            for x in range(max(0, x1), min(self.width, x2 + 1)):
                if self.map[y, x] == 0:
                    self.map[y, x] = 1
                    added_cells += 1
        return added_cells

    def save_map(self, filename: str = ""):
        if filename == "":
            filename = f"map_{int(self.density*100)}.npz"

        np.savez_compressed(
            filename,
            map=self.map,
            width=self.width,
            height=self.height
        )
        print(f"Map saved to {filename}")

    def load_map(self, filename: str) -> np.ndarray:
        data = np.load(filename)
        self.map = data['map']
        self.width = data['width']
        self.height = data['height']
        return self.map

    def visualize(self, title: str = ""):
        if title == "":
            title = f"Obstacle map {int(self.density*100)}% (0=free, 1=obstacle)"

        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray_r', vmin=0, vmax=1)
        plt.title(title)
        plt.xlabel("X coordinate")
        plt.ylabel("Y coordinate")
        plt.show()