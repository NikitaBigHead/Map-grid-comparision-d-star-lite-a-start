import numpy as np
import random
from typing import Tuple, List

from map_generator import MapGenerator


class Robot:
    def __init__(self, obstacle_map: np.ndarray):
        """
        Initialize the robot

        Args:
            obstacle_map: obstacle map (0 = free, 1 = obstacle)
        """
        self.map = obstacle_map
        self.height, self.width = obstacle_map.shape
        self.size = 5  # Robot size is 5x5

        # Coordinates of the robot's center (must be odd for 5x5 symmetry)
        self.x = None
        self.y = None

        # Definitions of 8 movement directions (dx, dy)
        self.directions = {
            'N': (0, -1),   # North
            'NE': (1, -1),  # Northeast
            'E': (1, 0),    # East
            'SE': (1, 1),   # Southeast
            'S': (0, 1),    # South
            'SW': (-1, 1),  # Southwest
            'W': (-1, 0),   # West
            'NW': (-1, -1)  # Northwest
        }

    def set_random_position(self) -> bool:
        """
        Place the robot at a random free position

        Returns:
            True if placement succeeded, False if no free positions exist
        """
        # List all valid center positions
        # The center must be at least 2 cells away from the border to fit the full 5x5 robot
        possible_positions = []

        for y in range(2, self.height - 2):
            for x in range(2, self.width - 2):
                if self._is_position_valid(x, y):
                    possible_positions.append((x, y))

        if not possible_positions:
            print("No free positions available to place the robot!")
            return False

        # Choose a random valid position
        self.x, self.y = random.choice(possible_positions)
        print(f"Robot placed at position ({self.x}, {self.y})")
        return True

    def set_position(self, x: int, y: int) -> bool:
        """
        Place the robot at a specified center position

        Args:
            x: X-coordinate of the center
            y: Y-coordinate of the center

        Returns:
            True if the position is valid, False otherwise
        """
        if self._is_position_valid(x, y):
            self.x = x
            self.y = y
            print(f"Robot placed at position ({x}, {y})")
            return True
        else:
            print(f"Position ({x}, {y}) is not accessible!")
            return False

    def _is_position_valid(self, x: int, y: int) -> bool:
        """
        Check whether the robot can be placed at the specified center position

        Args:
            x: X-coordinate of the center
            y: Y-coordinate of the center

        Returns:
            True if the position is free and within bounds, False otherwise
        """
        # Boundary check
        # A 5x5 robot centered at (x, y) occupies cells from x-2 to x+2 and y-2 to y+2
        if x < 2 or x >= self.width - 2 or y < 2 or y >= self.height - 2:
            return False

        # Collision check
        for dy in range(-2, 3):  # -2, -1, 0, 1, 2
            for dx in range(-2, 3):
                map_x = x + dx
                map_y = y + dy

                # Additional boundary validation
                if 0 <= map_x < self.width and 0 <= map_y < self.height:
                    if self.map[map_y, map_x] == 1:  # Obstacle found
                        return False
                else:
                    return False

        return True

    def check_collision(self, x: int, y: int) -> bool:
        """
        Check for collision if the robot were placed at (x, y)

        Args:
            x: X-coordinate of the center to test
            y: Y-coordinate of the center to test

        Returns:
            True if collision occurs, False if position is free
        """
        return not self._is_position_valid(x, y)

    def get_occupied_cells(self) -> List[Tuple[int, int]]:
        """
        Get the list of grid cells currently occupied by the robot

        Returns:
            List of (x, y) coordinates occupied by the robot
        """
        if self.x is None or self.y is None:
            return []

        occupied = []
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                occupied.append((self.x + dx, self.y + dy))
        return occupied

    def move(self, direction: str, steps: int = 1) -> bool:
        """
        Move the robot in the specified direction

        Args:
            direction: movement direction ('N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW')
            steps: number of steps to move

        Returns:
            True if movement succeeded, False if blocked by collision
        """
        if self.x is None or self.y is None:
            print("Robot is not placed on the map!")
            return False

        if direction not in self.directions:
            print(f"Unknown direction: {direction}")
            print(f"Valid directions: {list(self.directions.keys())}")
            return False

        dx, dy = self.directions[direction]

        # Target position
        target_x = self.x + dx * steps
        target_y = self.y + dy * steps

        # Validate target position
        if self._is_position_valid(target_x, target_y):
            old_x, old_y = self.x, self.y
            self.x, self.y = target_x, target_y
            print(f"Robot moved from ({old_x}, {old_y}) to ({self.x}, {self.y})")
            return True
        else:
            print(f"Cannot move to position ({target_x}, {target_y}) — collision!")
            return False

    def move_to(self, x: int, y: int) -> bool:
        """
        Attempt to move the robot directly to a target position

        Args:
            x: target X-coordinate of the center
            y: target Y-coordinate of the center

        Returns:
            True if movement succeeded, False if blocked by collision
        """
        if self._is_position_valid(x, y):
            old_x, old_y = self.x, self.y
            self.x, self.y = x, y
            print(f"Robot moved from ({old_x}, {old_y}) to ({self.x}, {self.y})")
            return True
        else:
            print(f"Cannot move to position ({x}, {y}) — collision!")
            return False

    def get_position(self) -> Tuple[int, int]:
        """
        Get the current center position of the robot

        Returns:
            Tuple (x, y) of the robot's center coordinates
        """
        return (self.x, self.y)

    def get_vision_mask(self, radius: int = 10) -> np.ndarray:
        """
        Generate a visibility mask around the robot

        Args:
            radius: vision radius in cells

        Returns:
            Visibility mask (1 = visible, 0 = not visible)
        """
        mask = np.zeros((self.height, self.width), dtype=np.uint8)

        if self.x is None or self.y is None:
            return mask

        for y in range(max(0, self.y - radius), min(self.height, self.y + radius + 1)):
            for x in range(max(0, self.x - radius), min(self.width, self.x + radius + 1)):
                if (x - self.x) ** 2 + (y - self.y) ** 2 <= radius ** 2:
                    mask[y, x] = 1

        return mask

    def visualize(self, show_vision: bool = False, vision_radius: int = 10) -> None:
        """
        Visualize the robot on the map

        Args:
            show_vision: whether to display the visibility area
            vision_radius: radius of the visibility circle
        """
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle, Circle

        # Create a visualization copy of the map
        vis_map = self.map.copy().astype(float)

        # Mark robot cells
        if self.x is not None and self.y is not None:
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    vis_x = self.x + dx
                    vis_y = self.y + dy
                    if 0 <= vis_x < self.width and 0 <= vis_y < self.height:
                        vis_map[vis_y, vis_x] = 0.5  # Robot body

            # Mark robot center distinctly
            vis_map[self.y, self.x] = 0.8

        # Plotting
        fig, ax = plt.subplots(figsize=(12, 10))
        im = ax.imshow(vis_map, cmap='RdYlBu_r', vmin=0, vmax=1)

        # Colorbar legend
        cbar = fig.colorbar(im, ax=ax, ticks=[0, 0.5, 0.8, 1])
        cbar.ax.set_yticklabels(['Free', 'Robot body', 'Robot center', 'Obstacle'])

        # Optionally show vision area
        if show_vision and self.x is not None and self.y is not None:
            vision_circle = Circle((self.x, self.y), vision_radius,
                                   fill=False, color='green', linewidth=2, alpha=0.7)
            ax.add_patch(vision_circle)

        # Draw robot boundary
        if self.x is not None and self.y is not None:
            robot_rect = Rectangle((self.x - 2.5, self.y - 2.5), 5, 5,
                                   fill=False, color='red', linewidth=3)
            ax.add_patch(robot_rect)

        ax.set_title(f"5x5 Robot on map (position: ({self.x}, {self.y}))")
        ax.set_xlabel("X coordinate")
        ax.set_ylabel("Y coordinate")
        ax.grid(True, alpha=0.3)
        plt.show()


# Demonstration example
def robot_demo():
    """Demonstrate robot functionality"""

    # Generate a map with lower obstacle density for clearer demo
    generator = MapGenerator(200, 200)
    obstacle_map = generator.generate_obstacles(0.15)  # 15% obstacle density

    # Create robot
    robot = Robot(obstacle_map)

    # Place robot randomly
    if not robot.set_random_position():
        # Fallback: try a fixed position
        robot.set_position(50, 50)

    # Simple movement sequence for demonstration
    path = ['E', 'E', 'E', 'SE', 'S', 'S', 'SW', 'W', 'W', 'NW', 'N', 'N']

    print("=== Robot Movement Demonstration ===")
    print(f"Initial position: {robot.get_position()}")

    for i, direction in enumerate(path):
        print(f"\nStep {i + 1}: Moving {direction}")
        success = robot.move(direction)
        if not success:
            print("  Movement stopped due to collision!")
            break

    print(f"\nFinal position: {robot.get_position()}")

    # Visualize result
    robot.visualize(show_vision=True, vision_radius=12)


if __name__ == "__main__":
    robot_demo()