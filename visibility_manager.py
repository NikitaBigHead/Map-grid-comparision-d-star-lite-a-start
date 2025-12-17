# visibility_manager.py
"""
Manages limited visibility for the robot.
The robot only "knows" the map within a circular radius around its current position.
Unknown cells are treated as free for planning purposes.
"""

import numpy as np
from typing import Set, Tuple


class VisibilityManager:
    def __init__(self, full_map: np.ndarray, radius: int):
        """
        Initialize with the true full map and vision radius.
        
        Args:
            full_map: The complete environment map (1 = obstacle, 0 = free).
            radius: Vision radius in grid cells.
        """
        self.full_map = full_map.copy()
        self.radius = radius
        self.height, self.width = full_map.shape
        self.visible_cells: Set[Tuple[int, int]] = set()

    def update_visibility(self, pos: Tuple[int, int]):
        """
        Update the set of visible cells based on current robot position.
        
        Args:
            pos: Current robot center position (x, y).
        """
        self.visible_cells.clear()
        x, y = pos
        r2 = self.radius * self.radius
        for dy in range(-self.radius, self.radius + 1):
            for dx in range(-self.radius, self.radius + 1):
                if dx * dx + dy * dy <= r2:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        self.visible_cells.add((nx, ny))

    def get_known_map(self) -> np.ndarray:
        """
        Return the currently known map.
        
        Returns:
            Map where:
              - 1 = known obstacle
              - 0 = known free or unknown (assumed free for optimistic planning)
        """
        known = np.zeros_like(self.full_map, dtype=np.uint8)
        for (x, y) in self.visible_cells:
            if self.full_map[y, x] == 1:
                known[y, x] = 1
        return known