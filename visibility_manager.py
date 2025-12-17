# visibility_manager.py
import numpy as np
from typing import Set, Tuple

class VisibilityManager:
    def __init__(self, full_map: np.ndarray, radius: int):
        self.full_map = full_map.copy()
        self.radius = radius
        self.height, self.width = full_map.shape
        self.visible_cells: Set[Tuple[int, int]] = set()

    def update_visibility(self, pos: Tuple[int, int]):
        """Mark all cells within radius as visible."""
        self.visible_cells.clear()
        x, y = pos
        for dy in range(-self.radius, self.radius + 1):
            for dx in range(-self.radius, self.radius + 1):
                if dx*dx + dy*dy <= self.radius*self.radius:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        self.visible_cells.add((nx, ny))

    def get_known_map(self) -> np.ndarray:
        """
        Return map where:
          - 1 = known obstacle
          - 0 = free OR unknown (assumed free for planning)
        """
        known = np.zeros_like(self.full_map, dtype=np.uint8)
        for (x, y) in self.visible_cells:
            if self.full_map[y, x] == 1:
                known[y, x] = 1
        return known