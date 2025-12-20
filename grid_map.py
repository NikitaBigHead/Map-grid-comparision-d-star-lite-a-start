from __future__ import annotations
import numpy as np
from dataclasses import dataclass
from typing import Iterable, Tuple

Point = Tuple[int, int]


def _half_extents(size: int) -> tuple[int, int]:
    hl = size // 2
    hr = size - 1 - hl
    return hl, hr


@dataclass
class GridMap:

    grid: np.ndarray

    def __post_init__(self) -> None:
        self.grid = self.grid.astype(np.uint8, copy=True)
        self.h, self.w = self.grid.shape

        ps = np.pad(self.grid.astype(np.int32), ((1, 0), (1, 0)), mode="constant")
        self._ps = ps.cumsum(0).cumsum(1)

    def in_bounds(self, p: Point) -> bool:
        x, y = p
        return 0 <= x < self.w and 0 <= y < self.h

    def is_free(self, p: Point) -> bool:
        x, y = p
        return self.in_bounds(p) and self.grid[y, x] == 0

    def neighbors4(self, p: Point) -> Iterable[Point]:
        x, y = p
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            q = (x + dx, y + dy)
            if self.in_bounds(q):
                yield q

    def window_has_obstacle(self, x0: int, y0: int, x1: int, y1: int) -> bool:

        ps = self._ps
        s = ps[y1, x1] - ps[y0, x1] - ps[y1, x0] + ps[y0, x0]
        return s > 0

    def center_valid_for_robot_with_clearance(self, center: Point, size: int, clearance: int) -> bool:

        if size <= 0 or clearance < 0:
            return False

        effective_size = size + 2 * clearance

        cx, cy = center
        hl, hr = _half_extents(effective_size)

        x0 = cx - hl
        y0 = cy - hl
        x1 = cx + hr + 1
        y1 = cy + hr + 1

        if x0 < 0 or y0 < 0 or x1 > self.w or y1 > self.h:
            return False

        return not self.window_has_obstacle(x0, y0, x1, y1)

    def random_valid_center_with_clearance(
        self, size: int, clearance: int, rng: np.random.Generator | None = None
    ) -> Point:

        rng = rng or np.random.default_rng()

        if size <= 0 or clearance < 0:
            raise ValueError("size must be > 0 and clearance must be >= 0")

        effective_size = size + 2 * clearance
        hl, hr = _half_extents(effective_size)

        xs = np.arange(hl, self.w - hr, dtype=int)
        ys = np.arange(hl, self.h - hr, dtype=int)

        if len(xs) == 0 or len(ys) == 0:
            raise ValueError("Robot effective size is larger than the map.")

        for _ in range(20000):
            cx = int(rng.choice(xs))
            cy = int(rng.choice(ys))
            if self.center_valid_for_robot_with_clearance((cx, cy), size, clearance):
                return (cx, cy)

        candidates: list[Point] = []
        for cy in ys:
            for cx in xs:
                p = (int(cx), int(cy))
                if self.center_valid_for_robot_with_clearance(p, size, clearance):
                    candidates.append(p)

        if not candidates:
            raise RuntimeError("No valid centers for this robot size+clearance on the given map.")

        return candidates[int(rng.integers(0, len(candidates)))]

    def random_valid_center_not_equal_with_clearance(
        self, size: int, clearance: int, not_this: Point, rng: np.random.Generator | None = None
    ) -> Point:
        rng = rng or np.random.default_rng()
        for _ in range(20000):
            p = self.random_valid_center_with_clearance(size, clearance, rng)
            if p != not_this:
                return p
        p = self.random_valid_center_with_clearance(size, clearance, rng)
        if p == not_this:
            raise RuntimeError("Could not find a different valid center.")
        return p
