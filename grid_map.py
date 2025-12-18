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
    """
    Map сущность: хранит статическую occupancy grid (0=free, 1=obstacle).
    Координаты: (x, y). Индексация в numpy: grid[y, x].

    Дополнительно:
    - умеет выдавать случайный центр для робота размера size×size.
    """
    grid: np.ndarray  # shape (H, W), dtype uint8

    def __post_init__(self) -> None:
        self.grid = self.grid.astype(np.uint8, copy=True)
        self.h, self.w = self.grid.shape

        # prefix sum для быстрых запросов "есть ли obstacle в окне"
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
        """
        Проверяет, есть ли obstacle (==1) в прямоугольнике [x0:x1, y0:y1] (x1,y1 НЕ включительно).
        Требует, чтобы окно было в пределах карты.
        """
        ps = self._ps
        s = ps[y1, x1] - ps[y0, x1] - ps[y1, x0] + ps[y0, x0]
        return s > 0

    def center_valid_for_robot(self, center: Point, size: int) -> bool:
        """
        Можно ли поставить робота size×size с центром center на статическую карту.
        """
        if size <= 0:
            return False

        cx, cy = center
        hl, hr = _half_extents(size)

        x0 = cx - hl
        y0 = cy - hl
        x1 = cx + hr + 1
        y1 = cy + hr + 1

        # квадрат должен целиком влезать в карту
        if x0 < 0 or y0 < 0 or x1 > self.w or y1 > self.h:
            return False

        # и не пересекать препятствия
        return not self.window_has_obstacle(x0, y0, x1, y1)

    def random_valid_center(self, size: int, rng: np.random.Generator | None = None) -> Point:
        """
        Случайный центр, валидный для робота size×size по статической карте.
        (не учитывает других роботов — только map obstacles)
        """
        rng = rng or np.random.default_rng()

        hl, hr = _half_extents(size)
        # допустимые центры по границам
        xs = np.arange(hl, self.w - hr, dtype=int)
        ys = np.arange(hl, self.h - hr, dtype=int)

        if len(xs) == 0 or len(ys) == 0:
            raise ValueError("Robot size is larger than the map.")

        # пробуем случайно, но с защитой
        for _ in range(20000):
            cx = int(rng.choice(xs))
            cy = int(rng.choice(ys))
            if self.center_valid_for_robot((cx, cy), size):
                return (cx, cy)

        # fallback: полный перебор (медленнее, но надёжно)
        candidates = []
        for cy in ys:
            for cx in xs:
                if self.center_valid_for_robot((int(cx), int(cy)), size):
                    candidates.append((int(cx), int(cy)))

        if not candidates:
            raise RuntimeError("No valid centers for this robot size on the given map.")

        return candidates[int(rng.integers(0, len(candidates)))]

    def random_valid_center_not_equal(
        self, size: int, not_this: Point, rng: np.random.Generator | None = None
    ) -> Point:
        rng = rng or np.random.default_rng()
        for _ in range(20000):
            p = self.random_valid_center(size, rng)
            if p != not_this:
                return p
        # если совсем плохо — добиваемся перебором
        p = self.random_valid_center(size, rng)
        if p == not_this:
            raise RuntimeError("Could not find a different valid center.")
        return p
