from __future__ import annotations
from dataclasses import dataclass, field
from typing import Tuple
import numpy as np
from planners.base import Planner, Plan

Point = Tuple[int, int]
RGB = Tuple[float, float, float]  # 0..1 (matplotlib-friendly)


def _half_extents(size: int) -> tuple[int, int]:
    """
    Для размера size возвращает (half_left, half_right) так, чтобы окно имело ровно size клеток.
    Работает и для чётных, и для нечётных размеров.
    """
    hl = size // 2
    hr = size - 1 - hl
    return hl, hr


@dataclass
class Robot:
    name: str
    start: Point          # центр
    goal: Point           # центр
    speed: int
    planner: Planner
    size: int             # сторона квадрата в клетках
    color_rgb: RGB        # (r,g,b) в диапазоне 0..1

    pos: Point = field(init=False)  # центр
    last_plan: Plan = field(default_factory=lambda: Plan(path=[], ok=False))
    _initialized_planner: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        if self.size <= 0:
            raise ValueError("size must be positive")
        for c in self.color_rgb:
            if not (0.0 <= float(c) <= 1.0):
                raise ValueError("color_rgb must be floats in [0,1]")
        self.pos = self.start

    def at_goal(self) -> bool:
        return self.pos == self.goal

    def tick(self, occ_cspace: np.ndarray) -> None:
        """
        occ_cspace — карта в пространстве центров:
        1 = центр запрещён (квадрат size×size пересекает obstacle или выходит за границы)
        """
        if self.at_goal():
            return

        if not self._initialized_planner:
            self.planner.reset(self.pos, self.goal, occ_cspace)
            self._initialized_planner = True
        else:
            self.planner.update(occ_cspace, self.pos)

        for _ in range(max(1, int(self.speed))):
            if self.at_goal():
                return

            nxt, plan = self.planner.next_step(self.pos)
            self.last_plan = plan

            if (not plan.ok) or (nxt == self.pos):
                return

            # столкновение в c-space => replanning (особенно важно для A*)
            if occ_cspace[nxt[1], nxt[0]] == 1:
                nxt2, plan2 = self.planner.next_step(self.pos)
                self.last_plan = plan2
                if (not plan2.ok) or (nxt2 == self.pos) or (occ_cspace[nxt2[1], nxt2[0]] == 1):
                    return
                nxt = nxt2

            self.pos = nxt
