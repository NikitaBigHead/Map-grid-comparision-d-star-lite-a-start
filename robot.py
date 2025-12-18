from __future__ import annotations
from dataclasses import dataclass, field
from typing import Tuple
import numpy as np
from planners.base import Planner, Plan
from planners.dstar_lite import DStarLitePlanner

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
    vision_radius: int

    pos: Point = field(init=False)  # центр
    last_plan: Plan = field(default_factory=lambda: Plan(path=[], ok=False))
    _initialized_planner: bool = field(default=False, init=False)
    known_occ: np.ndarray | None = field(default=None, init=False)
    _a_star_occ: np.ndarray | None = field(default=None, init=False)

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
        if self.at_goal():
            return

        h, w = occ_cspace.shape

        # --- known_occ обновляем всегда (для D* Lite и для отладки) ---
        if self.known_occ is None:
            self.known_occ = np.zeros_like(occ_cspace, dtype=np.uint8)

        cx, cy = self.pos
        R = self.vision_radius

        for y in range(max(0, cy - R), min(h, cy + R + 1)):
            for x in range(max(0, cx - R), min(w, cx + R + 1)):
                if abs(x - cx) + abs(y - cy) <= R:
                    self.known_occ[y, x] = occ_cspace[y, x]

        use_known = isinstance(self.planner, DStarLitePlanner)

        # ==========================
        # 1) D* Lite: update каждый шаг
        # ==========================
        if use_known:
            plan_occ = self.known_occ

            if not self._initialized_planner:
                self.planner.reset(self.pos, self.goal, plan_occ)
                self._initialized_planner = True
            else:
                self.planner.update(plan_occ, self.pos)

            for _ in range(max(1, int(self.speed))):
                if self.at_goal():
                    return

                nxt, plan = self.planner.next_step(self.pos)
                self.last_plan = plan

                if (not plan.ok) or (nxt == self.pos):
                    return

                # реальная коллизия по истинной карте
                if occ_cspace[nxt[1], nxt[0]] == 1:
                    # узнали obstacle
                    self.known_occ[nxt[1], nxt[0]] = 1
                    self.planner.update(self.known_occ, self.pos)
                    return

                self.pos = nxt
            return

        # ==========================
        # 2) A* (и другие неинкрементальные):
        #    НЕ update каждый шаг, replanning только при коллизии
        # ==========================
        # если плана ещё нет — делаем reset на "снимке мира"
       # если плана ещё нет — делаем reset на "снимке мира"



        if (not self._initialized_planner) or (self._a_star_occ is None):
            self._a_star_occ = occ_cspace.copy()
            self.planner.reset(self.pos, self.goal, self._a_star_occ)
            self._initialized_planner = True

        for _ in range(max(1, int(self.speed))):
            if self.at_goal():
                return

            nxt, plan = self.planner.next_step(self.pos)
            self.last_plan = plan

            if (not plan.ok) or (nxt == self.pos):
                return
            
            if occ_cspace[nxt[1], nxt[0]] == 1:
                self._a_star_occ = None
                return
            
            self.pos = nxt
            #     # перепланируем ограниченно
            #     found = False
            #     for _try in range(10):
            #         self._a_star_occ = occ_cspace.copy()
            #         self.planner.reset(self.pos, self.goal, self._a_star_occ)

            #         nxt2, plan2 = self.planner.next_step(self.pos)
            #         self.last_plan = plan2

            #         # если пути нет — нет смысла крутиться
            #         # if (not plan2.ok) or (nxt2 == self.pos):
            #         #     break

            #         if occ_cspace[nxt2[1], nxt2[0]] == 0:
            #             nxt = nxt2
            #             found = True
            #             break

            #     if not found:
            #         # важный анти-дедлок: иногда лучше "подождать"
            #         return

            # self.pos = nxt
