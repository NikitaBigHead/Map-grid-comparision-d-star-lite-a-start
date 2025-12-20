from __future__ import annotations
from dataclasses import dataclass, field
from typing import Tuple
import numpy as np
from planners.base import Planner, Plan
from planners.dstar_lite import DStarLitePlanner

Point = Tuple[int, int]
RGB = Tuple[float, float, float]

def _half_extents(size: int) -> tuple[int, int]:

    hl = size // 2
    hr = size - 1 - hl
    return hl, hr

@dataclass
class Robot:
    name: str
    start: Point
    goal: Point
    speed: int
    planner: Planner
    size: int
    color_rgb: RGB
    vision_radius: int
    deadlock_limit: int = 500

    pos: Point = field(init=False)
    last_plan: Plan = field(default_factory=lambda: Plan(path=[], ok=False))

    _initialized_planner: bool = field(default=False, init=False)
    known_occ: np.ndarray | None = field(default=None, init=False)
    _a_star_occ: np.ndarray | None = field(default=None, init=False)

    deadlocked: bool = field(default=False, init=False)
    _stuck_steps: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        if self.size <= 0:
            raise ValueError("size must be positive")
        if self.vision_radius < 0:
            raise ValueError("vision_radius must be >= 0")
        if self.deadlock_limit <= 0:
            raise ValueError("deadlock_limit must be positive")
        for c in self.color_rgb:
            if not (0.0 <= float(c) <= 1.0):
                raise ValueError("color_rgb must be floats in [0,1]")
        self.pos = self.start

    def at_goal(self) -> bool:
        return self.pos == self.goal

    def _finish_tick_deadlock(self, start_pos: Point) -> None:

        if self.at_goal():
            self._stuck_steps = 0
            return

        if self.pos == start_pos:
            self._stuck_steps += 1
            if self._stuck_steps >= self.deadlock_limit:
                self.deadlocked = True
        else:
            self._stuck_steps = 0

    def _exit(self, start_pos: Point) -> None:
        self._finish_tick_deadlock(start_pos)
        return

    def tick(self, occ_cspace: np.ndarray, static_cspace: np.ndarray) -> None:

        if self.at_goal():
            return
        if self.deadlocked:
            return

        h, w = occ_cspace.shape
        start_pos = self.pos

        if self.known_occ is None:
            self.known_occ = static_cspace.copy()

        cx, cy = self.pos
        R = self.vision_radius

        plan_occ = self.known_occ.copy()   

        for y in range(max(0, cy - R), min(h, cy + R + 1)):
            for x in range(max(0, cx - R), min(w, cx + R + 1)):
                if np.hypot(x - cx, y - cy) <= R:
                    plan_occ[y, x] = occ_cspace[y, x] 

        use_dstar = isinstance(self.planner, DStarLitePlanner)

        if use_dstar:
            if not self._initialized_planner:
                self.planner.reset(self.pos, self.goal, plan_occ)
                self._initialized_planner = True
            else:
                self.planner.update(plan_occ, self.pos)

            for _ in range(max(1, int(self.speed))):
                if self.at_goal() or self.deadlocked:
                    return self._exit(start_pos)

                nxt, plan = self.planner.next_step(self.pos)
                self.last_plan = plan

                if (not plan.ok) or (nxt == self.pos):
   
                    return self._exit(start_pos)

                if occ_cspace[nxt[1], nxt[0]] == 1:

                    # self.known_occ[nxt[1], nxt[0]] = 1
                    # self.planner.update(self.known_occ, self.pos)
                    plan_occ[nxt[1], nxt[0]] = 1
                    self.planner.update(plan_occ, self.pos)
                    return self._exit(start_pos)

                self.pos = nxt

            return self._exit(start_pos)

        if (not self._initialized_planner) or (self._a_star_occ is None):
            self._a_star_occ = occ_cspace.copy()
            self.planner.reset(self.pos, self.goal, self._a_star_occ)
            self._initialized_planner = True

        for _ in range(max(1, int(self.speed))):
            if self.at_goal() or self.deadlocked:
                return self._exit(start_pos)

            nxt, plan = self.planner.next_step(self.pos)
            self.last_plan = plan

            if (not plan.ok) or (nxt == self.pos):
                self._a_star_occ = None
                return self._exit(start_pos)

            if occ_cspace[nxt[1], nxt[0]] == 1:
                self._a_star_occ = None
                return self._exit(start_pos)

            self.pos = nxt

        return self._exit(start_pos)
