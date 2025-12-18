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
    deadlock_limit: int = 50  # сколько тиков подряд можно стоять до дедлока

    pos: Point = field(init=False)  # центр
    last_plan: Plan = field(default_factory=lambda: Plan(path=[], ok=False))

    _initialized_planner: bool = field(default=False, init=False)
    known_occ: np.ndarray | None = field(default=None, init=False)
    _a_star_occ: np.ndarray | None = field(default=None, init=False)

    # deadlock state
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

    # ----------------- deadlock helpers -----------------
    def _finish_tick_deadlock(self, start_pos: Point) -> None:
        """
        Увеличиваем счетчик, если робот не сдвинулся за тик.
        Если счетчик >= deadlock_limit => deadlocked=True.
        """
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

    # ----------------- main tick -----------------
    def tick(self, occ_cspace: np.ndarray) -> None:
        """
        occ_cspace — истинная карта в пространстве центров:
        1 = центр запрещён (квадрат size×size пересекает obstacle или выходит за границы)

        D* Lite: планирует по known_occ и update каждый шаг.
        A*: не update каждый шаг; при коллизии сбрасываем _a_star_occ (на следующем тике reset).
        Deadlock: если долго не двигается — robot.deadlocked=True и робот "выключается".
        """
        if self.at_goal():
            return
        if self.deadlocked:
            return

        h, w = occ_cspace.shape
        start_pos = self.pos

        # --- known_occ обновляем всегда ---
        if self.known_occ is None:
            # неизвестное считаем свободным (оптимистично)
            self.known_occ = np.zeros_like(occ_cspace, dtype=np.uint8)

        cx, cy = self.pos
        R = self.vision_radius

        for y in range(max(0, cy - R), min(h, cy + R + 1)):
            for x in range(max(0, cx - R), min(w, cx + R + 1)):
                if abs(x - cx) + abs(y - cy) <= R:
                    self.known_occ[y, x] = occ_cspace[y, x]

        use_dstar = isinstance(self.planner, DStarLitePlanner)

        # ==========================
        # 1) D* Lite
        # ==========================
        if use_dstar:
            plan_occ = self.known_occ

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

                # реальная коллизия по истинной карте
                if occ_cspace[nxt[1], nxt[0]] == 1:
                    # робот "узнал" obstacle
                    self.known_occ[nxt[1], nxt[0]] = 1
                    self.planner.update(self.known_occ, self.pos)
                    return self._exit(start_pos)

                self.pos = nxt

            return self._exit(start_pos)

        # ==========================
        # 2) A* (и прочие неинкрементальные)
        # ==========================
        if (not self._initialized_planner) or (self._a_star_occ is None):
            # снимок мира берём ТОЛЬКО когда перепланируем
            self._a_star_occ = occ_cspace.copy()
            self.planner.reset(self.pos, self.goal, self._a_star_occ)
            self._initialized_planner = True

        for _ in range(max(1, int(self.speed))):
            if self.at_goal() or self.deadlocked:
                return self._exit(start_pos)

            nxt, plan = self.planner.next_step(self.pos)
            self.last_plan = plan

            if (not plan.ok) or (nxt == self.pos):
                return self._exit(start_pos)

            # если столкнулись — считаем, что мир "изменился", и перепланируем на следующем тике
            if occ_cspace[nxt[1], nxt[0]] == 1:
                self._a_star_occ = None
                return self._exit(start_pos)

            self.pos = nxt

        return self._exit(start_pos)
