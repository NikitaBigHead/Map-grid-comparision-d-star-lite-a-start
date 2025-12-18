from __future__ import annotations
import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple
import numpy as np
from .base import Plan, Point


def manhattan(a: Point, b: Point) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def l2_dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

@dataclass
class AStarPlanner:
    """
    A* — планирует "с нуля".
    В Arena/Robot мы заставляем его пересчитываться при столкновении.
    """
    allow_diagonal: bool = False
    _start: Point | None = None
    _goal: Point | None = None
    _occ: np.ndarray | None = None
    _path: List[Point] = None

    def reset(self, start: Point, goal: Point, occ: np.ndarray) -> None:
        self._start = start
        self._goal = goal
        self._occ = occ.copy()
        self._path = []

    def update(self, occ: np.ndarray, current: Point) -> None:
        # A* тут не обязан пересчитывать на каждый тик.
        # Пересчёт мы делаем "по требованию" (например, при коллизии).
        self._occ = occ.copy()
        if self._start is None:
            self._start = current

    def _free(self, p: Point) -> bool:
        assert self._occ is not None
        h, w = self._occ.shape
        x, y = p
        return (0 <= x < w and 0 <= y < h) and (self._occ[y, x] == 0)

    def _neighbors(self, u: Point):
        nbrs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if self.allow_diagonal:
            nbrs += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dx, dy in nbrs:
            yield (u[0] + dx, u[1] + dy), dx, dy

    def _plan_astar(self, start: Point, goal: Point) -> List[Point]:
        if not self._free(start) or not self._free(goal):
            return []

        openh: List[Tuple[float, float, Point]] = []
        heapq.heappush(openh, (l2_dist(start, goal), 0.0, start))

        came: Dict[Point, Point] = {}
        g: Dict[Point, float] = {start: 0.0}

        while openh:
            _, cost_u, u = heapq.heappop(openh)
            if u == goal:
                path = [u]
                while u in came:
                    u = came[u]
                    path.append(u)
                path.reverse()
                return path

            for v, dx, dy in self._neighbors(u):
                if not self._free(v):
                    continue
                step = 1.0 if (dx == 0 or dy == 0) else math.sqrt(2.0)
                ng = cost_u + step
                if ng < g.get(v, float("inf")):
                    g[v] = ng
                    came[v] = u
                    f = ng + l2_dist(v, goal)
                    heapq.heappush(openh, (f, ng, v))

        return []

    def get_plan(self, current: Point) -> Plan:
        if self._goal is None or self._occ is None:
            return Plan(path=[], ok=False, reason="Planner not initialized")
        path = self._plan_astar(current, self._goal)
        self._path = path
        return Plan(path=path, ok=bool(path), reason="" if path else "No path")

    def next_step(self, current: Point) -> tuple[Point, Plan]:
        plan = self.get_plan(current)
        if plan.ok and len(plan.path) >= 2:
            return plan.path[1], plan
        return current, plan
