from __future__ import annotations
import heapq
from dataclasses import dataclass
from typing import Dict, List, Tuple
import numpy as np
from .base import Plan, Point
import math

INF = float("inf")


def manhattan(a: Point, b: Point) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def l2_dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class _PQ:

    def __init__(self) -> None:
        self.heap: List[list] = []
        self.entry: Dict[Point, list] = {}
        self.REMOVED = object()
        self.counter = 0

    def push(self, item: Point, key: tuple[float, float]) -> None:
        if item in self.entry:
            self.remove(item)
        self.counter += 1
        ent = [key, self.counter, item]
        self.entry[item] = ent
        heapq.heappush(self.heap, ent)

    def remove(self, item: Point) -> None:
        ent = self.entry.pop(item, None)
        if ent is not None:
            ent[2] = self.REMOVED

    def pop(self) -> tuple[Point, tuple[float, float]]:
        while self.heap:
            key, _, item = heapq.heappop(self.heap)
            if item is not self.REMOVED:
                self.entry.pop(item, None)
                return item, key
        raise KeyError("pop from empty PQ")

    def top_key(self) -> tuple[float, float]:
        while self.heap:
            key, _, item = self.heap[0]
            if item is self.REMOVED:
                heapq.heappop(self.heap)
                continue
            return key
        return (INF, INF)


@dataclass
class DStarLitePlanner:

    _initialized: bool = False
    _occ: np.ndarray | None = None
    _w: int = 0
    _h: int = 0
    _s_start: Point | None = None
    _s_goal: Point | None = None
    _km: float = 0.0
    _g: np.ndarray | None = None
    _rhs: np.ndarray | None = None
    _U: _PQ | None = None

    def reset(self, start: Point, goal: Point, occ: np.ndarray) -> None:
        self._occ = occ.copy()
        self._h, self._w = self._occ.shape
        self._s_start = start
        self._s_goal = goal
        self._km = 0.0

        self._g = np.full((self._h, self._w), INF, dtype=float)
        self._rhs = np.full((self._h, self._w), INF, dtype=float)
        self._rhs[goal[1], goal[0]] = 0.0

        self._U = _PQ()
        self._U.push(goal, self._calculate_key(goal))
        self._initialized = True
        self._compute_shortest_path()

    def update(self, occ: np.ndarray, current: Point) -> None:
        if not self._initialized:
            raise RuntimeError("D* Lite is not initialized. Call reset().")
        assert self._occ is not None and self._s_start is not None

        old_start = self._s_start
        self._s_start = current
        self._km += l2_dist(old_start, current)

        changes = np.argwhere(occ != self._occ)
        self._occ = occ.copy()

        for y, x in changes:
            cell = (int(x), int(y))
            self._update_vertex(cell)
            for n in self._neighbors(cell):
                self._update_vertex(n)

        self._compute_shortest_path()

    def _in_bounds(self, p: Point) -> bool:
        x, y = p
        return 0 <= x < self._w and 0 <= y < self._h

    def _free(self, p: Point) -> bool:
        assert self._occ is not None
        x, y = p
        return self._in_bounds(p) and self._occ[y, x] == 0

    def _neighbors(self, p: Point):
        x, y = p
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            q = (x + dx, y + dy)
            if self._in_bounds(q):
                yield q

    def _cost(self, a: Point, b: Point) -> float:
        if (not self._free(a)) or (not self._free(b)):
            return INF
        return 1.0

    def _calculate_key(self, s: Point) -> tuple[float, float]:
        assert self._g is not None and self._rhs is not None and self._s_start is not None
        x, y = s
        g_rhs = min(self._g[y, x], self._rhs[y, x])
        return (g_rhs + l2_dist(self._s_start, s) + self._km, g_rhs)

    def _update_vertex(self, u: Point) -> None:
        assert self._rhs is not None and self._g is not None and self._U is not None
        assert self._s_goal is not None

        if u != self._s_goal:
            min_rhs = INF
            for s in self._neighbors(u):
                c = self._cost(u, s)
                if c < INF:
                    sx, sy = s
                    val = c + self._g[sy, sx]
                    if val < min_rhs:
                        min_rhs = val
            ux, uy = u
            self._rhs[uy, ux] = min_rhs

        if u in self._U.entry:
            self._U.remove(u)

        ux, uy = u
        if self._g[uy, ux] != self._rhs[uy, ux]:
            self._U.push(u, self._calculate_key(u))

    def _compute_shortest_path(self) -> None:
        assert self._U is not None and self._s_start is not None
        assert self._rhs is not None and self._g is not None

        while (self._U.top_key() < self._calculate_key(self._s_start)) or (
            self._rhs[self._s_start[1], self._s_start[0]] != self._g[self._s_start[1], self._s_start[0]]
        ):
            u, k_old = self._U.pop()
            k_new = self._calculate_key(u)

            if k_old < k_new:
                self._U.push(u, k_new)
                continue

            ux, uy = u
            if self._g[uy, ux] > self._rhs[uy, ux]:
                self._g[uy, ux] = self._rhs[uy, ux]
                for s in self._neighbors(u):
                    self._update_vertex(s)
            else:
                self._g[uy, ux] = INF
                self._update_vertex(u)
                for s in self._neighbors(u):
                    self._update_vertex(s)

    def get_plan(self, current: Point) -> Plan:
        if (not self._initialized) or (self._s_goal is None):
            return Plan(path=[], ok=False, reason="Planner not initialized")

        if not self._free(current) or not self._free(self._s_goal):
            return Plan(path=[], ok=False, reason="Start or goal blocked")

        self._compute_shortest_path()

        path = [current]
        s = current

        for _ in range(self._w * self._h):
            if s == self._s_goal:
                return Plan(path=path, ok=True)

            best = None
            best_val = INF
            for sp in self._neighbors(s):
                c = self._cost(s, sp)
                if c == INF:
                    continue
                val = c + self._g[sp[1], sp[0]]
                if val < best_val:
                    best_val = val
                    best = sp

            if best is None or best_val == INF:
                return Plan(path=[], ok=False, reason="No path")

            s = best
            path.append(s)

        return Plan(path=[], ok=False, reason="No path (loop guard)")

    def next_step(self, current: Point) -> tuple[Point, Plan]:
        plan = self.get_plan(current)
        if plan.ok and len(plan.path) >= 2:
            return plan.path[1], plan
        return current, plan

