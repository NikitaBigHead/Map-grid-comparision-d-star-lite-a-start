from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol, Tuple, List, Optional
import numpy as np

Point = Tuple[int, int]


@dataclass
class Plan:
    path: List[Point]
    ok: bool
    reason: str = ""


class Planner(Protocol):
    """
    Общий интерфейс планировщика.
    """
    def reset(self, start: Point, goal: Point, occ: np.ndarray) -> None: ...
    def update(self, occ: np.ndarray, current: Point) -> None: ...
    def get_plan(self, current: Point) -> Plan: ...
    def next_step(self, current: Point) -> tuple[Point, Plan]: ...

