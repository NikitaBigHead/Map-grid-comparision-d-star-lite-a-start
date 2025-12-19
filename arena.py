from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

import numpy as np
from grid_map import GridMap
from robot import Robot, _half_extents

from PIL import Image
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.patches import Rectangle

Point = Tuple[int, int]


def _square_bounds_from_center(center: Point, size: int) -> tuple[int, int, int, int]:
    """
    По центру (cx,cy) и size возвращает bounds: x0,y0,x1,y1 (x1/y1 НЕ включительно)
    """
    cx, cy = center
    hl, hr = _half_extents(size)
    x0 = cx - hl
    y0 = cy - hl
    x1 = cx + hr + 1
    y1 = cy + hr + 1
    return x0, y0, x1, y1


def _stamp_square_center(occ: np.ndarray, center: Point, size: int) -> None:
    """
    Помечает в occ квадрат size×size вокруг центра как obstacle (обрезая по границам).
    """
    h, w = occ.shape
    x0, y0, x1, y1 = _square_bounds_from_center(center, size)

    x0c = max(0, min(w, x0))
    y0c = max(0, min(h, y0))
    x1c = max(0, min(w, x1))
    y1c = max(0, min(h, y1))

    if x0c < x1c and y0c < y1c:
        occ[y0c:y1c, x0c:x1c] = 1


def _to_cspace_centers(occ: np.ndarray, size: int) -> np.ndarray:
    """
    Делает c-space по центрам:
    center (cx,cy) разрешён, если квадрат size×size вокруг него:
      - полностью внутри карты
      - и не пересекает obstacle в occ
    """
    if size <= 1:
        return occ.copy()

    h, w = occ.shape
    ps = np.pad(occ.astype(np.int32), ((1, 0), (1, 0)), mode="constant").cumsum(0).cumsum(1)

    hl, hr = _half_extents(size)

    out = np.ones((h, w), dtype=np.uint8)  # по умолчанию запрещено
    for cy in range(h):
        y0 = cy - hl
        y1 = cy + hr + 1
        if y0 < 0 or y1 > h:
            continue
        for cx in range(w):
            x0 = cx - hl
            x1 = cx + hr + 1
            if x0 < 0 or x1 > w:
                continue

            # сумма obstacles в окне
            s = ps[y1, x1] - ps[y0, x1] - ps[y1, x0] + ps[y0, x0]
            out[cy, cx] = 1 if s > 0 else 0

    return out


@dataclass
class Arena:
    map: GridMap
    robots: List[Robot] = field(default_factory=list)
    t: int = 0

    # --- recording state ---
    _record: bool = field(default=False, init=False)
    _frames: List[Image.Image] = field(default_factory=list, init=False)
    _gif_path: str = field(default="sim.gif", init=False)
    _frame_ms: int = field(default=60, init=False)
    _record_every: int = field(default=1, init=False)

    # renderer cached
    _fig: Optional[object] = field(default=None, init=False)
    _canvas: Optional[object] = field(default=None, init=False)
    _ax: Optional[object] = field(default=None, init=False)
    _robot_patches: List[Rectangle] = field(default_factory=list, init=False)
    _goal_sc: Optional[object] = field(default=None, init=False)
    _txt: Optional[object] = field(default=None, init=False)

    clearance: int = 1  

    def add_robot(self, robot: Robot) -> None:
        self.robots.append(robot)

    def _static_occ(self) -> np.ndarray:
        return self.map.grid

    def environment(self, for_robot: Robot | None = None) -> np.ndarray:

        occ = self._static_occ().copy()

        for r in self.robots:
            if (for_robot is not None) and (r is for_robot):
                continue
            _stamp_square_center(occ, r.pos, r.size)

        eff_size = for_robot.size + 2 * self.clearance
        return _to_cspace_centers(occ, eff_size)

    def step(self) -> None:
        for r in self.robots:
            eff = r.size + 2 * self.clearance
            static_cs = self.static_cspace(eff)
            dyn_cs = self.environment(for_robot=r)  
            r.tick(dyn_cs, static_cs)

            # occ_cspace = self.environment(for_robot=r)
            # r.tick(occ_cspace)
            
            # if r.deadlocked:
            #     print("r is blocked",r.deadlocked)
            #     self.robots.remove(r)

        self.t += 1

        if self._record and (self.t % self._record_every == 0):
            self.record_frame()

    def all_reached(self) -> bool:
        return all(r.at_goal() for r in self.robots)

    def static_cspace(self, eff_size: int) -> np.ndarray:
        return _to_cspace_centers(self._static_occ(), eff_size)
  
    def start_recording(
        self,
        gif_path: str = "sim.gif",
        frame_ms: int = 60,
        record_every: int = 1,
        figsize: tuple[float, float] = (8, 8),
        dpi: int = 120,
        title: str = "Arena simulation"
    ) -> None:
        self._record = True
        self._frames.clear()
        self._gif_path = gif_path
        self._frame_ms = int(frame_ms)
        self._record_every = max(1, int(record_every))

        grid = self.map.grid
        h, w = grid.shape

        fig = Figure(figsize=figsize, dpi=dpi)
        canvas = FigureCanvas(fig)
        ax = fig.add_subplot(111)

        ax.set_title(title)
        ax.set_xlim(-0.5, w - 0.5)
        ax.set_ylim(h - 0.5, -0.5)
        ax.set_aspect("equal")
        ax.imshow(grid, cmap="gray_r", vmin=0, vmax=1, interpolation="nearest")

        # цели крестиками, цвет = цвет робота
        goals = np.array([r.goal for r in self.robots], dtype=float) if self.robots else np.zeros((0, 2))
        colors = np.array([r.color_rgb for r in self.robots], dtype=float) if self.robots else np.zeros((0, 3))
        goal_sc = ax.scatter(goals[:, 0], goals[:, 1], marker="x", s=80, c=colors)

        # квадраты роботов
        patches: List[Rectangle] = []
        for r in self.robots:
            x0, y0, x1, y1 = _square_bounds_from_center(r.pos, r.size)
            rect = Rectangle(
                (x0 - 0.5, y0 - 0.5),  # сдвиг для попадания в клетки
                r.size,
                r.size,
                facecolor=r.color_rgb,
                edgecolor=r.color_rgb,
                linewidth=1.5,
                alpha=0.9,
            )
            ax.add_patch(rect)
            patches.append(rect)

        txt = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top")

        self._fig, self._canvas, self._ax = fig, canvas, ax
        self._goal_sc = goal_sc
        self._robot_patches = patches
        self._txt = txt

    def record_frame(self) -> None:
        if not self._record:
            return
        if self._fig is None or self._canvas is None:
            raise RuntimeError("Recording not initialized. Call start_recording() first.")

        # обновляем квадраты роботов
        for rect, r in zip(self._robot_patches, self.robots):
            x0, y0, x1, y1 = _square_bounds_from_center(r.pos, r.size)
            rect.set_xy((x0 - 0.5, y0 - 0.5))

        reached = sum(int(r.at_goal()) for r in self.robots)
        self._txt.set_text(f"t={self.t} | reached={reached}/{len(self.robots)}")

        self._canvas.draw()
        w, h = self._fig.canvas.get_width_height()
        rgba = np.frombuffer(self._canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)
        rgb = rgba[:, :, :3]

        img = Image.fromarray(rgb, mode="RGB").convert("P", palette=Image.ADAPTIVE)
        self._frames.append(img)

    def stop_and_save_gif(self) -> None:
        if not self._record:
            return
        self._record = False

        if not self._frames:
            raise RuntimeError("No frames recorded.")

        first, rest = self._frames[0], self._frames[1:]
        first.save(
            self._gif_path,
            save_all=True,
            append_images=rest,
            duration=self._frame_ms,
            loop=0,
            optimize=False,
        )
        print(f"GIF saved: {self._gif_path}")

        self._frames.clear()
        self._fig = self._canvas = self._ax = None
        self._robot_patches = []
        self._goal_sc = self._txt = None
