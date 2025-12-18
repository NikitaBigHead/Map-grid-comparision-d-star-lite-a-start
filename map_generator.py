import numpy as np
import random
import matplotlib.pyplot as plt


class MapGenerator:
    def __init__(self, width: int = 200, height: int = 200):
        self.width = width
        self.height = height
        self.map = np.zeros((height, width), dtype=np.uint8)

    def generate_obstacles(self, density: float = 0.3) -> np.ndarray:
        self.density = density
        self.map.fill(0)

        total_cells = self.width * self.height
        target_obstacle_cells = int(total_cells * self.density)
        current_obstacle_cells = 0

        while current_obstacle_cells < target_obstacle_cells:
            obstacle_type = random.choice(['circle', 'rectangle'])

            if obstacle_type == 'circle':
                radius = random.randint(5, 20)
                center_x = random.randint(radius, self.width - radius - 1)
                center_y = random.randint(radius, self.height - radius - 1)
                added_cells = self._add_circle(center_x, center_y, radius)
            else:
                rect_width = random.randint(5, 40)
                rect_height = random.randint(5, 40)
                x1 = random.randint(0, self.width - rect_width - 1)
                y1 = random.randint(0, self.height - rect_height - 1)
                x2 = x1 + rect_width
                y2 = y1 + rect_height
                added_cells = self._add_rectangle(x1, y1, x2, y2)

            current_obstacle_cells += added_cells

            if current_obstacle_cells > target_obstacle_cells * 1.5:
                print("Предупреждение: достигнут предел генерации препятствий")
                break

        return self.map

    def _add_circle(self, center_x: int, center_y: int, radius: int) -> int:
        added_cells = 0
        for y in range(max(0, center_y - radius), min(self.height, center_y + radius + 1)):
            for x in range(max(0, center_x - radius), min(self.width, center_x + radius + 1)):
                if (x - center_x) ** 2 + (y - center_y) ** 2 <= radius ** 2:
                    if self.map[y, x] == 0:
                        self.map[y, x] = 1
                        added_cells += 1
        return added_cells

    def _add_rectangle(self, x1: int, y1: int, x2: int, y2: int) -> int:
        added_cells = 0
        for y in range(max(0, y1), min(self.height, y2 + 1)):
            for x in range(max(0, x1), min(self.width, x2 + 1)):
                if self.map[y, x] == 0:
                    self.map[y, x] = 1
                    added_cells += 1
        return added_cells

    def save_map(self, filename: str = ""):
        if filename == "":
            filename = f"map_{int(self.density*100)}.npz"

        np.savez_compressed(
            filename,
            map=self.map,
            width=self.width,
            height=self.height
        )
        print(f"Карта сохранена в {filename}")

    def load_map(self, filename: str) -> np.ndarray:
        data = np.load(filename)
        self.map = data['map']
        self.width = data['width']
        self.height = data['height']
        return self.map

    def visualize(self, title: str = ""):
        if title == "":
            title = f"Карта препятствий {int(self.density*100)}% (0=свободно, 1=препятствие)"

        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray_r', vmin=0, vmax=1)
        plt.title(title)
        plt.xlabel("X координата")
        plt.ylabel("Y координата")
        plt.show()

    def map_to_ascii(
        self,
        map_array: np.ndarray | None = None,
        start: tuple[int, int] | None = None,
        goal: tuple[int, int] | None = None,
    ) -> str:
        if map_array is None:
            map_array = self.map

        height, width = map_array.shape
        grid = np.where(map_array == 1, '#', '.').astype('<U1')

        def _check_point(p, name: str):
            if p is None:
                return
            x, y = p
            if not (0 <= x < width and 0 <= y < height):
                raise ValueError(
                    f"{name} вне карты: {(x, y)} при size=({width}, {height})"
                )

        _check_point(start, "start")
        _check_point(goal, "goal")

        if goal is not None:
            gx, gy = goal
            grid[gy, gx] = 'Z'
        if start is not None:
            sx, sy = start
            grid[sy, sx] = 'A'

        return "\n".join("".join(row) for row in grid)
