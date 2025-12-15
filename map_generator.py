import numpy as np
import random
import matplotlib.pyplot as plt


class MapGenerator:
    def __init__(self, width: int = 200, height: int = 200):
        """
        Инициализация генератора карты

        Args:
            width: ширина карты
            height: высота карты
        """
        self.width = width
        self.height = height
        self.map = np.zeros((height, width), dtype=np.uint8)

    def generate_obstacles(self, density: float = 0.3) -> np.ndarray:
        """
        Генерация препятствий на карте

        Args:
            density: плотность препятствий (0-1)

        Returns:
            Карта с препятствиями
        """
        self.density = density

        # Очищаем карту
        self.map.fill(0)

        # Вычисляем общее количество клеток для препятствий
        total_cells = self.width * self.height
        target_obstacle_cells = int(total_cells * self.density)
        current_obstacle_cells = 0

        # Пока не достигнута нужная плотность
        while current_obstacle_cells < target_obstacle_cells:
            # Случайно выбираем форму препятствия
            obstacle_type = random.choice(['circle', 'rectangle'])

            if obstacle_type == 'circle':
                # Генерация случайного круга
                radius = random.randint(5, 20)
                center_x = random.randint(radius, self.width - radius - 1)
                center_y = random.randint(radius, self.height - radius - 1)

                # Добавляем круг на карту
                added_cells = self._add_circle(center_x, center_y, radius)

            else:  # rectangle
                # Генерация случайного прямоугольника
                rect_width = random.randint(5, 40)
                rect_height = random.randint(5, 40)
                x1 = random.randint(0, self.width - rect_width - 1)
                y1 = random.randint(0, self.height - rect_height - 1)
                x2 = x1 + rect_width
                y2 = y1 + rect_height

                # Добавляем прямоугольник на карту
                added_cells = self._add_rectangle(x1, y1, x2, y2)

            current_obstacle_cells += added_cells

            # Защита от бесконечного цикла
            if current_obstacle_cells > target_obstacle_cells * 1.5:
                print("Предупреждение: достигнут предел генерации препятствий")
                break

        return self.map

    def _add_circle(self, center_x: int, center_y: int, radius: int) -> int:
        """
        Добавление круга на карту

        Returns:
            Количество добавленных клеток
        """
        added_cells = 0
        for y in range(max(0, center_y - radius), min(self.height, center_y + radius + 1)):
            for x in range(max(0, center_x - radius), min(self.width, center_x + radius + 1)):
                # Проверяем, находится ли точка внутри круга
                if (x - center_x) ** 2 + (y - center_y) ** 2 <= radius ** 2:
                    if self.map[y, x] == 0:  # Если клетка свободна
                        self.map[y, x] = 1
                        added_cells += 1
        return added_cells

    def _add_rectangle(self, x1: int, y1: int, x2: int, y2: int) -> int:
        """
        Добавление прямоугольника на карту

        Returns:
            Количество добавленных клеток
        """
        added_cells = 0
        for y in range(max(0, y1), min(self.height, y2 + 1)):
            for x in range(max(0, x1), min(self.width, x2 + 1)):
                if self.map[y, x] == 0:  # Если клетка свободна
                    self.map[y, x] = 1
                    added_cells += 1
        return added_cells

    def save_map(self, filename: str = ""):
        """
        Сохранение карты в формате .npz

        Args:
            filename: имя файла для сохранения
        """
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
        """
        Загрузка карты из файла .npz

        Args:
            filename: имя файла для загрузки

        Returns:
            Загруженная карта
        """
        data = np.load(filename)
        self.map = data['map']
        self.width = data['width']
        self.height = data['height']
        return self.map

    def visualize(self, title: str = ""):
        """
        Визуализация карты
        """
        if title == "":
            title = f"Карта препятствий {int(self.density*100)}% (0=свободно, 1=препятствие)"

        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray_r', vmin=0, vmax=1)
        plt.title(title)
        plt.xlabel("X координата")
        plt.ylabel("Y координата")
        plt.show()


# Пример использования
if __name__ == "__main__":
    # Простой пример
    generator = MapGenerator(200, 200)

    # Генерация карты с плотностью 25%
    obstacle_map = generator.generate_obstacles(0.25)

    # Сохранение карты
    generator.save_map()

    # Визуализация
    generator.visualize()

    # Загрузка карты для проверки
    loaded_map = generator.load_map("map_25.npz")
    print(f"\nЗагруженная карта имеет размеры: {loaded_map.shape}")
    print(f"Количество препятствий: {np.sum(loaded_map)}")