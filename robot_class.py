import numpy as np
import random
from typing import Tuple, List

from  map_generator import MapGenerator


class Robot:
    def __init__(self, obstacle_map: np.ndarray):
        """
        Инициализация робота

        Args:
            obstacle_map: карта препятствий (0=свободно, 1=препятствие)
        """
        self.map = obstacle_map
        self.height, self.width = obstacle_map.shape
        self.size = 5  # Размер робота 5x5

        # Координаты центра робота (нечетные координаты для симметрии 5x5)
        self.x = None
        self.y = None

        # Определения 8 направлений движения (dx, dy)
        self.directions = {
            'N': (0, -1),  # Север
            'NE': (1, -1),  # Северо-восток
            'E': (1, 0),  # Восток
            'SE': (1, 1),  # Юго-восток
            'S': (0, 1),  # Юг
            'SW': (-1, 1),  # Юго-запад
            'W': (-1, 0),  # Запад
            'NW': (-1, -1)  # Северо-запад
        }

    def set_random_position(self) -> bool:
        """
        Установка робота в случайную свободную позицию

        Returns:
            True если удалось разместить, False если нет свободных мест
        """
        # Список всех возможных позиций для центра робота
        # Центр должен быть не менее чем в 2 клетках от края, чтобы робот полностью помещался
        possible_positions = []

        for y in range(2, self.height - 2):
            for x in range(2, self.width - 2):
                if self._is_position_valid(x, y):
                    possible_positions.append((x, y))

        if not possible_positions:
            print("Нет свободных позиций для размещения робота!")
            return False

        # Выбираем случайную позицию
        self.x, self.y = random.choice(possible_positions)
        print(f"Робот размещен в позиции ({self.x}, {self.y})")
        return True

    def set_position(self, x: int, y: int) -> bool:
        """
        Установка робота в указанную позицию

        Args:
            x: координата X центра
            y: координата Y центра

        Returns:
            True если позиция допустима, False если нет
        """
        if self._is_position_valid(x, y):
            self.x = x
            self.y = y
            print(f"Робот установлен в позицию ({x}, {y})")
            return True
        else:
            print(f"Позиция ({x}, {y}) недоступна!")
            return False

    def _is_position_valid(self, x: int, y: int) -> bool:
        """
        Проверка, можно ли разместить робота в указанной позиции

        Args:
            x: координата X центра
            y: координата Y центра

        Returns:
            True если позиция доступна, False если есть столкновение или выход за границы
        """
        # Проверка границ
        # Для робота 5x5 с центром в (x, y), он занимает клетки от x-2 до x+2 и y-2 до y+2
        if x < 2 or x >= self.width - 2 or y < 2 or y >= self.height - 2:
            return False

        # Проверка столкновений
        for dy in range(-2, 3):  # -2, -1, 0, 1, 2
            for dx in range(-2, 3):
                map_x = x + dx
                map_y = y + dy

                # Проверяем, не выходит ли за границы (дополнительная проверка)
                if 0 <= map_x < self.width and 0 <= map_y < self.height:
                    # Если клетка занята препятствием
                    if self.map[map_y, map_x] == 1:
                        return False
                else:
                    return False

        return True

    def check_collision(self, x: int, y: int) -> bool:
        """
        Проверка столкновения робота с препятствиями в указанной позиции

        Args:
            x: координата X центра для проверки
            y: координата Y центра для проверки

        Returns:
            True если есть столкновение, False если позиция свободна
        """
        return not self._is_position_valid(x, y)

    def get_occupied_cells(self) -> List[Tuple[int, int]]:
        """
        Получение списка клеток, которые занимает робот

        Returns:
            Список координат (x, y) занятых клеток
        """
        if self.x is None or self.y is None:
            return []

        occupied = []
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                occupied.append((self.x + dx, self.y + dy))
        return occupied

    def move(self, direction: str, steps: int = 1) -> bool:
        """
        Перемещение робота в заданном направлении

        Args:
            direction: направление ('N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW')
            steps: количество шагов

        Returns:
            True если перемещение успешно, False если есть столкновение
        """
        if self.x is None or self.y is None:
            print("Робот не размещен на карте!")
            return False

        if direction not in self.directions:
            print(f"Неизвестное направление: {direction}")
            print(f"Допустимые направления: {list(self.directions.keys())}")
            return False

        dx, dy = self.directions[direction]

        # Целевая позиция
        target_x = self.x + dx * steps
        target_y = self.y + dy * steps

        # Проверяем, доступна ли целевая позиция
        if self._is_position_valid(target_x, target_y):
            old_x, old_y = self.x, self.y
            self.x, self.y = target_x, target_y
            print(f"Робот перемещен из ({old_x}, {old_y}) в ({self.x}, {self.y})")
            return True
        else:
            print(f"Невозможно переместиться в позицию ({target_x}, {target_y}) - столкновение!")
            return False

    def move_to(self, x: int, y: int) -> bool:
        """
        Попытка переместить робота в указанную позицию

        Args:
            x: целевая координата X центра
            y: целевая координата Y центра

        Returns:
            True если перемещение успешно, False если есть столкновение
        """
        if self._is_position_valid(x, y):
            old_x, old_y = self.x, self.y
            self.x, self.y = x, y
            print(f"Робот перемещен из ({old_x}, {old_y}) в ({self.x}, {self.y})")
            return True
        else:
            print(f"Невозможно переместиться в позицию ({x}, {y}) - столкновение!")
            return False

    def get_position(self) -> Tuple[int, int]:
        """
        Получение текущей позиции робота

        Returns:
            Кортеж (x, y) координат центра
        """
        return (self.x, self.y)

    def get_vision_mask(self, radius: int = 10) -> np.ndarray:
        """
        Создание маски видимости робота

        Args:
            radius: радиус видимости

        Returns:
            Маска видимости (1 - видимо, 0 - не видимо)
        """
        mask = np.zeros((self.height, self.width), dtype=np.uint8)

        if self.x is None or self.y is None:
            return mask

        for y in range(max(0, self.y - radius), min(self.height, self.y + radius + 1)):
            for x in range(max(0, self.x - radius), min(self.width, self.x + radius + 1)):
                # Проверяем, находится ли точка в радиусе
                if (x - self.x) ** 2 + (y - self.y) ** 2 <= radius ** 2:
                    mask[y, x] = 1

        return mask

    def visualize(self, show_vision: bool = False, vision_radius: int = 10) -> None:
        """
        Визуализация робота на карте

        Args:
            show_vision: показывать ли область видимости
            vision_radius: радиус видимости
        """
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle, Circle

        # Создаем копию карты для визуализации
        vis_map = self.map.copy().astype(float)

        # Помечаем клетки робота
        if self.x is not None and self.y is not None:
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    vis_x = self.x + dx
                    vis_y = self.y + dy
                    if 0 <= vis_x < self.width and 0 <= vis_y < self.height:
                        # Робот - значение 0.5 (серый цвет)
                        vis_map[vis_y, vis_x] = 0.5

            # Центр робота - другое значение
            vis_map[self.y, self.x] = 0.8

        # Визуализация
        fig, ax = plt.subplots(figsize=(12, 10))

        # Отображаем карту
        im = ax.imshow(vis_map, cmap='RdYlBu_r', vmin=0, vmax=1)

        # Добавляем цветовую легенду
        cbar = fig.colorbar(im, ax=ax, ticks=[0, 0.5, 0.8, 1])
        cbar.ax.set_yticklabels(['Свободно', 'Тело робота', 'Центр робота', 'Препятствие'])

        # Добавляем область видимости если нужно
        if show_vision and self.x is not None and self.y is not None:
            vision_circle = Circle((self.x, self.y), vision_radius,
                                   fill=False, color='green', linewidth=2, alpha=0.7)
            ax.add_patch(vision_circle)

        # Добавляем границы робота
        if self.x is not None and self.y is not None:
            robot_rect = Rectangle((self.x - 2.5, self.y - 2.5), 5, 5,
                                   fill=False, color='red', linewidth=3)
            ax.add_patch(robot_rect)

        ax.set_title(f"Робот 5x5 на карте (позиция: ({self.x}, {self.y}))")
        ax.set_xlabel("X координата")
        ax.set_ylabel("Y координата")
        ax.grid(True, alpha=0.3)
        plt.show()


# Демонстрационный пример
def robot_demo():
    """Демонстрация работы робота"""

    # Создаем простую карту с меньшей плотностью для демонстрации
    generator = MapGenerator(200, 200)
    obstacle_map = generator.generate_obstacles(0.15)  # 15% плотность

    # Создаем робота
    robot = Robot(obstacle_map)

    # Размещаем робота
    if not robot.set_random_position():
        # Если не удалось, попробуем конкретную позицию
        robot.set_position(50, 50)

    # Простой путь для демонстрации
    path = ['E', 'E', 'E', 'SE', 'S', 'S', 'SW', 'W', 'W', 'NW', 'N', 'N']

    print("=== Демонстрация перемещения робота ===")
    print(f"Начальная позиция: {robot.get_position()}")

    for i, direction in enumerate(path):
        print(f"\nШаг {i + 1}: Двигаемся {direction}")
        success = robot.move(direction)
        if not success:
            print(f"  Прервано из-за столкновения!")
            break

    print(f"\nФинальная позиция: {robot.get_position()}")

    # Визуализация
    robot.visualize(show_vision=True, vision_radius=12)


if __name__ == "__main__":
    # Запуск теста
    robot_demo()
