# seed_manager.py
from __future__ import annotations
import hashlib

def _u32(*parts) -> int:
    s = "|".join(map(str, parts)).encode("utf-8")
    h = hashlib.blake2b(s, digest_size=8).digest()
    return int.from_bytes(h, "little") & 0xFFFFFFFF

class SeedManager:
    def __init__(self, master_seed: int):
        self.master_seed = int(master_seed)

    # одинаковые карты между задачами (НЕ зависит от N роботов)
    def map_seed(self, density: float, map_idx: int) -> int:
        return _u32(self.master_seed, "map", round(float(density), 4), int(map_idx))

    # одинаковые starts/goals внутри задачи и эксперимента (зависит от N)
    def scenario_seed(self, density: float, map_idx: int, n_robots: int) -> int:
        return _u32(self.master_seed, "scenario", round(float(density), 4), int(map_idx), int(n_robots))

    # чтобы цвета были одинаковы у A* и D* в одном эксперименте
    def colors_seed(self, density: float, map_idx: int, n_robots: int) -> int:
        return _u32(self.master_seed, "colors", round(float(density), 4), int(map_idx), int(n_robots))
