import numpy as np
from grid_map import GridMap

def bounds_from_center(center: tuple[int, int], eff_size: int):
    cx, cy = center
    hl = eff_size // 2
    hr = eff_size - 1 - hl
    return (
        cx - hl,
        cy - hl,
        cx + hr + 1,
        cy + hr + 1,
    )


def rects_overlap(a, b) -> bool:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    return not (ax1 <= bx0 or bx1 <= ax0 or ay1 <= by0 or by1 <= ay0)


def center_ok(center, eff_size, used_centers):
    ra = bounds_from_center(center, eff_size)
    for c in used_centers:
        rb = bounds_from_center(c, eff_size)
        if rects_overlap(ra, rb):
            return False
    return True


def sample_center_unique(
    gm: GridMap,
    size: int,
    clearance: int,
    rng: np.random.Generator,
    used_centers: list[tuple[int, int]],
    max_tries: int = 20000,
):
    eff_size = size + 2 * clearance

    for _ in range(max_tries):
        p = gm.random_valid_center_with_clearance(size, clearance, rng)
        if center_ok(p, eff_size, used_centers):
            return p

    raise RuntimeError(
        "Could not sample non-overlapping center "
        "(try fewer robots, smaller size/clearance, or lower obstacle density)"
    )


