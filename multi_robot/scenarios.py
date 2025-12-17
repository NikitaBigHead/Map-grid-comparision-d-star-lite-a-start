# multi_robot/scenarios.py
"""
Scenario generation for multi-robot comparison.
The secondary robot acts as a moving obstacle that crosses the primary robot's path.
"""

import numpy as np
from typing import List, Tuple, Dict


def create_multi_robot_scenario() -> Dict:
    """
    Create a scenario where the secondary robot crosses a narrow corridor,
    temporarily blocking the primary robot's path.
    
    Returns:
        Dictionary with map, start/goal positions, and secondary robot path.
    """
    height, width = 80, 120
    map_grid = np.zeros((height, width), dtype=np.uint8)

    # Create a narrow horizontal corridor at y = 35 to 45 (10 cells wide)
    map_grid[35:45, :55] = 1   # Left wall of corridor
    map_grid[35:45, 75:] = 1   # Right wall of corridor

    # Primary robot goes straight through the corridor
    primary_start = (15, 40)   # Left side of corridor
    primary_goal = (105, 40)   # Right side of corridor

    # Secondary robot starts above the corridor and crosses it vertically
    secondary_path: List[Tuple[int, int]] = [
        (65, 15),   # Start above corridor
        (65, 25),
        (65, 32),   # Approaching corridor top
        (65, 38),   # Entering corridor
        (65, 40),   # BLOCKING the primary path (center of corridor)
        (65, 42),   # Still blocking
        (65, 48),   # Exiting corridor bottom
        (65, 60),
        (65, 70),   # Moving away
    ]

    return {
        'map': map_grid,
        'primary_start': primary_start,
        'primary_goal': primary_goal,
        'secondary_path': secondary_path,
        'description': 'Secondary robot crosses narrow corridor, temporarily blocking primary path.'
    }