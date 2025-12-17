# multi_robot/scenarios.py
import numpy as np
from typing import Dict

def create_multi_robot_scenario() -> Dict:
    """
    GUARANTEED WORKING SCENARIO.
    Primary robot path: (10, 30) ---> (90, 30) (straight line along Y=30).
    Secondary robot will block it at (50, 30).
    """
    height, width = 60, 100
    map_grid = np.zeros((height, width), dtype=np.uint8)
    
    primary_start = (10, 30)
    primary_goal = (90, 30)
    
    # The secondary robot's PATH MUST include (50, 30) or a cell very close to it.
    # It starts above the path and moves down to block it.
    secondary_path = [
        # Start position (above the primary's path)
        (50, 45),
        # Move down towards the path
        (50, 40),
        (50, 35),
        # BLOCK THE PATH! This is the critical point.
        (50, 30), (50, 30), (50, 30),
        # Move away
        (50, 25)
    ]
    
    return {
        'map': map_grid,
        'primary_start': primary_start,
        'primary_goal': primary_goal,
        'secondary_path': secondary_path,
        'description': 'Secondary robot blocks primary path at (50, 30).'
    }