from __future__ import annotations

import numpy as np


def path_from_a_star(map_points: np.ndarray, grid_size: tuple[int, int, int] = (10, 10, 10)) -> np.ndarray:
    """
    Plan a path on a 3D grid using A* (6-connected).

    Args:
        map_points: An (N, 3) array where:
                    - map_points[0] is the start [x, y, z]
                    - map_points[-1] is the target [x, y, z]
                    - map_points[1:-1] are obstacle locations in the grid.
                    All coordinates are floats but correspond to integer grid centers.
                    Note: The grid indices are 1-based (i.e. x in [1, max_x]).
        grid_size: A tuple (max_x, max_y, max_z) defining the dimensions of the grid.

    Returns:
        np.ndarray: An (M, 3) array of waypoints for the path.
                    The waypoints should be the center of the grid cells (i.e., index - 0.5).
                    The path must start with the start position and end with the target position.
    """
    map_points = np.asarray(map_points, dtype=float)
    if map_points.shape[0] < 2:
        raise ValueError("map_points must contain at least start and target")

    # TODO: Implement A* algorithm here

    # 1. Parse start, target, and obstacles
    # 2. Initialize open/closed lists (or queue/visited sets)
    # 3. Perform A* search
    # 4. Reconstruct path

    raise NotImplementedError("path_from_a_star not implemented")
