from __future__ import annotations

import numpy as np

from .trajectory_generator import TrajectoryGenerator


PATHS = {
    "Path 1": np.array(
        [
            [0.00, 0.00, 1.0],
            [0.25, 0.25, 1.0],
            [-0.50, 0.50, 1.0],
            [-0.75, -0.75, 1.0],
            [1.00, -1.00, 1.0],
            [1.25, 1.25, 1.0],
            [-1.50, 1.50, 1.0],
            [-1.75, -1.75, 1.0],
            [2.00, -2.00, 1.0],
            [2.25, 2.25, 1.0],
            [-2.50, 2.50, 1.0],
        ]
    ),
    "Path 2": np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [2.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [-1.1, 0.0, 1.0],
            [-1.2, 0.0, 1.0],
            [-1.3, 0.0, 1.0],
            [-3.0, 0.0, 1.0],
        ]
    ),
    "Path 3": np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 2.0, 1.0],
            [1.0, 3.0, 1.0],
            [-1.0, 4.0, 1.0],
            [1.0, 5.0, 1.0],
            [-1.0, 6.0, 1.0],
            [1.0, 7.0, 1.0],
            [-1.0, 8.0, 1.0],
            [1.0, 9.0, 1.0],
            [0.0, 10.0, 1.0],
        ]
    ),
}


def build_generator(path_name: str, method: str = "snap") -> TrajectoryGenerator:
    if path_name not in PATHS:
        raise KeyError(f"Unknown path: {path_name}")
    return TrajectoryGenerator(PATHS[path_name], method=method)


def trajectory_fn_from_generator(generator: TrajectoryGenerator):
    def _trajectory(t: float, true_s: np.ndarray) -> np.ndarray:
        return generator.evaluate(t)

    return _trajectory
