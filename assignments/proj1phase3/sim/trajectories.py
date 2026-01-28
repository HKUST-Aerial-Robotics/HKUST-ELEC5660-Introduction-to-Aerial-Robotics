from __future__ import annotations

import numpy as np

from .path_planning import path_from_a_star
from .trajectory_generator import TrajectoryGenerator


MAPS = {
    "Map 1": np.array(
        [
            [1.0, 1.0, 1.0],
            [1.0, 2.0, 1.0],
            [3.0, 3.0, 1.0],
            [3.0, 7.0, 1.0],
            [1.0, 5.0, 1.0],
            [3.0, 5.0, 1.0],
            [2.0, 7.0, 1.0],
            [2.0, 9.0, 1.0],
        ]
    ),
    "Map 2": np.array(
        [
            [1.0, 1.0, 1.0],
            [2.0, 1.0, 1.0],
            [3.0, 3.0, 1.0],
            [1.0, 3.0, 1.0],
            [2.0, 5.0, 1.0],
            [4.0, 5.0, 1.0],
            [3.0, 7.0, 1.0],
            [4.0, 9.0, 1.0],
        ]
    ),
}


def _map3_points(rng: np.random.Generator) -> np.ndarray:
    points = [[1.0, 1.0, 1.0]]
    seeds_a = rng.integers(1, 36, size=4)
    seeds_b = rng.integers(35, 71, size=4)
    seeds = np.concatenate([seeds_a, seeds_b])
    for seed in seeds:
        z_coord = seed // 35 + 1
        y_coord = (seed % 35) // 5 + 2
        x_coord = (seed % 35) % 5 + 1
        points.append([float(x_coord), float(y_coord), float(z_coord)])
    points.append([5.0, 9.0, 1.0])
    return np.array(points, dtype=float)


def build_map(map_name: str, rng: np.random.Generator | None = None) -> np.ndarray:
    if map_name in MAPS:
        return MAPS[map_name]
    if map_name == "Map 3":
        rng = rng or np.random.default_rng()
        return _map3_points(rng)
    raise KeyError(f"Unknown map: {map_name}")


def build_path_from_map(map_points: np.ndarray) -> np.ndarray:
    return path_from_a_star(map_points)


def build_path(map_name: str, rng: np.random.Generator | None = None) -> np.ndarray:
    map_points = build_map(map_name, rng)
    return build_path_from_map(map_points)


def build_generator_from_map(map_points: np.ndarray, method: str = "jerk") -> TrajectoryGenerator:
    path = build_path_from_map(map_points)
    return TrajectoryGenerator(path, method=method)


def build_generator(map_name: str, method: str = "jerk", rng: np.random.Generator | None = None) -> TrajectoryGenerator:
    map_points = build_map(map_name, rng)
    return build_generator_from_map(map_points, method)


def trajectory_fn_from_generator(generator: TrajectoryGenerator):
    def _trajectory(t: float, true_s: np.ndarray) -> np.ndarray:
        return generator.evaluate(t)

    return _trajectory
