# Project 1 Phase 3: Path Planning and System Integration

Assignd: Mar 10, 2026

Due: Mar 17, 2026

## Overview

In Phase 1 you implemented a quadrotor controller, and in Phase 2 you built an optimization-based trajectory generator. In this phase of Project 1, you will add a **Path Planning** module (A* on a 3D grid) and integrate all modules into a complete navigation stack:

`Map (start/obstacles/target) → A* path (waypoints) → trajectory generator → controller → flight`

## Objectives

1. Implement a collision-free path planner using **A\*** search on a 3D grid.
2. Convert the planned path into smooth polynomial trajectories using your Phase 2 generator.
3. Demonstrate autonomous navigation on multiple obstacle maps.

| ![image-20260128224739032](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260128224739032.png) | ![image-20260128224808511](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260128224808511.png) | ![image-20260128224835463](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260128224835463.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |

## Environment / Map Representation

The environment is a 3D grid with size `grid_size = (10, 10, 10)`.

The map is provided as an array `map_points` with shape `(N, 3)`:

- `map_points[0]`: start cell index `[x, y, z]`
- `map_points[-1]`: target cell index `[x, y, z]`
- `map_points[1:-1]`: obstacle cell indices

Important coordinate convention:

- Grid indices are **1-based** (i.e., valid `x` is in `[1, 10]`).
- The simulator uses cell **centers** in meters, computed as:
  $$p = [x, y, z] - 0.5$$
  For example, grid index `[1, 1, 1]` corresponds to position `[0.5, 0.5, 0.5]`.

Your A* output should follow the simulator convention (centers in meters).

## Your Task

### 1) Path Planning (A*)

Complete `path_from_a_star` in `sim/path_planning.py`.

- **Input**: `map_points` and optional `grid_size`.
- **Output**: `path_points` with shape `(M, 3)` in **cell-center coordinates** (i.e., `index - 0.5`).
- **Connectivity**: Use **6-connected** neighbors (±1 step in one axis each move).
- **Requirements**:
  - Start at `map_points[0] - 0.5` and end at `map_points[-1] - 0.5`.
  - Never step into obstacle cells (`map_points[1:-1]`).
  - Stay within grid bounds.

### 2) System Integration

This phase reuses your previous work:

- **Controller**: copy your Phase 1 implementation into `sim/controller.py`.
- **Trajectory Generator**: copy your Phase 2 implementation into `sim/trajectory_generator.py`.

The provided UI will call your trajectory generator with a `method` string in `{ "smooth", "jerk", "snap" }`.

## Running the Simulation

1. Run the web interface:
    ```bash
    python app.py
    ```

2. Open your browser to `http://localhost:8080`.

3. Select:
    - **Map**: `Map 1`, `Map 2`, or `Map 3` (random obstacles)
    - **Optimization**: `Smooth Only` / `Minimum Jerk` / `Minimum Snap`
    - **Disturbance std (N)**: external force noise level

4. Click **Run simulation**.

The UI will display a 3D view of the map (start/target/obstacles), the A* path, and the tracked trajectory, along with RMSE and smoothness metrics.

## Evaluation Criteria

Your solution will be evaluated on:

1. **Path Validity**
    - The returned path starts/ends at the correct cells.
    - The path is collision-free (no obstacle cells) and stays within bounds.
    - Consecutive waypoints are 6-connected neighbors.

2. **Path Quality**
    - Reasonable path length. (With an admissible heuristic, A* returns an optimal shortest path on the grid graph.)

3. **End-to-end Execution**
    - The quadrotor tracks the generated trajectory and reaches the target without diverging.
    - Lower tracking RMSE and smoother controls are better (given the same disturbance level).

## Provided Code Structure

- `app.py`: Web UI to run the full navigation stack
- `sim/path_planning.py`: **Your A\* implementation goes here**
- `sim/trajectory_generator.py`: Copy your Phase 2 implementation here
- `sim/controller.py`: Copy your Phase 1 implementation here
- `sim/trajectories.py`: Map definitions and helper builders (`build_map`, `build_path_from_map`, `build_generator_from_map`)
- `sim/simulator.py`: Simulation engine and metric computation
- `sim/dynamics.py`: Quadrotor dynamics model
- `sim/model.py`: Quadrotor parameters
- `sim/math_utils.py`: Rotation utilities and `wrap_to_pi`
- `sim/visualization.py`: Plotting utilities (map cubes, A* path, trajectories)

## Tips

1. Represent obstacle cells as a `set` of integer tuples for fast membership checks.
2. Use a priority queue (`heapq`) for the A* open set.
3. Use an admissible heuristic (e.g., Manhattan distance on the grid) so A* remains optimal.
4. Store `came_from` and `g_score` dictionaries to reconstruct the final path.
5. Carefully handle the index/position conversion:
    - planning is easiest in **integer grid indices**
    - output should be **float positions** at cell centers: `idx - 0.5`

## Submission

Submit your completed code along with a brief report.

Code should include all necessary files to run the full phase3 stack in simulation.

The report should include the following sections with max 2 pages:

- A* implementation details (state, neighbor expansion, heuristic).
- Results on Map 1/2/3 (screenshots/figures of path + trajectory).
- Statistics (e.g., path length, RMSE position/velocity/yaw, smoothness score).
- Discussion of failure cases (if any) and how you handled them.
- Any other things we should be aware of

Please submit a single zip file named `proj1phase3_yourname.zip` to the canvas.

Please cite the paper, GitHub repository, or any other resources you referred to while completing this assignment. Please keep [academic integrity](https://registry.hkust.edu.hk/resource-library/academic-integrity), plagiarism is not tolerated in this course.
