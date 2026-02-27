# Project 1 Phase 2: Trajectory Generation

Assignd: Mar 3, 2026

Due: Mar 13, 2026

## Overview

In Phase 1, you have implemented a controller to track trajectories. In Phase 2, you will focus on the **Trajectory Generator**. A carefully designed trajectory generator enables the quadrotor to operate aggressively and precisely.

## Objectives

1.  Implement trajectory generation algorithms to connect a sequence of waypoints.
2.  Ensure trajectories satisfy smoothness criteria (continuity in position, velocity, acceleration).
3.  Implement optimization-based trajectory generation (Minimum Jerk or Minimum Snap).

| ![image-20260128220915420](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260128220915420.png) | ![image-20260128220950766](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260128220950766.png) | ![image-20260128221017859](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/image-20260128221017859.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |

## Your Task

Complete the `TrajectoryGenerator` class in `sim/trajectory_generator.py`. You need to implement the following functions:

1.  `_generate_smooth_only`: A naive method that ensures geometric continuity ($C^2$) but doesn't minimize a cost function. (Uses quintic splines).
2.  `_generate_minimum_jerk`: Minimizes the jerk (3rd derivative) squared integral. (Uses 6th order polynomials).
3.  `_generate_minimum_snap`: Minimizes the snap (4th derivative) squared integral. (Uses 8th order polynomials).

## Running the Simulation

1.  Run the web interface:
    ```bash
    python app.py
    ```

2.  Open your browser to `http://localhost:8080`

3.  Select a "Path" (set of waypoints) and an "Optimization" method.
    -   *Smooth Only*
    -   *Minimum Jerk*
    -   *Minimum Snap*

4.  Click "Run Simulation".

## Evaluation Criteria

1.  **Waypoint Connection**: The trajectory must pass through all defined waypoints.
2.  **Smoothness**: The quadrotor should fly smoothly without discontinuities in position, velocity, or acceleration.
3.  **Tracking Error**: A better trajectory (like Min Snap) should result in lower tracking error given the same controller.

## Tips

-   You are provided with a helper function `_solve_equality_qp(H, Aeq, beq)` which solves the standard Quadratic Program:
    $$ \min \frac{1}{2} x^T H x \quad \\ \text{s.t.} \quad A_{eq} x = b_{eq} $$
    You can use this to solve for the polynomial coefficients.
-   Review Lecture 4 slides on Trajectory Generation.
-   Map the geometric constraints (waypoints, continuity) to the linear constraints $A_{eq} x = b_{eq}$.
-   Map the cost function integrals to the Hessian matrix $H$.

## Provided Code Structure

- `app.py`: Web interface for running simulations
- `sim/trajectory_generator.py`: **Your implementation goes here**
- `sim/trajectories.py`: Predefined paths (waypoints) and helper trajectory utilities
- `sim/controller.py`: Controller from Phase 1 (used to track the generated trajectory)
- `sim/simulator.py`: Simulation engine
- `sim/dynamics.py`: Quadrotor dynamics model
- `sim/model.py`: Quadrotor parameters
- `sim/math_utils.py`: Helper functions for rotations
- `sim/visualization.py`: Plotting utilities

## Submission

Submit your completed code along with a brief report.

Code should include all necessary files to run your trajectory generator in simulation.

The report should include the following sections with max 2 pages:

- Figures plotted by simulator (compare Smooth Only / Min Jerk / Min Snap on at least one path).
- Statistics about your trajectories/tracking (e.g., RMS position error, peak velocity/acceleration/jerk).
- Analysis of your results (e.g., parameter/time-allocation studies).
- Any other things we should be aware of.

Please submit a single zip file named `proj1phase2_yourname.zip` to the canvas.

Please cite the paper, GitHub repository, or any other resources you referred to while completing this assignment. Please keep [academic integrity](https://registry.hkust.edu.hk/resource-library/academic-integrity), plagiarism is not tolerated in this course.

## Late Submission Policy

Late submissions are accepted up to 7 days after the due date, with 5% (of the total grade of the item) penalty per day.
