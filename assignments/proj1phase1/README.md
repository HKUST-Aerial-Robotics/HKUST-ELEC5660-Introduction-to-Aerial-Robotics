# Project 1 Phase 1: Quadrotor Control

Assignd: Feb 24, 2026

Due: Mar 3, 2026

## Overview

In this phase, you will implement a controller for a quadrotor to track different trajectories. The goal is to understand the basic control principles and implement a controller that can stabilize the quadrotor and follow desired trajectories.

## Objectives

1. Implement a controller for quadrotor trajectory tracking.
2. Test your controller with different trajectories:
   - Hover (stationary at origin)
   - Circle (helical trajectory with increasing radius)
   - Square (piecewise linear path through waypoints)
3. Tune controller gains to achieve stable and accurate trajectory tracking

**Bonus points** will be given if you write your own trajectory besides the above three.

| ![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/newplot.png) | ![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/newplot%20(1).png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

## System Model

The quadrotor has 13 states:
- **Position** (x, y, z): 3D position in world frame
- **Velocity** (vx, vy, vz): Linear velocity in world frame
- **Quaternion** (qw, qx, qy, qz): Attitude representation
- **Angular velocity** (ωx, ωy, ωz): Angular velocity in body frame

The controller outputs:
- **F**: Total thrust force
- **M**: Moment vector [Mx, My, Mz] in body frame

## Your Task

Complete the `Controller` class in `sim/controller.py`:

Compute the control inputs `F` and `M` based on the current state `s` and desired state `s_des`.

**Important**: Remember to wrap angles to [-π, π] using `wrap_to_pi()` when computing angle errors.

## Running the Simulation

1. Run the web interface:
   ```bash
   python app.py
   ```

2. Open your browser to `http://localhost:8080`

3. Select a trajectory and click "Run Simulation"

## Evaluation Criteria

Your controller will be evaluated on:
1. **Stability**: The quadrotor should not diverge or oscillate excessively
2. **Tracking accuracy**: Minimize position and velocity errors
3. **Smoothness**: Avoid sudden control inputs or jerky motion

## Provided Code Structure

- `app.py`: Web interface for running simulations
- `sim/controller.py`: **Your implementation goes here**
- `sim/trajectories.py`: Predefined trajectory functions, **you can add your own trajectory here**
- `sim/simulator.py`: Simulation engine
- `sim/dynamics.py`: Quadrotor dynamics model
- `sim/model.py`: Quadrotor parameters
- `sim/math_utils.py`: Helper functions for rotations
- `sim/visualization.py`: Plotting utilities

## Tips

1. Start with the hover trajectory to test basic stabilization
2. The state vector `s` and desired state `s_des` have this structure:
   - `s[0:3]`: position [x, y, z]
   - `s[3:6]`: velocity [vx, vy, vz]
   - `s[6:10]`: quaternion [qw, qx, qy, qz]
   - `s[10:13]`: angular velocity [ωx, ωy, ωz]
   - `s_des[0:9]`: desired [position, velocity, acceleration]
   - `s_des[9:11]`: desired [yaw, yaw_rate]

## Submission

Submit your completed code along with a brief report.

Code should include all necessary files to run your controller in simulation.

The report should include the following sections with max 2 pages:

- Figures plotted by simulator.
- Statistics about your controller. (For example, RMS error between current state and desired state for position, velocity).
- Analysis of your result. (For example, parameter studies).
- Any other things we should be aware of

Please submit a single zip file named `proj1phase1_yourname.zip` to the canvas.

Please cite the paper, GitHub repository, or any other resources you referred to while completing this assignment.  Please keep [academic integrity](https://registry.hkust.edu.hk/resource-library/academic-integrity), plagiarism is not tolerated in this course.
