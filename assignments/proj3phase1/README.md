# Project 3 Phase 1: State Estimation using EKF

Assigned: Apr 14, 2026

Due: May 1, 2026

## Overview

In this phase, you will implement an **Extended Kalman Filter (EKF)** for sensor fusion of **IMU measurements** and a **tag-based visual pose measurement produced by your `proj2phase1` `tag_detector`**.

The supplied ROS workspace already includes:

- an `ekf` package with a scaffold for the filter
- a `tag_detector` package carried over from `proj2phase1`
- launch files, configuration files, and a rosbag for evaluation

This phase does **not** assign new `tag_detector` work. The intended workflow is to reuse your working `proj2phase1` `tag_detector` implementation in `catkin_ws/src/tag_detector/`, so that it publishes `/tag_detector/odom_ref` for the EKF pipeline. If this directory still contains the old Project 2 scaffold or unfinished TODOs, replace it with your completed `proj2phase1` version before running this assignment.

Your job in this assignment is to complete the EKF logic in the scaffolded `ekf` package.

## Objectives

1. Implement IMU-based EKF prediction for a 15-state aerial robot model.
2. Implement EKF correction using the tag-based visual pose measurement produced by the reused `tag_detector`.
3. Maintain a consistent state, covariance, and frame convention throughout the filter.
4. Produce a stable fused pose estimate that behaves reasonably in RViz.

## Your Task

Complete the missing parts in `catkin_ws/src/ekf/src/ekf_node.cpp`.

You need to implement the following callbacks:

1. `imu_callback`
   - Read the incoming IMU message.
   - Compute the propagation time step.
   - Propagate the EKF state with the IMU measurements.
   - Propagate the covariance.

2. `odom_callback`
   - Read the visual pose measurement from `/tag_detector/odom_ref`, published by the reused `proj2phase1` `tag_detector`.
   - Convert the measurement to the IMU-aligned state convention used by your filter.
   - Perform the EKF measurement update.
   - Publish the fused odometry estimate.

The only new implementation work in this phase is in `catkin_ws/src/ekf/src/ekf_node.cpp`. The scaffold intentionally does not include every state variable you may need. If your implementation requires additional variables such as state storage, covariance storage, gravity, timestamps, or intermediate matrices, add them in the same file.

## State / Notation

Use the following 15-state model:

$$
x = \begin{bmatrix} p & rpy & v & b_g & b_a \end{bmatrix}^T
$$

where:

- $p \in \mathbb{R}^3$: position in the world frame
- $rpy \in \mathbb{R}^3$: roll, pitch, yaw
- $v \in \mathbb{R}^3$: velocity in the world frame
- $b_g \in \mathbb{R}^3$: gyroscope bias
- $b_a \in \mathbb{R}^3$: accelerometer bias

Use the following notation consistently in your implementation:

- `x`: state vector
- `P`: state covariance
- `Q`: process noise covariance
- `Rt`: measurement noise covariance
- `F`: continuous-time state Jacobian
- `U`: continuous-time noise input Jacobian, or an equivalent matrix with the same role
- `F_t = I + F dt`: first-order discrete-time transition matrix
- `V_t = U dt`: first-order discrete-time noise mapping

## Measurement and Frame Convention

The EKF receives the visual measurement from the topic `/tag_detector/odom_ref`, which is published by the reused `tag_detector` package and remapped to the private subscriber `~tag_odom` in `A3.launch`.

For this phase:

- treat `/tag_detector/odom_ref` as the visual pose measurement input to your EKF
- make sure your `proj2phase1` `tag_detector` implementation is working before debugging the EKF
- use the supplied fixed camera-to-IMU extrinsic values in `ekf_node.cpp`

The scaffold provides the following camera/IMU mounting information:

- camera origin expressed in the IMU frame: `(0.05, 0.05, 0.0)`
- camera-to-IMU rotation: `Quaternion(0, 1, 0, 0)` in `(w, x, y, z)` order
- equivalent rotation matrix:
  $$
  \begin{bmatrix}
  1 & 0 & 0 \\
  0 & -1 & 0 \\
  0 & 0 & -1
  \end{bmatrix}
  $$

Your EKF state is defined for the IMU-aligned body state. Therefore, before the measurement update, convert the provided visual pose measurement to the same IMU-aligned convention used by your state definition.

Be careful with:

- world frame vs IMU/body frame vs camera frame
- Euler-angle state representation vs rotation-matrix computation
- keeping your prediction and correction steps consistent with the same state convention

## Running the Simulation

1. **Prepare the reused `tag_detector` package**:
   - Make sure `catkin_ws/src/tag_detector/` contains your completed `proj2phase1` implementation.
   - In particular, your `tag_detector` should be able to publish `/tag_detector/odom_ref` when launched.

2. **Download the rosbag** into `catkin_ws/src/ekf/bag/`:
   ```bash
   cd catkin_ws/src/ekf/bag
   wget https://repo.hkust-uav.org/artifactory/generic-artifactory/rosbag/elec5660/ekf_A3.bag
   ```

3. **Build the workspace**:
   ```bash
   cd catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Launch the EKF system**:
   ```bash
   roslaunch ekf A3.launch
   ```

5. **Visualization**:
   - RViz will display the tag-based measurement from your reused `tag_detector`.
   - RViz will display your fused EKF odometry estimate.
   - The rosbag will provide synchronized IMU and camera data through the launch file.

## Evaluation Criteria

Your implementation will be evaluated on:

1. **Prediction Correctness**
   - The IMU propagation step uses a reasonable dynamic model and time step.
   - The covariance propagation is dimensionally and numerically consistent.

2. **Measurement Update Correctness**
   - The visual measurement is converted to the EKF state convention correctly.
   - The Kalman gain, state update, and covariance update are implemented correctly.

3. **Pose Accuracy**
   - The estimated state should remain accurate and consistent with the robot motion.
   - The fused trajectory should achieve a reasonable RMSE when compared with the ground-truth trajectory.

4. **Filter Stability**
   - The fused estimate should behave reasonably in RViz without obvious divergence or erratic jumps.

The evaluation assumes that your reused `proj2phase1` `tag_detector` can already provide a functioning visual measurement source. The accuracy, robustness, and stability of your `proj2phase1` implementation may also affect the final EKF performance.

## Provided Code Structure

- `catkin_ws/src/ekf/`: ROS package for the EKF assignment
  - `src/ekf_node.cpp`: **Your implementation goes here**
  - `launch/A3.launch`: Launch file for rosbag playback, tag detector, EKF, and RViz
  - `config/TA-camera.yml`: Camera calibration used by the provided tag detector
  - `config/tag_1.yml`: Tag board configuration used by the provided tag detector
  - `config/rviz_cfg.rviz`: RViz configuration
  - `bag/`: Directory for storing `ekf_A3.bag`
- `catkin_ws/src/tag_detector/`: Your reused `proj2phase1` package, expected to publish the visual pose measurement used by this phase

## Submission

Submit your completed code along with a brief report.

Code should include all necessary files to run your EKF in simulation.

The report should include the following sections with max 2 pages:

- Figures from RViz showing the raw tag-based measurement and the fused EKF result
- A short description of your prediction and correction model
- Parameter choices for `Q` and `Rt`, and any tuning observations
- Discussion of failure cases, drift, instability, or other behavior worth noting
- Any other things we should be aware of

Please submit a single zip file named `proj3phase1_yourname.zip` to Canvas.

Please cite the paper, GitHub repository, or any other resources you referred to while completing this assignment. Please keep [academic integrity](https://registry.hkust.edu.hk/resource-library/academic-integrity), plagiarism is not tolerated in this course.

## Late Submission Policy

Late submissions are accepted up to 7 days after the due date, with 5% (of the total grade of the item) penalty per day.
