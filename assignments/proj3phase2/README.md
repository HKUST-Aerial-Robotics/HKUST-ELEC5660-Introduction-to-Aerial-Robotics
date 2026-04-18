# Project 3 Phase 2: Augmented-State EKF Fusion

Assigned: Apr 21, 2026

Due: May 8, 2026

## Overview

In this phase, you will extend the Project 3 Phase 1 EKF into an **Augmented-State EKF** that fuses three sources of information:

- IMU measurements
- absolute pose measurements from marker-based PnP
- relative pose measurements from stereo visual odometry

The release workspace already includes:

- a `tag_detector` package that mirrors the **Project 2 Phase 1**
- a `stereo_vo` package that mirrors the **Project 2 Phase 2**
- an `aug_ekf` package that contains the scaffold for this phase

Your main new task in this project is to implement the Augmented-State EKF logic inside `aug_ekf`.

The included `tag_detector` and `stereo_vo` packages are intentionally shipped as earlier assignment release versions. For the full end-to-end pipeline, you may need to bring forward your own implementations from Project 2 Phase 1 and Project 2 Phase 2 into these scaffolds.

Notes:

1. This project is difficult. Start early.
2. This is an individual project.

## Objectives

1. Extend a 15-state EKF to a 21-state augmented filter by copying the keyframe pose.
2. Fuse both absolute and relative pose measurements in one consistent filter.
3. Handle keyframe changes and out-of-order measurements correctly.
4. Handle temporary loss of either the absolute or the relative pose measurement stream.
5. Publish a stable fused odometry result and a `nav_msgs::Path` for visualization.

## Your Task

Complete the missing logic in the following files:

- `catkin_ws/src/aug_ekf/src/ekf_model.cpp`
- `catkin_ws/src/aug_ekf/src/ekf_filter.cpp`

### Functions in `ekf_model.cpp`

You need to implement:

- `modelF`
- `jacobiFx`
- `jacobiFn`
- `modelG1`
- `jacobiG1x`
- `jacobiG1v`
- `modelG2`
- `jacobiG2x`
- `jacobiG2v`

These functions define the process model, measurement models, and the corresponding Jacobians used by the filter.

### Functions in `ekf_filter.cpp`

You need to implement:

- `PnPCallback`
- `stereoVOCallback`
- `imuCallback`
- `predictIMU`
- `updatePnP`
- `updateVO`
- `changeAugmentedState`
- `processNewState`
- `insertNewState`
- `repropagate`
- `removeOldState`
- `initFilter`
- `initUsingPnP`

These functions define how new measurements are inserted into the history queue, how the filter is initialized, how the augmented state is updated, and how re-propagation is handled when measurements arrive out of order.

## State Definition and Notation

The augmented state has 21 variables:

$$
\begin{bmatrix}
x & y & z & \phi & \theta & \psi & v_x & v_y & v_z & b_{gx} & b_{gy} & b_{gz} & b_{ax} & b_{ay} & b_{az} & x_k & y_k & z_k & \phi_k & \theta_k & \psi_k
\end{bmatrix}^T
$$

The first 15 states are the current EKF state:

- position in the world frame
- Euler angles `(phi, theta, psi)`
- velocity in the world frame
- gyroscope bias
- accelerometer bias

The last 6 states are the copied pose of the current visual-odometry keyframe:

- keyframe position in the world frame
- keyframe Euler angles

The scaffold uses the following notation:

- `Qt_`: IMU process noise covariance
- `Rt1_`: PnP measurement noise covariance
- `Rt2_`: stereo VO relative-pose measurement noise covariance
- `M_a_`: state augmentation matrix
- `M_r_`: state reduction matrix

## Measurement Conventions

### Absolute pose from PnP

The filter subscribes to a private topic named `~tag_odom`.

- In `augekf_simple.launch`, this is remapped to `/tag_detector/odom_ref`.
- In `augekf.launch`, this is remapped to `/tag_detector/odom_yourwork`.

This measurement should be interpreted as an **absolute body pose in the world frame** before it is fused by the EKF.

Important:

- In the release package, `tag_detector` is an earlier assignment scaffold.
- In `augekf_simple.launch`, the bag already contains the precomputed PnP topic, so you can debug `aug_ekf` without finishing `tag_detector`.
- In `augekf.launch`, the full pipeline expects your own completed `tag_detector` implementation.
- In the small bag workflow, the absolute pose from PnP is already expressed in the world frame.

If you use your own Project 2 Phase 1 implementation in the full pipeline, be careful about frame transformation. In the original PnP problem, the pose obtained directly from solving PnP is the pose of the marker origin in the camera frame. The previous project used the following transforms:

$$
R_{wt} =
\begin{bmatrix}
0 & 1 & 0 \\
1 & 0 & 0 \\
0 & 0 & -1
\end{bmatrix}
$$

and

$$
T_{bcd} =
\begin{bmatrix}
1 & 0 & 0 & 0.07 \\
0 & -1 & 0 & -0.02 \\
0 & 0 & -1 & 0.01 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

For the full pipeline, make sure your own `tag_detector` implementation publishes a body pose that is consistent with the EKF state convention.

### Relative pose from stereo VO

The filter subscribes to:

- `/vo/Relative_pose`

The message type is:

- `stereo_vo/relative_pose`

This measurement represents the **pose of the current body frame relative to the body frame of the keyframe**. Your relative-pose measurement model should be consistent with the augmented keyframe copy stored in the filter state.

Important:

- In the release package, `stereo_vo` is an earlier assignment scaffold.
- In `augekf_simple.launch`, the bag already contains the precomputed VO topic, so you can debug `aug_ekf` without finishing `stereo_vo`.
- In `augekf.launch`, the full pipeline expects your own completed `stereo_vo` implementation.
- In the small bag workflow, the relative pose from VO is already expressed in the body frame of the keyframe.
- You may plot `/tag_detector/odom_ref` and `/vo/Odometry` to visualize the PnP and VO trajectories in the world frame.

## Keyframe Copying and Out-of-Order Measurements

This project requires two ideas beyond a standard EKF:

### 1. Keyframe copying

When stereo VO changes its keyframe, the copied keyframe pose inside the augmented state must also change. This means you need to:

- locate the state that corresponds to the VO keyframe timestamp
- update the copied keyframe part of the state
- update the associated covariance consistently

### 2. Out-of-order measurements

Measurements may not arrive in strictly increasing timestamp order. Your filter should therefore:

- keep a time-ordered history queue
- insert each new IMU / PnP / VO input into the correct position
- re-run prediction and update from the affected point onward

This queue logic is part of the required implementation in `processNewState`, `insertNewState`, and `repropagate`.

## Implementation Notes

Be careful about frame consistency:

- The relative pose measurement should be fused in the body-frame convention expected by the augmented state.
- The absolute pose measurement should be fused in the world-frame convention expected by the EKF.
- In the full pipeline, this consistency depends on your own Project 2 solutions.

The release scaffold is structured around the following processing flow:

1. `insertNewState`
2. `initFilter`
3. `repropagate`
4. `removeOldState`
5. `publishFusedOdom`

The callback functions feed this pipeline through `processNewState`. Following this structure will make the implementation much easier to debug.

## Running the Release Package

Place the required bag files in:

- `catkin_ws/src/aug_ekf/bag/`

The release package includes placeholder files in that directory. The bag files themselves are distributed separately.

### Mode 1: Simple Debug Mode

Use this mode first when you want to debug only the Augmented EKF with precomputed PnP and VO measurements.

Expected bag file:

- `imu_pnp_vo.bag`

Launch command:

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch aug_ekf augekf_simple.launch
```

This mode runs:

- the `aug_ekf` node
- rosbag playback of `imu_pnp_vo.bag`

This mode is intended to be independent of your `tag_detector` and `stereo_vo` implementations. The bag already contains the measurement topics needed by `aug_ekf`.

### Mode 2: Full Pipeline Mode

Use this mode when you want to run the complete pipeline using the bundled `tag_detector`, bundled `stereo_vo`, and your `aug_ekf`.

Expected bag file:

- `bag_aug_ekf.bag`

Launch command:

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch aug_ekf augekf.launch
```

This mode runs:

- the bundled `stereo_vo` node
- the bundled `tag_detector` node
- the `aug_ekf` node
- RViz
- rosbag playback of `bag_aug_ekf.bag`

Because the bundled `tag_detector` and `stereo_vo` packages are release scaffolds from earlier assignments, this full mode is expected to work only after you port your own earlier Project 2 solutions into those packages.

The large bag contains:

- IMU data
- images from a forward-looking stereo camera
- images from a downward-looking monocular camera

## Provided Code Structure

- `catkin_ws/src/aug_ekf/`
  - `src/ekf_filter.cpp`: student scaffold for filter logic and queue handling
  - `src/ekf_model.cpp`: student scaffold for process and measurement models
  - `include/ekf_filter.h`: filter interface, augmented state definition, and ROS members
  - `include/ekf_model.h`: matrix typedefs and function declarations
  - `launch/augekf_simple.launch`: simple debugging mode
  - `launch/augekf.launch`: full pipeline mode
  - `config/aug_ekf.rviz`: RViz configuration
  - `bag/`: bag placeholder directory
- `catkin_ws/src/tag_detector/`
  - Project 2 Phase 1 release scaffold for marker-based pose estimation
  - you should copy or port your own previous PnP solution here if you want the full pipeline to work
- `catkin_ws/src/stereo_vo/`
  - Project 2 Phase 2 release scaffold for stereo visual odometry
  - this package includes the extra `realsense_2` config used by `augekf.launch`
  - you should copy or port your own previous stereo VO solution here if you want the full pipeline to work

## Config Files and Launch Files

The full pipeline uses additional config files for `tag_detector` and `stereo_vo`.

- `augekf.launch` starts `stereo_vo` with `config/realsense_2/realsense_n3_unsync.yaml`
- `augekf.launch` starts `tag_detector` with `config/pt_grey.yml` and `config/a.yml`

You may modify `augekf.launch` if your own Project 2 solutions require different topic names, calibration files, or launch-time configuration.

## Evaluation

Your implementation will be evaluated on:

1. **Process-model correctness**
   - IMU prediction should be dimensionally correct and frame-consistent.
2. **Measurement-model correctness**
   - Absolute and relative pose updates should match the intended state definition.
3. **Augmented-state handling**
   - Keyframe copying and covariance handling should be consistent.
4. **Out-of-order handling**
   - The history queue and re-propagation logic should behave correctly.
5. **System behavior**
   - The fused odometry and path should behave reasonably in RViz.

## Submission

Submit your completed code together with a short report.

Your submission should include:

- all source files needed to run your `aug_ekf` solution
- the `tag_detector` and `stereo_vo` packages you used
- a report of at most 2 pages

The report should include:

- figures from RViz and/or `rqt_plot`
- a brief description of your implementation
- comments on keyframe handling, out-of-order handling, and filter tuning
- any failure cases, limitations, or observations worth noting

Do not include the bag files in your submission unless your instructor explicitly asks for them.

The submission title should be:

- `proj3phase2 YOUR-NAMES`

Please cite the paper, GitHub repository, or any other resources you referred to while completing this assignment. Please keep [academic integrity](https://registry.hkust.edu.hk/resource-library/academic-integrity), plagiarism is not tolerated in this course.

## Late Submission Policy

Late submissions are accepted up to 7 days after the due date, with 5% (of the total grade of the item) penalty per day.
