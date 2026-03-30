# Project 2 Phase 2: Stereo Visual Odometry

Assignd: Mar 31, 2026

Due: Apr 17, 2026

## Overview

In this phase, you will implement a **stereo visual odometry** system for aerial robots. The system takes synchronized left-right images, tracks visual features across frames, estimates the relative camera motion, triangulates new 3D landmarks, and reconstructs a sparse map of the environment.

The goal is to estimate the camera trajectory and sparse 3D structure from a stereo image sequence, and use them for **6-DOF pose estimation** in ROS.

## Objectives

1. Understand the main components of a keyframe-based stereo visual odometry pipeline.
2. Implement feature tracking and stereo matching for establishing 2D-3D correspondences.
3. Estimate the relative pose between frames using PnP-based motion estimation.
4. Maintain a sparse 3D map and publish pose / point cloud results for visualization.

## Your Task

Complete the missing parts in `catkin_ws/src/stereo_vo/stereo_vo_estimator/src/estimator.cpp`.

You need to implement the following functions:

1. `inputImage`
   - Run the full stereo VO pipeline for each incoming stereo pair.
   - Handle initialization, keyframe updates, and current-frame state propagation.

2. `trackFeatureBetweenFrames`
   - Track 2D features from the key frame to the current left image.
   - Keep the matched 3D points from the key frame and the tracked 2D points in the current frame.

3. `estimateTBetweenFrames`
   - Estimate the relative rotation `R` and translation `t` between the key frame and the current frame from 2D-3D correspondences.
   - Reject bad correspondences and keep the pose estimate stable.

4. `extractNewFeatures`
   - Detect new feature points in the current left image to maintain enough tracked landmarks.

5. `trackFeatureLeftRight`
   - Match the newly extracted left-image features to the right image.
   - Keep only valid stereo correspondences for triangulation.

6. `updateLatestStates`
   - Update the published current pose, relative pose to the key frame, and the latest point cloud.

### Intended Pipeline

1. Track left-image features from the key frame to the current frame.
2. Use the matched 2D-3D correspondences to estimate the relative pose.
3. Extract new features in the current left image.
4. Match those features between the left and right images.
5. Triangulate new 3D points from the stereo correspondences.
6. Update the current world pose and relative pose.
7. Decide whether to switch the current frame to a new key frame.

## Running the Simulation

1. **Download the rosbag** into `catkin_ws/src/stereo_vo/stereo_vo_estimator/bag/`:
   ```bash
   cd catkin_ws/src/stereo_vo/stereo_vo_estimator/bag
   wget https://repo.hkust-uav.org/artifactory/generic-artifactory/rosbag/elec5660/realsense_1.bag
   ```

2. **Build the workspace**:
   ```bash
   cd catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Launch the stereo VO system**:
   ```bash
   roslaunch stereo_vo stereo_vo_bag.launch
   ```

4. **Visualization**:
   - RViz will display the estimated odometry / path.
   - RViz will display the current camera pose.
   - RViz will display the sparse point cloud reconstructed by your stereo VO pipeline.

## Evaluation Criteria

Your implementation will be evaluated on:

1. **Feature Tracking Robustness**: The system should track enough valid features across frames for stable estimation.
2. **Pose Estimation Correctness**: The estimated relative motion should be geometrically reasonable and consistent over time.
3. **Map / Trajectory Quality**: The reconstructed sparse point cloud and estimated path should be reasonable in RViz.
4. **System Stability**: The pipeline should handle keyframe updates and normal frame-to-frame motion without frequent failure or divergence.

## Provided Code Structure

  - `catkin_ws/src/stereo_vo/stereo_vo_estimator/`: ROS package for stereo visual odometry
  - `src/estimator.cpp`: **Your implementation goes here**
  - `include/estimator.h`: Estimator class, frame data structure, and internal state variables
  - `src/stereo_vo_node.cpp`: Main ROS node for subscribing to stereo images and publishing results
  - `launch/stereo_vo_bag.launch`: Launch file to run the estimator, RViz, and rosbag playback
  - `config/realsense_1/realsense_n3_unsync.yaml`: Topics, stereo extrinsics, and tracking thresholds
  - `config/realsense_1/left.yaml`: Left camera calibration
  - `config/realsense_1/right.yaml`: Right camera calibration
  - `bag/`: Directory for storing the rosbag file `realsense_1.bag`

## Submission

Submit your completed code along with a brief report.

Code should include all necessary files to run your stereo visual odometry system in simulation.

The report should include the following sections with max 2 pages:

- Figures from RViz showing estimated trajectory, camera pose, and sparse point cloud.
- Statistics about your implementation. (For example, tracked feature count, number of successful pose estimates, or any other quantitative metrics you use.)
- Analysis of your results. (For example, drift behavior, failure cases, or parameter studies.)
- Any other things we should be aware of.

Please submit a single zip file named `proj2phase2_yourname.zip` to the canvas.

Please cite the paper, GitHub repository, or any other resources you referred to while completing this assignment. Please keep [academic integrity](https://registry.hkust.edu.hk/resource-library/academic-integrity), plagiarism is not tolerated in this course.

## Late Submission Policy

Late submissions are accepted up to 7 days after the due date, with 5% (of the total grade of the item) penalty per day.
