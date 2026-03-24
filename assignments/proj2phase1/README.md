# Project 2 Phase 1: Tag Detection and Pose Estimation

Assigned: Mar 24, 2026

Due: April 3, 2026

## Overview

In this phase, you will work on a **vision-based state estimation** system for aerial robots. Specifically, you will implement the **Perspective-n-Point (PnP)** algorithm using the **Direct Linear Transformation (DLT)** method to estimate the camera's pose (rotation matrix $R$ and translation vector $T$) relative to a known ARuco tag marker. This enables a drone to localize itself using visual markers.

## Objectives

1. Understand the camera projection model and the PnP problem.
2. Implement the **Direct Linear Transformation (DLT)** algorithm for PnP using SVD.
3. Estimate the camera pose $[R|t]$ given 3D points in the world frame and their corresponding 2D projections in the image frame.
4. Enforce **SO(3)** rotation constraints and recover the scale of the translation vector.

## Your Task

Complete the function `solvePnP` in `catkin_ws/src/tag_detector/src/pnp.hpp`.

### Function Signature

```cpp
void solvePnP(
    const std::vector<cv::Point3f> &pts_3,    // 3D world points
    const std::vector<cv::Point2f> &pts_2,    // 2D image points
    const cv::Mat &K,                         // Camera intrinsic matrix
    Eigen::Matrix3d &R,                       // Output: rotation matrix
    Eigen::Vector3d &T                        // Output: translation vector
);
```

### Input

- `pts_3`: A vector of 3D points $(X, Y, Z)$ in the world coordinate system (tag frame).
- `pts_2`: A vector of 2D points $(u, v)$ in the image pixel coordinate system.
- `K`: The $3 \times 3$ camera intrinsic matrix.

### Output

- `R`: The estimated $3 \times 3$ rotation matrix (from world to camera frame).
- `T`: The estimated $3 \times 1$ translation vector (from world to camera frame).

### Implementation Steps

1. **Construct the linear system**: Build the matrix $A$ for $A \mathbf{h} = 0$, where $\mathbf{h}$ contains the elements of the projection matrix.
2. **Solve using SVD**: Solve for $\mathbf{h}$ via Singular Value Decomposition of $A$.
3. **Extract $R$ and $T$**: Recover the rotation matrix and translation vector from the projection matrix.
4. **Enforce SO(3) constraint**: Apply SVD-based correction to ensure $R$ is a valid rotation matrix ($R^T R = I$, $\det(R) = 1$).
5. **Recover scale**: Scale $T$ appropriately to match the physical dimensions of the tag.

## Running the Simulation

1. **Build the workspace**:
   ```bash
   cd catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. **Launch the node**:
   ```bash
   roslaunch tag_detector bag_tag.launch
   ```
   This will play the provided rosbag and run the tag detector.

3. **Visualization**:
   - RViz will display the estimated camera pose.
   - The terminal will print the **Reference Pose** (from OpenCV's `solvePnP`).
   - The **RMSE** (Root Mean Square Error) of the reprojection will also be displayed.

## Evaluation Criteria

Your implementation will be evaluated on:

1. **RMSE Accuracy**: Lower reprojection error indicates a correct implementation.
2. **Pose Correctness**: The estimated $R$ must be a valid rotation matrix (orthogonal, $\det(R) = 1$).
3. **Scale Correctness**: The translation vector $T$ must have the correct physical scale relative to the tag size.

## Provided Code Structure

- `catkin_ws/src/tag_detector/`: The ROS package for tag detection
  - `src/tag_detector_node.cpp`: Main ROS node — handles image callbacks, ARuco detection, RMSE evaluation, and publishes odometry
  - `src/pnp.hpp`: **Your implementation goes here**
  - `launch/bag_tag.launch`: Launch file to run rosbag + detector + rviz
  - `config/a.yml`: Marker board configuration (12×6 grid, 144 markers)
  - `config/camera.yml`: Camera intrinsic parameters
  - `bag/`: Contains the test rosbag file (download link)

## Submission

Submit your completed code along with a brief report.

Code should include all necessary files to run the tag detector in simulation.

The report should include the following sections with max 2 pages:

- Figures from RViz showing pose estimation results.
- Statistics about your implementation (e.g., RMSE across frames).
- Analysis of your results (e.g., accuracy, failure cases).
- Any other things we should be aware of.

Please submit a single zip file named `proj2phase1_yourname.zip` to the canvas.

Please cite the paper, GitHub repository, or any other resources you referred to while completing this assignment. Please keep [academic integrity](https://registry.hkust.edu.hk/resource-library/academic-integrity), plagiarism is not tolerated in this course.

## Late Submission Policy

Late submissions are accepted up to 7 days after the due date, with 5% (of the total grade of the item) penalty per day.
