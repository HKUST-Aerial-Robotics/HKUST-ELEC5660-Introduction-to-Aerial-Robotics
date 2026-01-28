<img src="https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/picgo/Gemini_Generated_Image_brfxjrbrfxjrbrfx.png" alt="Gemini_Generated_Image_brfxjrbrfxjrbrfx" style="zoom: 25%" />

## HKUST ELEC5660: Introduction to Aerial Robotics

ELEC5660 is an HKUST PG course which gives a comprehensive introduction to aerial robots. The goal of this course is to expose students to relevant mathematical foundations and algorithms and train them to develop real-time software modules for aerial robotic systems. Topics to be covered include rigid-body dynamics, system modeling, control, trajectory planning, sensor fusion, and vision-based state estimation. Students will complete a series of projects that combine into an aerial robot that is capable of vision-based autonomous indoor navigation.

**Instructor:** [Shaojie SHEN](https://ece.hkust.edu.hk/eeshaojie)

**TAs:** [Yang Xu](https://jason-xy.cn/self/) (yxuew@connect.ust.hk), Pusen Gao (pgaoak@connect.ust.hk)

**Lab:** [HKUST Aerial Robotics Group](https://uav.ust.hk)

---

### File structure

* course_node: notes
* assignment: assignment code
* lab: lab notes
* workspace_template: Terraform template for setting up cloud workspace

### Brief description for assignments and labs

| Name | Description | Demo |
|------|-------------|------|
| proj1phase1 | Implement a basic controller for quadrotor trajectory tracking | ![p1p1](fig/p1p1.gif) |
| proj1phase2 | Implement trajectory generation with minimum jerk/snap optimization | ![p1p2](fig/p1p2.gif) |
| proj1phase3 | Implement A* path planning and intergrate with trajectory generation and control | ![p1p3](fig/p1p3.gif) |
| lab1 | Assemble and fly a drone in manual mode and prepare hardware & software environment for later labs | ![IMG_6944](fig/IMG_6944.jpg) |
| proj1phase4_lab2 | Fly the drone in autonomous control mode with OptiTrack motion capture system and analyze flight data | ![lab2](fig/lab2.gif)|
| proj2phase1 | Implement Perspective-n-Point (PnP) algorithm for vision-based state estimation | ![pnp](fig/pnp.gif) |
| proj2phase2 | Implement stereo visual odometry for 6-DOF pose estimation | ![stereo_vo](fig/stereo_vo.gif) |
| proj3phase1 | Implement an Extended Kalman Filter (EKF) for sensor fusion of IMU and visual odometry | ![p3p1](fig/p3p1.gif) |
| proj3phase2 | Implement an augmented EKF for sensor fusion of IMU, visual odometry, and tag-based pose estimation | ![p3p2](fig/p3p2.gif) |
| proj3phase3_lab3 |  Integrate the whole system onboard for tracking trajectory or autonomous flight without OptTrack motion capture system | ![p3p3](fig/p3p3.gif) |

### Contact

For questions and suggestions, please contact TAs