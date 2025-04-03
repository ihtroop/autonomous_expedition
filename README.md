# Autonomous Expedition Task

## Overview
This repository contains the **Autonomous Expedition Stack** for navigating a simulated Martian environment. The stack includes modules for **arrow detection, rover positioning, obstacle detection, and a finite state machine (FSM) controller** to make real-time navigation decisions.

The system is designed to autonomously follow arrows, avoid obstacles and pits, and optimize rover traversal across rough terrain.

---

## System Flow
1. **Autonomous Stack Initialization**
2. **Arrow Detection** (via OpenCV-based detection and Pinhole camera model)
3. **Orientation and Positioning of the Rover** (using IMU & encoder-based odometry)
4. **Obstacle Detection & Avoidance** (via Intel RealSense point cloud processing)
5. **Controller (Finite State Machine - FSM)** (for navigation decision-making)

---

## Modules

### 1️⃣ Arrow Node
**Purpose:** Detect and track arrows to guide the rover along its path.
- Uses **Intel RealSense** and **Lenovo Camera** feeds.
- Implements **OpenCV-based algorithms** to detect arrows while filtering out false positives (ghost arrows).
- **Distance Estimation** via **Pinhole Camera Model** and camera calibration matrix.
- Applies **contour-based filtering** for arrow shape detection.
- **Arrow Tracking** using **Euclidean Distance Model**.
- **Frame Division & Centroid Tracking** for proper arrow alignment.
- **Cone Detection using YOLOv11** to trigger a final stop in the autonomous stack.

### 2️⃣ Orientation & Positioning of the Rover
**Purpose:** Maintain accurate localization using sensor fusion.
- Uses **Intel RealSense IMU** to obtain real-time orientation.
- Fuses **6-wheel rocker bogie encoder data** with IMU using an **Extended Kalman Filter (EKF)** for enhanced odometry.
- Outputs **filtered odometry data** for navigation and localization.

### 3️⃣ Obstacle & Pit Detection & Avoidance
**Purpose:** Identify and avoid obstacles and pits to ensure safe navigation.
- Uses **Intel RealSense Point Cloud Library (PCL)** for 3D mapping.
- Filters **point cloud data** to match the rover’s field of view.
- Implements **threshold-based detection** for obstacles and pits.
- Executes **path retracing and re-alignment** to continue towards the next arrow after obstacle avoidance.

### 4️⃣ Controller (Finite State Machine - FSM)
**Purpose:** Make real-time navigation decisions for efficient path planning.
- Takes inputs from:
  - **Arrow Detection Node**
  - **Orientation & Positioning Node**
  - **Obstacle & Pit Detection Node**
- Computes the optimal path to navigate the simulated Martian terrain.
- Handles deep pits and high hills with dynamic route adjustments.

---

## Dependencies
Ensure the following dependencies are installed before running the stack:

```bash
sudo apt update && sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-realsense2-camera \
    ros-humble-yolo \
    python3-opencv \
    python3-numpy \
    pcl-tools \
    ros-humble-robot-localization
```

---

## Setup & Installation
1. **Clone the repository**
   ```bash
   git clone https://github.com/your-repo/autonomous-expedition.git
   cd autonomous-expedition
   ```

2. **Build the ROS 2 workspace**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Launch the system**
   ```bash
   ros2 launch autonomous_stack expedition.launch.py
   ```

---

## Usage
- Start the **Arrow Detection Node**:
  ```bash
  ros2 run autonomous_stack input1a
  ```
- Start the **Orientation & Positioning Node**:
  ```bash
  ros2 run autonomous_stack input2
  ```
- Start the **Obstacle & Pit Detection Node**:
  ```bash
  ros2 run autonomous_stack obstacle_detection 
  ```
  ```bash
  ros2 run autonomous_stack pit_detection 
  ```
  ```bash
  ros2 run autonomous_stack avoidance
  ```
- Start the **Controller FSM**:
  ```bash
  ros2 run autonomous_stack controller
  ```

---


