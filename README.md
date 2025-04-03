# YuMi_Pick_and_Place
# 🤖 YuMi ROS Workspace

This workspace includes a set of ROS packages for simulating, visualizing, and planning motion for the **ABB YuMi (IRB 14000)** collaborative robot. It is designed for **ROS Noetic** on **Ubuntu 20.04**.

## 📦 Package Overview

### 1. `yumi_description`
Contains URDF models, STL 3D meshes, and RViz configuration files for the YuMi robot. The `meshes/` folder includes both `coarse` and `fuller` geometry versions.

### 2. `yumi_moveit_config`
Provides the **MoveIt! configuration** for YuMi, including SRDF, planning settings, controllers, and demo launch files. Use this for motion planning with MoveIt.

### 3. `yumi_scene`
Includes simulation scenes with environment models (e.g., table) and RViz or Gazebo launch files. Useful for testing in realistic environments.

### 4. `yumi_mg_py`
A Python-based motion generation package. It may include:
- Scripts for joint control
- Trajectory generation
- Integration with MoveIt! or RViz

## Dependencies

- ROS Noetic
- MoveIt (`ros-noetic-moveit`)
- RViz (`ros-noetic-rviz`)
- Other required packages:
  - `robot_state_publisher`
  - `joint_state_publisher`
  - `xacro`
  - `controller_manager`

Install them with:

```bash
sudo apt update
sudo apt install ros-noetic-moveit ros-noetic-joint-state-publisher \
ros-noetic-robot-state-publisher ros-noetic-xacro

## Installation

Clone the packages or unzip them in your ROS workspace:
cd ~/catkin_ws/src
# Replace <repo-url> if needed
unzip src.zip in an empty folder
cd "empty_folder_with_src_unzipped"
catkin build
source devel/setup.bash

## Launch MoveIt! demo for motion planning
roslaunch yumi_moveit_config demo.launch
## Execute Planned Pick and Place Trajectory
roslaunch yumi_moveit_config demo.launch
rosrun yumi_mg_py yumi_traj_py.py
