# Crazyflie OptiTrack ROS 2 Package

## Overview

This ROS 2 package (`crazyflie_pose_follower`) enables autonomous waypoint navigation and dynamic landing for a Crazyflie drone using motion capture data from an OptiTrack (NatNet) system.

It integrates with `cflib` to control the Crazyflie via high-level commands and subscribes to robot pose data from ROS 2 topics to guide flight and execute precision landing sequences.

---

## ROS 2 Topics

| Topic Name          | Message Type         | Description                                  |
|---------------------|----------------------|----------------------------------------------|
| `/crazyflie/takeoff`| `std_msgs/String`     | Starts the waypoint flight sequence          |
| `/Robot_1/pose`     | `geometry_msgs/PoseStamped` | Crazyflie's pose via NatNet or other MoCap |
| `/Robot_2/pose`     | `geometry_msgs/PoseStamped` | Landing target's pose (e.g., TurtleBot)     |

---

## Directory Structure
```bash
crazyflie_pose_follower/
├── launch/
│ └── pose_follower.launch.py
├── crazyflie_pose_follower/
│ ├── init.py
│ └── pose_follower_node.py
├── resource/
│ └── crazyflie_pose_follower
├── setup.py
└── package.xml
---
```

## Dependencies

- ROS 2 Humble 
- `cflib` (Crazyflie Python Library)
- Crazyflie firmware (with high-level commander support)
- OptiTrack/NatNet pose publishing to ROS 2 (`/Robot_1/pose`, `/Robot_2/pose`) https://github.com/ros-drivers/mocap_optitrack
You need to make few modifcations to make it work for NATNet Version : 4.1.0.0
and Server Version : 3.0 later versions.

---

## Launch Instructions

```bash
git clone https://github.com/MAHASHANA/crazyflie_optitrack.git
cd crazyflie_optitrack
# Build the repo 
colcon build
# Source your ROS 2 workspace
source install/setup.bash
source /opt/ros/humble/setup.bash

# Launch the Crazyflie pose follower node
ros2 run crazyflie_pose_follower pose_follower_node 
