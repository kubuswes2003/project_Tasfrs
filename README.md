# TurtleBot ArUco Control

ROS2 package for controlling TurtleBot3 robot using ArUco marker detection.


## Description

This project controls a TurtleBot3 robot based on the position of an ArUco marker detected by a camera:
- Marker **above** image center → Robot moves **forward**
- Marker **below** image center → Robot moves **backward**
- Marker **centered** → Robot **stops**

## Requirements

- ROS2 Humble
- Ubuntu 22.04
- TurtleBot3 packages
- OpenCV with ArUco support
- USB camera or laptop webcam

## Installation

```bash
# Clone repository
cd ~/ros2_ws/src
git clone <your-repository-url> turtlebot_aruco_control

# Install dependencies
sudo apt update
sudo apt install -y python3-opencv python3-numpy
sudo apt install -y ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-usb-cam

# Build
cd ~/ros2_ws
colcon build --packages-select turtlebot_aruco_control --symlink-install
source install/setup.bash

# Set TurtleBot model
export TURTLEBOT3_MODEL=burger
```

## Usage

### Quick Start (Single Command)

```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot_aruco_control robot_control.launch.py
```

This launches:
- TurtleBot3 Gazebo simulation
- USB camera node
- ArUco controller node

### Prepare ArUco Marker

1. Generate marker at: https://chev.me/arucogen/
2. Settings: Dictionary = 4x4 (50), Marker ID = 0
3. Print or display on screen
4. Show to camera and move up/down to control robot

## Features

- ArUco marker detection (DICT_4X4_50, ID: 0)
- Real-time robot control based on marker position
- Visual feedback with debug window
- Single launch file for complete system

## Topics

**Subscribed:**
- `/image_raw` - Camera images

**Published:**
- `/cmd_vel` - Velocity commands

## Configuration

Edit `config/params.yaml` to customize:
- Linear speed
- Marker detection threshold
- Camera topic
- Debug mode

