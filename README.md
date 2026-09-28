# TurtleBot ArUco Control

A ROS 2 node that watches an ArUco marker through a webcam and drives a TurtleBot3 forward or backward depending on where the marker sits in the image.

## Context

Small individual course lab for *Tools and Software for Robotic Systems* (Poznań University of Technology), ROS 2 Humble on Ubuntu 22.04.

The setup is deliberately mixed: the robot is **simulated in Gazebo**, while the camera is a **real USB/laptop webcam**. You hold a printed or on-screen marker in front of your own camera and the simulated robot reacts.

## How it works

The node subscribes to the camera image, detects markers with OpenCV's ArUco module and looks for one specific ID. It compares the marker's centre with the vertical centre of the image and publishes a `geometry_msgs/Twist`:

| Marker position | Command |
|---|---|
| Above the image centre, by more than the threshold | forward, `linear.x = +linear_speed` |
| Below the image centre, by more than the threshold | backward, `linear.x = -linear_speed` |
| Within the threshold band | stop |

Only the **vertical** offset is used, and only **linear** velocity is driven — `angular_speed` exists as a parameter but defaults to `0.0`, so the robot does not steer. A debug window draws the detected marker, the centre line and the current decision. On shutdown the node publishes a zero `Twist` so the robot does not keep rolling.

```
webcam ──/image_raw──► aruco_controller ──/cmd_vel──► TurtleBot3 (Gazebo)
```

## Configuration

`config/params.yaml`:

| Parameter | Default | Meaning |
|---|---|---|
| `aruco_dict` | `DICT_4X4_50` | ArUco dictionary |
| `marker_id` | `0` | which marker the node reacts to |
| `linear_speed` | `0.2` | m/s, forward and backward |
| `angular_speed` | `0.0` | published as-is; no steering logic |
| `threshold` | `20` | dead band in pixels around the image centre |
| `debug` | `true` | show the detection window |
| `camera_topic` | `/image_raw` | image source |
| `cmd_vel_topic` | `/cmd_vel` | velocity output |

## Running it

```bash
cd ~/ros2_ws/src
git clone https://github.com/kubuswes2003/turtlebot-aruco-control.git
cd ~/ros2_ws

sudo apt install -y python3-opencv python3-numpy \
    ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs ros-humble-usb-cam

colcon build --packages-select turtlebot_aruco_control --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot_aruco_control robot_control.launch.py
```

This starts Gazebo (empty world), the `usb_cam` node on `/dev/video0` and the controller. Generate a marker with [chev.me/arucogen](https://chev.me/arucogen/) — dictionary 4×4 (50), ID 0 — then show it to the camera and move it up and down.

Launch arguments: `use_sim` (default `true`), `use_camera` (`true`), `world` (`empty_world`), `debug` (`true`).

## Limitations and what I would improve

- **Image-plane control only.** The node uses the marker's pixel position, not its pose. With `cv2.aruco.estimatePoseSingleMarkers` and a calibrated camera it could hold a real distance instead of a pixel offset.
- **No steering.** Horizontal marker offset is ignored, so the robot cannot turn towards the marker.
- **Bang-bang control.** Speed is constant outside the dead band; a proportional law would stop the visible jerkiness.
- **Not run on a physical TurtleBot.** `use_sim:=false` only skips launching Gazebo; nothing here brings up a real robot, and the real-hardware path is untested.
- **Legacy OpenCV API.** `cv2.aruco.Dictionary_get` / `DetectorParameters_create` were removed in OpenCV 4.7; the node works with the 4.5.x that ships with Ubuntu 22.04 but needs `ArucoDetector` on newer versions.
- No tests, and no handling for the marker leaving the frame beyond stopping.
