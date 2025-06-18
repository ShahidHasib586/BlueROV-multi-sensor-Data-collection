# BlueROV Autonomous Control System

![ROS 2](https://img.shields.io/badge/ROS-2-%230A0FF9) ![License](https://img.shields.io/badge/License-Apache%202.0-blue)

ROS 2 package for autonomous control of BlueROV underwater vehicles, featuring MAVROS integration, real-time sensor processing, and gamepad teleoperation.

## Table of Contents
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
  - [Parallel Execution](#parallel-execution-recommended)
  - [Individual Components](#individual-components)
- [Package Structure](#package-structure)
- [🎥How to Record and Play a ROS 2 Bag from Camera](#how-to-record-and-play-a-ros-2-bag-from-camera)


## Features
- **MAVROS Bridge**: Robust communication with Pixhawk flight controller
- **Sensor Fusion**: Real-time processing of IMU, depth, and sonar data
- **Gamepad Control**: Low-latency teleoperation via Xbox/PS4 controller
- **Video Pipeline**: HD video streaming with OpenCV processing
- **Parallel Execution**: Optimized multi-core performance using GNU Parallel

## Installation

### Prerequisites
- Ubuntu 22.04 (Recommended)
- ROS 2 Humble Hawksbill
- Python 3.8+

```bash
# Install GNU Parallel
sudo apt update && sudo apt install parallel

# Verify installation
parallel --version
```
## Build Instructions

```bash
# Create workspace
mkdir -p ~/bluerov_ws/src
cd ~/bluerov_ws/src
```
# Clone repository
```bash
git clone https://github.com/shahidhasib586/bluerov_ws.git
```

# Install dependencies
```bash
cd ~/bluerov_ws
rosdep install --from-paths src --ignore-src -r -y
```

# Build package
```bash
colcon build --packages-select autonomous_rov
source install/setup.bash
```
## Usage

### Parallel Execution (Recommended)

Launch all nodes simultaneously with automatic error handling:

```bash
cd ~/bluerov_ws
source install/setup.bash

parallel --lb --halt now,fail=1 ::: \
  "ros2 launch autonomous_rov run_mavros.launch" \
  "ros2 launch autonomous_rov run_listener.launch" \
  "ros2 launch autonomous_rov run_gamepad.launch" \
  "ros2 run autonomous_rov video"
```
## Command Breakdown

| Option | Description |
|--------|-------------|
| `--lb` | Line-buffered output for clean logs |
| `--halt now,fail=1` | Automatic shutdown if any node fails |
| `:::` | Separator for parallel commands |

## Individual Components

```bash
# MAVROS Interface
ros2 launch autonomous_rov run_mavros.launch.py

# Sensor Processing
ros2 launch autonomous_rov run_listener.launch.py

# Gamepad Control
ros2 launch autonomous_rov run_gamepad.launch.py

# Video Processing
ros2 run autonomous_rov video --ros-args -r __node:=video_node
```
## Package Structure
```
bluerov_ws/
├── autonomous_rov/
│ ├── launch/ # Launch configurations
│ │ ├── run_mavros.launch.py
│ │ ├── run_listener.launch.py
│ │ └── ...
│ ├── config/ # YAML configuration files
│ │ ├── mavros_params.yaml
│ │ └── gamepad_mappings.yaml
│ ├── autonomous_rov/ # Python source
│ │ ├── init.py
│ │ ├── video.py # OpenCV processing
│ │ ├── listener.py # Sensor fusion
│ │ └── ...
│ ├── package.xml # ROS 2 package metadata
│ └── setup.py # Python build config
└── ...
```
# 🎥 How to Record and Play a ROS 2 Bag from Camera

## 🔍 Step 1: Find the Actual Image Topic

First, list available image-related topics:
```bash
ros2 topic list | grep image
```
Look for something like /camera/image — this is usually the actual image stream.

To verify that it publishes camera data, run:
```bash
ros2 topic info /camera/image
```
You should see:

Type: sensor_msgs/msg/Image

## 📀 Step 2: Record the Image Topic to a Bag

To start recording:
```bash
ros2 bag record -o my_camera_bag /camera/image
```
Replace my_camera_bag with your desired folder name.

Press Ctrl+C to stop recording.

## ▶️ Step 3: Play the Bag

To play back the recorded video:
```bash
ros2 bag play my_camera_bag --loop
```
The --loop option will continuously replay the bag.

Remove --loop if you only want to play it once.

## 🖼 Step 4: View the Camera Video

In a separate terminal, run:
```bash
ros2 run rqt_image_view rqt_image_view
```
Then select /camera/image from the topic list to view the video stream.

## Want to test your laptop camera as well?

### install the following
```bash
sudo apt install ros-jazzy-v4l2-camera # my distro is jazzy, replace the name with your distro (ex: humble)
```
## Than run the following command:

```bash
ros2 run v4l2_camera v4l2_camera_node
```
## On a seperate terminal run:

```bash
ros2 run rqt_image_view rqt_image_view
```

## Save both camera data:

# run the following to confirm the topic

```bash
ros2 topic list | grep image
```
## save the recordng in a folder
```bash
ros2 bag record -o camera_bag /image_raw/compressed /camera/image
```
## See the replay:

## remap and play

```bash
ros2 bag play camera_bag   --remap /image_raw/compressed:=/playback/image_raw/compressed           /camera/image:=/playback/camera/image --loop
```
## open viwer
```bash
ros2 run rqt_image_view rqt_image_view
```

## Install usb camera in blueROV

login to the blueROV terminal

```bash
ssh pi@192.168.2.2
pass: raspberry
```
#### Then check if the webcam is detected:
```bash
ls /dev/video*
dmesg | grep -i usb
lsusb
```
### If you see /dev/video4 (or similar), the webcam is recognized.

Verify UDP Streams Are Live

Make sure both streams are actively sending data:

    BlueROV on port 5600
    USB camera on port 5602

Use this to check traffic:
```bash
sudo tcpdump -i any port 5600
```
### To run both camera:
```bash
ros2 run autonomous_rov multivideo
```
## Record Bag File from Both Topics

Run this in a terminal:
```bash
ros2 bag record /bluerov/image /usb_cam/image -o multivideo_bag
```
  ros2 bag record: the command to start recording

  /bluerov/image /usb_cam/image: the image topics

  -o multivideo_bag: name of the output bag directory



















