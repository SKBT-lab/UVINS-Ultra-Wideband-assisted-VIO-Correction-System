# UVINS: An UWB-assisted VIO Correction System

![License](https://img.shields.io/github/license/SKBT-lab/UVINS-Ultra-Wideband-assisted-VIO-Correction-System) 

## Overview

This project achieves the fusion of UWB and VIO, with VIO based on VINS_Fusion. The goal is to leverage UWB's drift-free measurements to eliminate the accumulated error of VIO, thereby achieving drift correction superior to visual loop closure.

- **Highlight 1**: The adopted method integrates the two mainstream sensors fusion approaches: filtering and optimization.
- **Highlight 2**: Dataset for UWB-Vision-Inertial is provided.

### Experimental Setup
![experimental setup](https://github.com/SKBT-lab/UVINS-Ultra-Wideband-assisted-VIO-Correction-System/blob/master/support_files/experimental_setup.jpeg)
## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Installation
This project is built and run based on the ROS system.
```bash
# Creat ROS workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
```

```bash
# Clone the repository
cd ~/catkin_ws/src
git clone https://github.com/SKBT-lab/UVINS-Ultra-Wideband-assisted-VIO-Correction-System.git
```
```bash
# build
cd UVINS-Ultra-Wideband-assisted-VIO-Correction-System
catkin_make --only-pkg-with-deps uwb
sudo rm -r build
catkin_make
```
## Usage
### Parameters Setting
Besides parameters in VINS-Fusion, some other parameters also need to be set.
- "uwb_topic": in path_to_package/config/SKBT_280.yaml
- The coordinates of the UWB anchors: in path_to_package/config/uwb.yaml
- The extrinsic parameters between UWB tag and IMU: in path_to_package/config/uwb.yaml
### Run ROSnode
```bash
roslaunch uvins ros_test.launch
```
### Play Rosbag
The UWB-Vision-Inertial dataset published by this project is provided in the form of ROSbag.
```bash
rosbag play path_to_dataset/my_data.bag
```
### Run visualization plugin
```bash
roslaunch uwb vis.launch
```

## Dataset
http://......

## Paper
Paper is on the way. Stay tuned!

## Vedio
Stay tuned