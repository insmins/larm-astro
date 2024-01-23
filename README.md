# Astro Group's repository for the LARM UV


## Authors
Inès El Hadri  
Lucas Naury

--- 
## Introduction
This repository is a ROS2 package that allows the control of a kobuki robot. 

### Table of Contents :

1. [Authors](#authors)
1. [Introduction](#introduction)
1. [How it works](#how-it-works)
    1. [Goal](#goal)
    1. [Expected behaviour](#expected-behaviour)
    1. [Additional functionality](#additional-functionality)
1. [Installation](#installation)
    1. [Requirements](#requirements)
    1. [Install the package](#install-the-package)
    1. [Tune the camera HSV](#tune-the-camera-hsv)
    1. [Build the package](#build-the-packages)
1. [How to use the package](#how-to-use-the-package)
    1. [In simulation](#in-simulation)
    1. [On the bot](#on-the-bot)
    1. [Visualization](#visualization)
1. [Frequently Asked Questions](#faq)




---
## How it works

### Goal

The goal is to explore a closed area (i.e. an area bounded with obstacles) with the robot while avoiding obstacles. While doing so, we must determine the number and position of green painted bottles.

### Expected behaviour

- The robot moves **continuously** in the area by **avoiding obstacles**
    - If there are no obstacles, it moves straight
    - If an obstacle is in front, it turns the opposite direction until there's no obstacle left in front. 
- Using the **SLAM algorithm** with data from the LiDAR and the odometer, the robot builds a map and localizes itself in it.
- Using a RealSense RGBD camera (D435i), the robot is able to **detect the green bottles**. Messages are sent in topics:
    - `detection` : to state the detection
    - `bottle_relative_pos` : to tell the position of the bottle relative to the camera
    - `bottle_marker` : to mark the position of a bottle on the map
- Experiments can be performed with **2 computers**:
    - one on the robot (Control PC) running all the robot, movement and vision nodes
    - a second for visualization and human control (Operator PC)

### Additional functionality

- An [**automatic HSV tuner script**](#tune-the-camera-hsv) allows you to calculate the ideal threshold to mask your bottle
- Most configuration variables (robot speeds, bounding boxes, are set as **ROS parameters** so that they can be modified
- The robot stops moving when you press any of the **3 robot buttons**. If you press it again, movement will continue.
> All other data is still being processed when the robot is in pause
- The robot **stops when you lift it** (i.e. the wheels are "falling"). It you put the robot back on the ground, movement will continue.

In this branch, you'll also find a node that **automates the map discovery** (by sending goal poses to unknown areas for the robot to go to).
> This has only been tested on the simulator, since we didn't have time to test it on the real robot

## Installation
### Requirements
Before starting, please ensure you have installed the following
- ROS2 Iron : https://docs.ros.org/en/iron/index.html
- ROS2 packages : `apt install ros-iron-slam-toolbox ros-iron-nav2-bringup`
- Python 3.10 : https://www.python.org/downloads/
- Python packages :
    * math, os, signal, sys, time (installed with python)
    * numpy
    * colcon-common-extensions
    * opencv-python
    * pyrealsense2
    * cvbridge3
    * scikit-image
</br>

> Command :  
> `pip install numpy colcon-common-extensions opencv-python pyrealsense2 cvbridge3 scikit-image`

- $`\textcolor{red}{\text{[OPTIONAL]}}`$ Gazebo (for the simulation) (`apt install ros-iron-gazebo-ros-pkgs`)
-  $`\textcolor{red}{\text{[OPTIONAL]}}`$ Teleop twist keyboard (to control manually the robot)

### Configuration
Add the following lines to your `~/.bashrc` :
```
source /opt/ros/iron/setup.bash #Add ROS2 source
export ROS_DOMAIN_ID=12 #Setup domain ID
```
If you want your ROS nodes to only be accessible on localhost, add :
```
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST #Tell ROS to make your nodes only accessible by the same machine
```
However, if you want to be able to visualize data from another computer on the same network, add :

```
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET #Tell ROS to make your nodes accessible by machines on the same network
```


### Install the package
1. Open the command prompt in the ROS2 workspace directory
1. Clone the IMT Tbot packages : `git clone https://bitbucket.org/imt-mobisyst/pkg-tbot.git` and `git clone https://bitbucket.org/imt-mobisyst/pkg-tsim.git`
1. Clone this repository : `git clone http://gvipers.imt-nord-europe.fr/ines.el.hadri/larm.git`


### Tune the camera HSV

To tune the HSV threshold parameters for the camera mask, we will use a script to automate it.

First, put a bottle in front of the robot.

Then, go in the `larm` directory and launch the HSV tuner python script using the following command
```
python3 src/config/mask_tuner.py
```

A window will popup with a frame from the RealSense camera :
- Make a rectangle selection of the inside of the bottle (every pixel in the rectangle should be green, make the biggest rectangle possible to have a lot of diversity in the green colors).
- Then press enter to save the selection and calculate the HSV thresholds
- If the mask seems good enough, press a key. Otherwise, press a key and restart the script

> Every time you run the tuner script, you will have to rebuild the package (see [Build the package](#build-the-packages))

### Build the packages
In the same ROS2 workspace directory:
- `./pkg-tbot/bin/install`
- `colcon build`
- `source install/setup.sh`

This will build all the packages in the folder.

---
## How to use the package
First, go in the ROS2 workspace.

### In simulation
To launch the challenge 1 in **simulation**, run the following command :
`ros2 launch grp_astro simulation_launch.yaml`

If you want to launch the **automatic discovery nodes**, run the following command instead :
`ros2 launch grp_astro auto_simulation_launch.yaml`

### On the tbot
To launch the challenge 1 on the **real turtlebot**, run the following command :
`ros2 launch grp_astro tbot_launch.yaml`

### Visualization  
In parallel, if you want to **visualize** the information that is published on the different topics, you can run :
- `ros2 launch tbot_operator_launch.yaml` for the real robot
- `ros2 launch sim_operator_launch.yaml`  in simulation

> If you want to run this visualization on another computer than the one running the robot, make sure :
> - they are on the **same network**
> - they have the same **`ROS_DOMAIN_ID`** environment variable
> - they have configured **`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`** in the environment variables



If you want to launch the **automatic discovery visualizer** (path found, goal poses...), run the following commands instead :
- `ros2 launch auto_sim_operator_launch.yaml`  in simulation

---
## FAQ
