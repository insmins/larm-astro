# group Astro, repository for the UV LARM
---
## Authors :
Inès El Hadri
Lucas Naury

---
## Introduction
This repository is a ROS2 package that allows the control of a kobuki robot. 
### Table of Contents :

1. [Authors](#authors)
1. [Introduction](#introduction)
1. [Installation](#installation)
    1. [Requirements](#requirements)
    1. [Install the package](#install-the-package)
    1. [Build the package](#build-the-packages)
1. [How to use the package](#how-to-use-the-package)
    1. [In simulation](#in-simulation)
    1. [On the bot](#on-the-bot)
    1. [Visualization](#visualization)
1. [Frequently Asked Questions](#faq)

---
## Installation
### Requirements
Before starting, please ensure you have installed the following
- ROS2 Iron : https://docs.ros.org/en/iron/index.html
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
    >Command :  
    >`pip install numpy colcon-common-extensions opencv-python pyrealsense2 cvbridge3 scikit-image`

- $`\textcolor{red}{\text{[OPTIONAL]}}`$ Gazebo (for the simulation)
-  $`\textcolor{red}{\text{[OPTIONAL]}}`$ Teleop twist keyboard (to control manually the robot)

### Install the package
1. Open the command prompt in the ROS2 workspace directory
1. Clone the IMT Tbot packages : `git clone https://bitbucket.org/imt-mobisyst/pkg-tbot/src/master/`
1. Clone this repository : `git clone http://gvipers.imt-nord-europe.fr/ines.el.hadri/larm.git`


### Build the packages
In the same ROS2 workspace directory:
- `./master/bin/install`
- `colcon build`
- `source install/setup.sh`

This will build all the packages in the folder.

---
## How to use the package
First, go in the ROS2 workspace.

### In simulation
To launch the challenge 1 in **simulation**, run the following command :
`ros2 launch grp_astro simulation_launch.yaml`

### On the tbot
To launch the challenge 1 on the **real turtlebot**, run the following command :
`ros2 launch grp_astro tbot_launch.yaml`

### Visualization  
In parallel, if you want to **visualize** the information that is published on the different topics, you can run
`ros2 launch grp_astro visualize.launch.py`

> If you want to run this visualization on another computer than the one running the robot, make sure :
> - they are on the **same network**
> - they have the same **`ROS_DOMAIN_ID`** environment variable
> - they have configured **`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`** in the environment variables

---
## FAQ
