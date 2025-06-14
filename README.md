# Embedded ROS 2 Architecture for Bi-Manual Robotic Motion Planning
This repository includes Bi-Manual robot control with motion planning using ROS 2 Jazzy and MoveIt 2 for collaborative robot motion execution. The wo UR5e arms and a 2FG7 are integrated into a modular system that can be extended with different tools for further applications.

## System Requirements
- ROS2 Jazzy
- MoveIt 2 (via binary install)
- Ubuntu 24.04
- UR5e Robots (else only simulated environment)
- OnRobot 2FG7 Gripper

## Installation
### ROS Jazzy installation 

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Always source ROS2 before using it. Simplest way to never forget it is to put it directly in the .bashrc file to automatically source it.

source /opt/ros/jazzy/setup.bash

### MoveIt 2 installation

https://moveit.ai/install-moveit2/binary/

MoveIt 2 was installed via binary install and is a separate workspace. Also here always source it before using motion planning! 

source $COLCON_WS/install/setup.bash

###Installing this repository

git clone 



