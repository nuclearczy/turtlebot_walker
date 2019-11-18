# turtlebot_walker
[![Build Status](https://travis-ci.com/nuclearczy/turtlebot_walker.svg?branch=master)](https://travis-ci.com/nuclearczy/turtlebot_walker)
[![GitHub](https://github.com/nuclearczy/turtlebot_walker/blob/master/LICENSE)](https://img.shields.io/github/license/nuclearczy/turtlebot_walker)

## Overview

The package turtlebot_walker contains a class Walker to drive the turtlebot forward and turn the robot 
when the turtlebot scans an obstacle in the clearance area.

## Dependencies

- Ubuntu 16.04 LTS
- ros-kinetic-desktop-full
- Gazebo turtlebot packages

## Build Steps

''' bash
cd ~/catkin_ws/src
git clone https://github.com/nuclearczy/turtlebot_walker
cd ..
catkin_make
source devel/setup.bash
'''

## Before Running

Assume ros-kinetic-desktop-full is already installed on Ubuntu 16.04 LTS.
''' bash
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-turtlebot-gazebo 
sudo apt-get install ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
'''

## Run the Launch File

Run the launch file without ROS bag record:
''' bash
roslaunch turtlebot_walker turtlebot_walk.launch
'''

Run the launch file without ROS bag record:
''' bash
roslaunch turtlebot_walker turtlebot_walk.launch rosbagRecord:=true
'''

Gazebo should start automatically and the turtlebot will run as project described.
The ROS bag will be saved in the results folder.

## ROS Bag Playback

In terminal 1:
''' bash
roscore
'''

In terminal 2:
''' bash
rosbag play turtlebotRecord.bag
'''

In terminal 3:
''' bash
rostopic echo /mobile_base/commands/velocity
'''



