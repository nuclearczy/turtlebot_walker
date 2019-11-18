/** Copyright (c) 2019   Zuyang Cao
 *  @file       walker.cpp
 *  @brief      A simple walker class for turtlebot.
 *  @license    BSD 3-Clause LICENSE
 *
 * Copyright (c) 2018, Zuyang Cao
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without  
 * modification, are permitted provided that the following conditions are 
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the   
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include "std_msgs/String.h"
#include "../include/walker.h"

// Default Walker constructor
Walker::Walker() {
  botSpeed = 0.2;
  botAngle = 0.8;
  clearance = 0.5;
  obstacleAhead = false;
  pubVel = nh.advertise<geometry_msgs::Twist>
           ("/cmd_vel_mux/input/navi", 1000);
  sub = nh.subscribe<sensor_msgs::LaserScan>
        ("/scan", 500, &Walker::laserCallback, this);
  ROS_INFO_STREAM("Default setting initialized.");
  resetBot();
}

// Implementation of custom Walker constructor
Walker::Walker(const float& speed, const float& angle,
               const double& clr) {
  botSpeed = speed;
  obstacleAhead = false;
  if (angle > 1.57 || angle < -1.57) {
    ROS_WARN_STREAM("Angle exceeds 90 deg, setting to 90 deg.");
    botAngle = 1.57;
  } else {
    botAngle = angle;
  }
  clearance = clr;
  ROS_INFO_STREAM("Custom setting initialized.");
}

// Destructor that resets the velocity of the bot
Walker::~Walker() {
  resetBot();
}

// Reset velocities to 0
void Walker::resetBot() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  pubVel.publish(msg);
}

// Get obstacleAhead
bool Walker::getObstacleAhead() {
  return obstacleAhead;
}

// Start walking
void Walker::startWalking() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    if (getObstacleAhead()) {
      ROS_WARN_STREAM("Obstacle detected.");
      msg.linear.x = 0.0;
      msg.angular.z = botAngle;
    } else {
      msg.angular.z = 0.0;
      msg.linear.x = botSpeed;
    }
    pubVel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

// Callback function for the subscriber
void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (auto m : msg->ranges) {
    if (m < clearance) {
      obstacleAhead = true;
      return;
    }
  }
  obstacleAhead = false;
}
