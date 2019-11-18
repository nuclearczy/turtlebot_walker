/** Copyright (c) 2019   Zuyang Cao
 *  @file       walker.h
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

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Walker {
 private:
  // Status for obstacle
  bool obstacleAhead;

  // Twist msg
  geometry_msgs::Twist msg;

  // Publisher Vel
  ros::Publisher pubVel;

  // Subscriber object
  ros::Subscriber sub;

  // Speed
  float botSpeed;

  // Angle
  float botAngle;

  // Clearance between turtlebot and obstacle
  double clearance;

  // Initiate NodeHandle object
  ros::NodeHandle nh;

 public:
  /** @brief Default constructor*/
  Walker();

  /** @brief Constructor with initial values
   *  @param speed speed of turtlebot
   *  @param angle angle of turtlebot
   *  @param clr clearance
   *  @return void
   */
  Walker(const float& speed, const float& angle,
         const double& clr);

  /** @brief Default destructor*/
  ~Walker();

  /** @brief Check if any obstacle in clearance
   *  @return boolean
   */
  bool getObstacleAhead();

  /** @brief Start walking
   *  @return void
   */
  void startWalking();

  /** @brief Callback function for laserscanner
   *  @param msg data from LaserScan node
   *  @return void
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief reset velocities of turtlebot
   * @return void
   */
  void resetBot();
};
#endif  // INCLUDE_WALKER_HPP_

