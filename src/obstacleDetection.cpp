/**
 * MIT License
 *
 * Copyright (c) 2018 Ashish Patel
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/**
 * @file       obstacleDetection.cpp
 * @version    0.1
 * @author     Ashish Patel
 * @brief      ObstacleDetection class header file
 * @date       12-15-2018
 */

// including ROS Header files
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

// including user-defined Header file
#include "obstacleDetection.hpp"

ObstacleDetection::ObstacleDetection(double distThreshold):
  distThreshold_(distThreshold) {
}

ObstacleDetection::~ObstacleDetection() {
}

bool ObstacleDetection::detectObstacle(
  const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Check if any scan data from the laser is less than range_min +
  // distThreshold from the front of the robot. If so, a collision is about
  // to occur, return true
  for (auto i : msg->ranges) {
    if (i < msg->range_min + distThreshold_) {
      ROS_WARN_STREAM("About to hit... Obstacle ahead!!!");
      return true;
    }
  }
  // return false, as collision is not about to occur
  return false;
}

void ObstacleDetection::setDistThreshold(double threshold) {
  distThreshold_ = threshold;
}

double ObstacleDetection::getDistThreshold() {
  return distThreshold_;
}
