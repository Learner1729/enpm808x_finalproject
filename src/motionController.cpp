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
 * @file       motionController.hpp
 * @version    0.1
 * @author     Ashish Patel
 * @brief      MotionController class header file
 * @date       12-15-2018
 */

// including C++ Header files
#include <memory>

// including ROS Header files
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

// including user-defined Header file
#include "motionController.hpp"
#include "obstacleDetection.hpp"
#include "naivik_robot/changeThresholdService.h"
#include "naivik_robot/changeLinearSpeedService.h"
#include "naivik_robot/changeAngularSpeedService.h"
#include "naivik_robot/controlMotionService.h"

MotionController::MotionController(double lSpeed, double aSpeed):
  linearSpeed_(lSpeed), angularSpeed_(aSpeed),
  obstacleDetection_(std::make_shared<ObstacleDetection>(1.0)) {
  // intialize robot action to stay still
  naivikAction_.linear.x = 0.0;
  naivikAction_.linear.y = 0.0;
  naivikAction_.linear.z = 0.0;
  naivikAction_.angular.x = 0.0;
  naivikAction_.angular.y = 0.0;
  naivikAction_.angular.z = 0.0;
}

MotionController::~MotionController() {
}

void MotionController::determineAction(
  const sensor_msgs::LaserScan::ConstPtr& msg) {
  // initializing a geometry_msgs variable which will be updated according 
  // to obstacle detector
  geometry_msgs::Twist velocity;
  velocity.linear.x = 0.0;
  velocity.linear.y = 0.0;
  velocity.linear.z = 0.0;
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = 0.0;
  if (stopRobot_ == false) {
    if (obstacleDetection_->detectObstacle(msg)) { 
      ROS_INFO_STREAM("Stop and turn the until robot is free.");
      // set linear velocity to zero
      velocity.linear.x = 0.0;
      // set turn rate about the z-axis
      velocity.angular.z = angularSpeed_;
    }
    else {
      // set turn rate to zero
      velocity.angular.z = 0.0;
      // move forward slowly
      velocity.linear.x = linearSpeed_;
    }
  }
  else {
    // stop the Motion
    velocity.angular.z = 0.0;
    velocity.linear.x = 0.0;
  }
  // set naivik action:
  naivikAction_ = velocity;
}

geometry_msgs::Twist MotionController::getVehicleAction() {
  return naivikAction_;
}

void MotionController::setLinearSpeed(double speed) {
  linearSpeed_ = speed;
}

double MotionController::getLinearSpeed() {
  return linearSpeed_;
}

void MotionController::setAngularSpeed(double speed) {
  angularSpeed_ = speed;
}

double MotionController::getAngularSpeed() {
  return angularSpeed_;
}

bool MotionController::changeThreshold(
  naivik_robot::changeThresholdService::Request& req,
  naivik_robot::changeThresholdService::Response& res) {

  obstacleDetection_->setDistThreshold(req.changeThreshold);
  res.response = true;
  ROS_INFO("Setting Distance Threshold: %f", req.changeThreshold);
  return res.response;
}

bool MotionController::changeLinearSpeed(
  naivik_robot::changeLinearSpeedService::Request& req,
  naivik_robot::changeLinearSpeedService::Response& res) {
  
  setLinearSpeed(req.changeLSpeed);
  res.response = true;
  ROS_INFO("Updated linearSpeed to: %f", req.changeLSpeed);
  return res.response;
}

bool MotionController::changeAngularSpeed(
  naivik_robot::changeAngularSpeedService::Request& req,
  naivik_robot::changeAngularSpeedService::Response& res) {
  
  setAngularSpeed(req.changeASpeed);
  res.response = true;
  ROS_INFO("Updated angularSpeed to: %f", req.changeASpeed);
  return res.response;
}

bool MotionController::controlMotion(
  naivik_robot::controlMotionService::Request& req,
  naivik_robot::controlMotionService::Response& res) {

  stopRobot_ = req.motion;
  res.response = true;
  if (stopRobot_) {
    ROS_INFO_STREAM("Robot Stopped...");
  } else {
    ROS_INFO_STREAM("Robot motion restarted...");
  }
  return res.response;
}
