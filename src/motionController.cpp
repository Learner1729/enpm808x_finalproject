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
 * @version    1.0
 * @author     Ashish Patel
 * @brief      MotionController class header file
 * @date       12-02-2018
 */

// including ROS Header files
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

// including user-defined Header file
#include "motionController.hpp"
#include "obstacleDetection.hpp"

/**
 * @brief MotionController class overloaded constructor
 *        It sets the speed data of naivik robot
 * @param lSpeed of type double
 * @param aSpeed of type double
 */
MotionController::MotionController(double lSpeed, double aSpeed):
  linearSpeed_(lSpeed), angularSpeed_(aSpeed),
  obstacleDetection_(std::make_shared<ObstacleDetection>(1.0)) {
  // Intialize naivik action to stay still
  naivikAction_.linear.x = 0.0;
  naivikAction_.linear.y = 0.0;
  naivikAction_.linear.z = 0.0;
  naivikAction_.angular.x = 0.0;
  naivikAction_.angular.y = 0.0;
  naivikAction_.angular.z = 0.0;
}
  
/**
 * @brief MotionController class destructor
 */
MotionController::~MotionController() {
}
  
/**
 * @brief Determine a naivik action based on results from the obstacle
 *        detector
 * @param a reference to a variable of type sensor_msgs::LaserScan
 * @return none
 */
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
  
  if (obstacleDetection_->detectObstacle(msg)) { 
    ROS_INFO_STREAM("Stop and turn the until naivik is free.");
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
  // set naivik action:
  naivikAction_ = velocity;
}
  
/**
 * @brief Return the current naivik action
 * @param none
 * @return naivik velocity data of type geometry_msgs::Twist
 */
geometry_msgs::Twist MotionController::getVehicleAction() {
  return naivikAction_;
}
 
/**
 * @brief set linear speed data of naivik
 * @param speed of type double
 * @return none
 */
void MotionController::setLinearSpeed(double speed) {
  linearSpeed_ = speed;
}

/**
 * @brief get linear speed data of naivik
 * @param none
 * @return linear speed of type double
 */
double MotionController::getLinearSpeed() {
  return linearSpeed_;
}

/**
 * @brief set angular speed data of naivik
 * @param speed: double
 * @return none
 */
void MotionController::setAngularSpeed(double speed) {
  angularSpeed_ = speed;
}

/**
 * @brief get angular speed data of naivik
 * @param none
 * @return angular speed of type double
 */
double MotionController::getAngularSpeed() {
  return angularSpeed_;
}

/**
 * @brief Callback back function for changeThreshold function
 */
bool MotionController::changeThreshold(
  naivik_robot::changeThresholdService::Request& req,
  naivik_robot::changeThresholdService::Response& res) {
  obstacleDetection_->setDistThreshold(req.changeThreshold);
  res.response = true;
  ROS_INFO("Setting Distance Threshold: %f", req.changeThreshold);
  return res.response;
}
