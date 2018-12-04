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
 * @file       naivik.cpp
 * @version    1.0
 * @author     Ashish Patel
 * @brief      Naivik class implementation file
 * @date       12-02-2018
 */

// including ROS Header file
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

// including user-defined Header files
#include "naivik.hpp"
#include "motionController.hpp"
#include "camera.hpp"

/**
 * @brief Default Naivik class contructor
 */
Naivik::Naivik() {
}
  
/**
 * @brief Overloaded contructor
 * @param nh ROS node handle as reference
 */
Naivik::Naivik(ros::NodeHandle nh):nh_(nh),
  motionController_(std::make_shared<MotionController>(0.1,1.0)),
  camera_(std::make_shared<Camera>()) {
  
  laserSubscriber_ = nh_.subscribe <sensor_msgs::LaserScan> 
    ("/scan", 100, &MotionController::determineAction, motionController_.get());
  velocityPublisher_ = nh_.advertise <geometry_msgs::Twist> 
    ("/mobile_base/commands/velocity", 100);
}
  
/**
 * @brief Naivik class destructor
 */
Naivik::~Naivik() {
}
  
/**
 * @brief Drive the turtlebot using laser scan data
 * @param none
 * @return none
 */
void Naivik::drive() {
  // Grab current naivik robot action
  geometry_msgs::Twist naivikCommand = motionController_->getVehicleAction();
  // Publish command
  velocityPublisher_.publish(naivikCommand);
}
