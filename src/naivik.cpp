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
 * @version    0.1
 * @author     Ashish Patel
 * @brief      Naivik class implementation file
 * @date       12-15-2018
 */

// including C++ Header files
#include <memory>

// including ROS Header file
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

// including user-defined Header files
#include "naivik.hpp"
#include "motionController.hpp"
#include "camera.hpp"

Naivik::Naivik(ros::NodeHandle nh):nh_(nh),
  motionController_(std::make_shared<MotionController>(0.1, 1.0)),
  camera_(std::make_shared<Camera>(false)) {
  laserSubscriber_ = nh_.subscribe <sensor_msgs::LaserScan>
    ("/scan", 100, &MotionController::determineAction, motionController_.get());
  velocityPublisher_ = nh_.advertise <geometry_msgs::Twist>
    ("/mobile_base/commands/velocity", 100);
  cameraSubscriber_ = nh_.subscribe <sensor_msgs::Image>
    ("/camera/rgb/image_raw", 100, &Camera::cameraCallback, camera_.get());

  changeThresholdServer_ = nh_.advertiseService("changeThresholdService",
    &MotionController::changeThreshold, motionController_.get());
  changeLinearSpeedServer_ = nh_.advertiseService("changeLinearSpeedService",
    &MotionController::changeLinearSpeed, motionController_.get());
  changeAngularSpeedServer_ = nh_.advertiseService("changeAngularSpeedService",
    &MotionController::changeAngularSpeed, motionController_.get());
  controlMotionServer_ = nh_.advertiseService("controlMotionService",
    &MotionController::controlMotion, motionController_.get());
  takeImageServer_ = nh_.advertiseService("takeImageService",
    &Camera::takeImage, camera_.get());
}

Naivik::~Naivik() {
}

void Naivik::drive() {
  // grab current robot action
  geometry_msgs::Twist naivikCommand = motionController_->getVehicleAction();
  // Publish command
  velocityPublisher_.publish(naivikCommand);
}
