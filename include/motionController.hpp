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

#ifndef INCLUDE_MOTIONCONTROLLER_HPP_
#define INCLUDE_MOTIONCONTROLLER_HPP_

// including C++ Header files
#include <memory>

// including ROS defined Header files
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

// including user-defined Header file
#include "obstacleDetection.hpp"
#include "naivik_robot/changeThresholdService.h"
#include "naivik_robot/changeLinearSpeedService.h"
#include "naivik_robot/changeAngularSpeedService.h"
#include "naivik_robot/controlMotionService.h"

/**
 * @brief MotionController class determine's robot control actions. 
 *        ROS doesn't provide any controller, thus we need to design our own 
 *        based on the requirement that we have
 */
class MotionController {
 public:
  /**
   * @brief MotionController class overloaded constructor
   *        It sets the speed data of robot
   * @param lSpeed of type double
   * @param aSpeed of type double
   */
  explicit MotionController(double lSpeed, double aSpeed);

  /**
   * @brief MotionController class destructor
   * @param none
   * @return none
   */
  ~MotionController();

  /**
   * @brief Determine robot's actions based on results from the obstacle 
   *        detector
   * @param a reference to a variable of type sensor_msgs::LaserScan
   * @return none
   */
  void determineAction(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief Return the current turtlebot action
   * @param none
   * @return robot's velocity data of type geometry_msgs::Twist
   */
  geometry_msgs::Twist getVehicleAction();

  /**
   * @brief Set linear speed of the robot
   * @param speed of type double
   * @return none
   */
  void setLinearSpeed(double speed);

  /**
   * @brief Get linear speed of the robot
   * @param none
   * @return robot's linear speed data of type double
   */
  double getLinearSpeed();

  /**
   * @brief Set angular speed of the robot
   * @param speed of type double
   * @return none
   */
  void setAngularSpeed(double speed);

  /**
   * @brief get angular speed of the robot
   * @param none
   * @return robot's angular speed data of type double
   */
  double getAngularSpeed();

  /**
   * @brief callback back function for changeThresholdService
   * @param a reference to a request variable of type defined in 
   *        changeThresholdService.srv file
   * @param a reference to a response variable of type defined in 
   *        changeThresholdService.srv file
   * @return a boolean on success or failure
   */
  bool changeThreshold(
    naivik_robot::changeThresholdService::Request& req,
    naivik_robot::changeThresholdService::Response& res);

  /**
   * @brief callback back function for changeLinearSpeedService
   * @param a reference to a request variable of type defined in 
   *        changeLinearSpeedService.srv file
   * @param a reference to a response variable of type defined in 
   *        changeLinearSpeedService.srv file
   * @return a boolean on success or failure
   */
  bool changeLinearSpeed(
    naivik_robot::changeLinearSpeedService::Request& req,
    naivik_robot::changeLinearSpeedService::Response& res);

  /**
   * @brief callback back function for changeAngularSpeedService
   * @param a reference to a request variable of type defined in 
   *        changeAngularSpeedService.srv file
   * @param a reference to a response variable of type defined in 
   *        changeAngularSpeedService.srv file
   * @return a boolean on success or failure
   */
  bool changeAngularSpeed(
    naivik_robot::changeAngularSpeedService::Request& req,
    naivik_robot::changeAngularSpeedService::Response& res);

  /**
   * @brief callback back function for controlMotionService
   * @param a reference to a request variable of type defined in 
   *        controlMotionService.srv file
   * @param a reference to a response variable of type defined in 
   *        controlMotionService.srv file
   * @return a boolean on success or failure
   */
  bool controlMotion(
    naivik_robot::controlMotionService::Request& req,
    naivik_robot::controlMotionService::Response& res);

 private:
  /**
   * @brief declare a container for ObstacleDetection class object and 
   *        initialize it to nullptr
   */
  std::shared_ptr<ObstacleDetection> obstacleDetection_{nullptr};

  /**
   * @brief declare a container for linearSpeed data
   */
  double linearSpeed_;

  /**
   * @brief declare a container for angularSpeed data
   */
  double angularSpeed_;

  /**
   * @brief declare a container for Twist message to be sent to the robot
   */
  geometry_msgs::Twist naivikAction_;

  /**
   * @brief declare a container to denote robot status and initialize it to
   *        false
   */
  bool stopRobot_{false};
};

#endif  // INCLUDE_MOTIONCONTROLLER_HPP_
