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
 *        It sets the speed data of turtlebot
 * @param lSpeed of type double
 * @param aSpeed of type double
 */
MotionController::MotionController(double lSpeed, double aSpeed) {

}
  
/**
 * @brief MotionController class destructor
 */
MotionController::~MotionController() {

}
  
/**
 * @brief Determine a turtlebot action based on results from the obstacle
 *        detector
 * @param a reference to a variable of type sensor_msgs::LaserScan
 * @return none
 */
void MotionController::determineAction(
  const sensor_msgs::LaserScan::ConstPtr& msg) {

}
  
/**
 * @brief Return the current turtlebot action
 * @param none
 * @return turtlebot velocity data of type geometry_msgs::Twist
 */
geometry_msgs::Twist MotionController::getVehicleAction() {

}
 
/**
 * @brief set linear speed data of turtlebot
 * @param speed of type double
 * @return none
 */
void MotionController::setLinearSpeed(double speed) {

}

/**
 * @brief get linear speed data of turtlebot
 * @param none
 * @return linear speed of type double
 */
double MotionController::getLinearSpeed() {

}

/**
 * @brief set angular speed data of turtlebot
 * @param speed: double
 * @return none
 */
void MotionController::setAngularSpeed(double speed) {

}

/**
 * @brief get angular speed data of turtlebot
 * @param none
 * @return angular speed of type double
 */
double MotionController::getAngularSpeed() {
  
}
