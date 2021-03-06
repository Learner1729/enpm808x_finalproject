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
 * @file       naivik.hpp
 * @version    0.1
 * @author     Ashish Patel
 * @brief      Naivik class header file
 * @date       12-15-2018
 */

#ifndef INCLUDE_NAIVIK_HPP_
#define INCLUDE_NAIVIK_HPP_

// including C++ Header files
#include <memory>

// including ROS Header file
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

// including user-defined Header files
#include "motionController.hpp"
#include "camera.hpp"

/**
 * @brief Naivik class handles the camera and motion controller interactions
 *        of the robot
 */
class Naivik {
 public:
  /**
   * @brief Overloaded contructor
   * @param nh ROS node handle
   * @return none
   */
  explicit Naivik(ros::NodeHandle nh);

  /**
   * @brief Naivik class destructor
   * @param none
   * @return none
   */
  ~Naivik();

  /**
   * @brief Drive the robot using laser scan data
   * @param none
   * @return none
   */
  void drive();

 private:
  /**
   * @brief declare a container for camera class object and initialize it to 
   *        nullptr
   */
  std::shared_ptr<Camera> camera_{nullptr};

  /**
   * @brief declare a container for MotionController class object and 
   *        initialize it to nullptr
   */
  std::shared_ptr<MotionController> motionController_{nullptr};

  /**
   * @brief declare a container to Node handler for subscribing to services 
   *        and topics
   */
  ros::NodeHandle nh_;

  /**
   * @brief declare a container for a ROS publisher to publish vehicle motion
   *        commands
   */
  ros::Publisher velocityPublisher_;

  /**
   * @brief declare a container for a ROS subscriber for laser scan topics
   */
  ros::Subscriber laserSubscriber_;

  /**
   * @brief declare a container for a ROS subscriber for camera topics
   */
  ros::Subscriber cameraSubscriber_;

  /**
   * @brief declare a container for ROS changeThresholdService 
   */
  ros::ServiceServer changeThresholdServer_;

  /**
   * @brief declare a container for ROS changeLinearSpeedService 
   */
  ros::ServiceServer changeLinearSpeedServer_;

  /**
   * @brief declare a container for ROS changeAngularSpeedService 
   */
  ros::ServiceServer changeAngularSpeedServer_;

  /**
   * @brief declare a container for ROS controlMotionService 
   */
  ros::ServiceServer controlMotionServer_;

  /**
   * @brief declare a container for ROS takeImageService 
   */
  ros::ServiceServer takeImageServer_;
};

#endif  // INCLUDE_NAIVIK_HPP_
