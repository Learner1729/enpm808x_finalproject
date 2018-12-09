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
 * @file       camera.hpp
 * @version    1.0
 * @author     Ashish Patel
 * @brief      Camera class header file
 * @date       12-02-2018
 */

#ifndef INCLUDE_CAMERA_HPP_
#define INCLUDE_CAMERA_HPP_

// including C++ Header files
#include <memory>
#include <string>
#include <vector>
#include <sstream>

// including ROS Header file
#include "ros/ros.h"

// including image_transport header file which is used for publishing and 
// subscribing to images in ROS
#include "image_transport/image_transport.h"

// including the header for CvBridge as well as some useful constants and
// functions related to image encodings. 
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

// including the headers for OpenCV's image processing and GUI modules
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// including ROS message header file
#include "sensor_msgs/Image.h"

// including service header file
#include "naivik_robot/takeImageService.h"

/**
 * @brief Camera class handles viewing and taking images in front of turtlebot
 */
class Camera {
 public:
  /**
   * @brief Camera constructor
   */
  Camera();

  /**
   * @brief Camera class parameterized constructor
   */
  Camera(bool imageCapture);
  
  /**
   * @brief Camera constructor
   */
  ~Camera();
  
  /**
   * @brief Take an image of the current camera view and store it for later use
   */
  bool takeImage(naivik_robot::takeImageService::Request& req,
    naivik_robot::takeImageService::Response& res);

  /**
   * @brief camera callback function
   */
  void cameraCallback(const sensor_msgs::ImageConstPtr& msg);

 private:
  /**
   * @brief Initialize a variable named saveImages_ to store image data 
   */
  std::vector<std::string> saveImages_;
  
  /**
   * @brief Initialize a variable named takeImageFlag_ as a flag to instruct whether to take image or not not
   */
  bool takeImageFlag_{false};
  
  /**
   * @brief Initialize a variable to Node handler for subscribing to service and topics
   */
  ros::NodeHandle nh_;
  
  /**
   * @brief Initialize a ROS service client object for the takeImageService
   */
  ros::ServiceClient cameraClient_;
};

#endif  // INCLUDE_CAMERA_HPP_