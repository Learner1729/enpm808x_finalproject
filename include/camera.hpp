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
 * @files      camera.hpp
 * @version    0.1
 * @author     Ashish Patel
 * @brief      Camera class header file
 * @date       12-15-2018
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
#include "sensor_msgs/Image.h"

// including the headers for OpenCV's image processing and GUI modules
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// including service header file
#include "naivik_robot/takeImageService.h"

/**
 * @brief Camera class handles viewing and taking images in front of the robot
 */
class Camera {
 public:
  /**
   * @brief Camera class parameterized constructor
   * @param imageCapture of type bool, it's default value is false
   * @return none
   */
  Camera(bool imageCapture);
  
  /**
   * @brief Camera class destructor
   * @param none
   * @return none
   */
  ~Camera();

  /**
   * @brief A callback function to trigger a flag to capture an image of the
   *        current camera view and store it for later use
   * @param a reference to a request variable of type defined in 
   *        takeImageService.srv file
   * @param a reference to a response variable of type defined in 
   *        takeImageService.srv file
   * @return a boolean on success or failure
   */
  bool takeImage(naivik_robot::takeImageService::Request& req,
    naivik_robot::takeImageService::Response& res);

  /**
   * @brief A cameraCallback function to convert images from ROS image messages
   *        to the useable format and store it based on the takeImageFlag_
   * @param a reference to a variable of type sensor_msgs::Image
   * @return none
   */
  void cameraCallback(const sensor_msgs::ImageConstPtr& msg);

 private:
  /**
   * @brief declare a container named saveImages_ to store image data 
   */
  std::vector<std::string> saveImages_;
  
  /**
   * @brief declare and initialize a container named takeImageFlag_ as a flag to
   *        instruct whether to capture and store image or not
   */
  bool takeImageFlag_{false};
  
  /**
   * @brief declare a container to Node handler for subscribing to service 
   *        and topics
   */
  ros::NodeHandle nh_;
  
  /**
   * @brief declare a ROS service client object for the takeImageService
   */
  ros::ServiceClient cameraClient_;
};

#endif  // INCLUDE_CAMERA_HPP_
