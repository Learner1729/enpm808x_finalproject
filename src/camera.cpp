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
 * @file       camera.cpp
 * @version    0.1
 * @author     Ashish Patel
 * @brief      Camera class implementation file
 * @date       12-15-2018
 */

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
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

// including the headers for OpenCV's image processing and GUI modules
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// including service header file
#include "naivik_robot/takeImageService.h"

// including user-defined header file
#include "camera.hpp"

Camera::Camera(bool imageCapture): takeImageFlag_(imageCapture) {
  // Define a camera client object for takeImage service. 
  // It needs to know the data type of the service and its name.
  cameraClient_ = nh_.serviceClient<naivik_robot::takeImageService>("takeImage");

  // OpenCV HighGUI calls to create a display window
  // (uncomment below line if you want to see image_view in a separate window)
  // cv::namedWindow("OPENCV_IMAGE_WINDOW");
}

Camera::~Camera() {
  // (uncomment below line if you want to see image_view in a separate window)
  // cv::destroyWindow("OPENCV_IMAGE_WINDOW");
}

bool Camera::takeImage(naivik_robot::takeImageService::Request& req,
  naivik_robot::takeImageService::Response& res) {
  
  res.response = true;
  // ROS_INFO_STREAM("Success...!!");
  takeImageFlag_ = res.response;
  return true;
}

void Camera::cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  // We first convert the ROS image message to a CvImage suitable for
  // working with OpenCV. Since we need to save going an image, we need a 
  // mutable copy of it, so we use toCvCopy() instead of toCvShared().
  // Secondly, sensor_msgs::image_encodings::BGR8 is simply a constant for 
  // "bgr8", but less susceptible to typos.
  // There are also other image_encodings scheme which you can use.
  // 
  // Also, we should always wrap your calls to toCvCopy() / toCvShared() to 
  // catch conversion errors as those functions will not check for the 
  // validity of your data.
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  // (uncomment below lines if you want to see image_view in a separate window)
  // cv::imshow("OPENCV_IMAGE_WINDOW",cv_ptr->image);
  // cv::waitKey(3);

  // if takeImageFlag_ is enabled
  if(takeImageFlag_) {

    ROS_INFO_STREAM("New Image saved...");
    
    // a variable to store image count of saved Images
    static int imageCount = 0; 
    
    // A stringstream works essentially the same as an input/output file stream
    // You can write to it.
    std::stringstream sstream;
    
    // Giving filename and attaching count to it.
    sstream << "robot_env_" << imageCount << ".jpg";          
    
    // Write image data to a file.
    cv::imwrite(sstream.str(),  cv_ptr->image);

    // Add file to the list.
    saveImages_.push_back(sstream.str());
    
    // Increment the image counter...
    imageCount++;

    // Reset takeImageFlag_
    takeImageFlag_ = false;
  }
}
