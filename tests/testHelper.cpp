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
 * @file       testHelper.cpp
 * @version    0.1
 * @author     Ashish Patel
 * @brief      TestHelper implementation file
 * @date       12-15-2018
 */

// including ros header file
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// including user-defined header file
#include "testHelper.hpp"

TestHelper::TestHelper() {
  ROS_INFO_STREAM("Test helper default constructor called...");
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
}
  
TestHelper::~TestHelper() {
  ROS_INFO_STREAM("Test helper destructor called...");
}

void TestHelper::velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  twist = *msg;
  ROS_INFO_STREAM("Twist.linear.x = " << twist.linear.x);
  ROS_INFO_STREAM("Twist.angular.z = " << twist.angular.z);
  return;
}
