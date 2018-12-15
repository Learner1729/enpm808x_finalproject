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
 * @file       naivik_robot_node.cpp
 * @version    0.1
 * @author     Ashish Patel
 * @brief      ROS package entry point
 * @date       12-15-2018
 */

// including C++ Header files
#include <memory>

// including ROS Header file
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

// including user-defined header files
#include "naivik.hpp"
#include "motionController.hpp"
#include "camera.hpp"

int main(int argc, char **argv) {
  // The ros::init() function needs to see argc and argv so that it can perform
  // any ROS arguments and name remapping that were provided at the command line.
  // For programmatic remappings you can use a different version of init() which takes
  // remappings directly, but for most command-line programs, passing argc and argv is
  // the easiest way to do it.  The third argument to init() is the name of the node.
  // 
  // You must call one of the versions of ros::init() before using any other
  // part of the ROS system.
  // 
  // Initialize ROS and name our node "naivik_robot_node"
  ros::init(argc, argv, "naivik_robot_node");
  
  // NodeHandle is the main access point to communications with the ROS system.
  // The first NodeHandle constructed will fully initialize this node, and the last
  // NodeHandle destructed will close down the node.
  ros::NodeHandle nh;

  // Creating an instance of Naivik class using parameterized constructor
  Naivik naivik_robot(nh);
  
  // A ros::Rate object allows you to specify a frequency that you would like to loop at. 
  // It will keep track of how long it has been since the last call to Rate::sleep(), and 
  // sleep for the correct amount of time. Set up the publisher rate to 10 Hz.
  // Loop 10 times per second.
  ros::Rate loop_rate(10);
  
  // By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will 
  // cause ros::ok() to return false if that happens.
  // ros::ok() will return false if:
  // 1] a SIGINT is received (Ctrl-C)
  // 2] we have been kicked off the network by another node with the same name
  // 3] ros::shutdown() has been called by another part of the application.
  // 4] all ros::NodeHandles have been destroyed 
  // Once ros::ok() returns false, all ROS calls will fail. 
  while (ros::ok()) {
   
    naivik_robot.drive();

    // "Spin" a callback to call any callbacks. 
    ros::spinOnce();
    
    // Sleep for the remaining time until we hit our 10 Hz rate
    loop_rate.sleep();
  }
  return 0;
}
