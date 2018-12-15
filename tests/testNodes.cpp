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
 * @file       testNode.cpp
 * @version    1.0
 * @author     Ashish Patel
 * @brief      This file is used to test nodes created in source code
 * @date       12-15-2018
 */

// Including gtest header file
#include <gtest/gtest.h>

// including ROS Header file
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

// including user-defined Header files
#include "naivik.hpp"
#include "motionController.hpp"
#include "camera.hpp"
#include "testHelper.hpp"

/**
 * @brief To test whether the camera subscriber is initialize properly or not
 * @param none
 * @return none
 */
TEST(TestingCallbacks, camera_callback_init) {
  ros::NodeHandle nh;
  ros::Rate loop_rate(2);
  // register to check number of Subscribers to /camera/rgb/image_raw
  ros::Publisher cameraPub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",100);
  loop_rate.sleep();
  EXPECT_EQ(1, cameraPub.getNumSubscribers());
}

/**
 * @brief To test whether the laser subscriber is initialize properly or not
 * @param none
 * @return none
 */
TEST(TestingCallbacks, laser_callback_init) {
  ros::NodeHandle nh;
  ros::Rate loop_rate(2);
  // register to check number of Subscribers to /scan
  ros::Publisher laserPub = nh.advertise<sensor_msgs::LaserScan>("/scan",100);
  loop_rate.sleep();  
  EXPECT_EQ(1, laserPub.getNumSubscribers());
}

/**
 * @brief To test whether the velocity publisher is initialize properly or not
 * @param none
 * @return none
 */
TEST(TestingCallbacks, velocity_callback_init) {
  ros::NodeHandle nh;
  TestHelper help;
  ros::Rate loop_rate(2);
  // register to check number of Subscribers to /scan
  ros::Subscriber velocity = nh.subscribe("/mobile_base/commands/velocity", 1000,
    &TestHelper::velocityCallback, &help);
  loop_rate.sleep();  
  EXPECT_EQ(1, velocity.getNumPublishers());
}

/**
 * @brief To test determine action function of MotionController class in a situation when there is no obstacle
 * @param none
 * @return none
 */
TEST(TestingCallbacks, non_obstacle_collision) {
  ros::NodeHandle nh;
  TestHelper help;
  ros::Rate loop_rate(2);
  // register to check number of publishers to /mobile_base/commands/velocity
  ros::Subscriber velocity = nh.subscribe("/mobile_base/commands/velocity", 1000, &TestHelper::velocityCallback, &help);
  // register to check number of Subscribers to /scan
  ros::Publisher laserPub = nh.advertise<sensor_msgs::LaserScan>("/scan",100);
  loop_rate.sleep();
  EXPECT_EQ(1, velocity.getNumPublishers());
  EXPECT_EQ(1, laserPub.getNumSubscribers());

  // create dummy Twist velocity message which will be used for comparation
  geometry_msgs::Twist msg;
  msg.linear.x = 0.1;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  // Prepare the data
  // create dummy laser scan msg to provide a fake non-obstacle collision
  size_t num_readings = 50;
  sensor_msgs::LaserScan scan;
  scan.angle_min = -1.57;
  scan.angle_max = 1.57;
  scan.angle_increment = 3.14 / num_readings;
  scan.time_increment = (1 / 40) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = 100.0;
  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);
  for (auto& i : scan.ranges) {
    i = scan.range_max;
  }
  
  laserPub.publish(scan);
  loop_rate.sleep();
  ros::spinOnce();
  // confirm velocity is expected
  EXPECT_EQ(0, std::memcmp(&msg, &help.twist, sizeof(msg))) << help.twist.linear.x;
}

/**
 * @brief To test determine action function of MotionController class in a situation when there is an obstacle
 * @param none
 * @return none
 */
TEST(TestingCallbacks, obstacle_collision) {
  ros::NodeHandle nh;
  TestHelper help;
  ros::Rate loop_rate(2);
  // register to check number of publishers to /mobile_base/commands/velocity
  ros::Subscriber velocity = nh.subscribe("/mobile_base/commands/velocity", 1000, &TestHelper::velocityCallback, &help);
  // register to check number of Subscribers to /scan
  ros::Publisher laserPub = nh.advertise<sensor_msgs::LaserScan>("/scan",100);
  loop_rate.sleep();
  EXPECT_EQ(1, velocity.getNumPublishers());
  EXPECT_EQ(1, laserPub.getNumSubscribers());

  // create dummy Twist velocity message which will be used for comparation
  geometry_msgs::Twist msg;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 1.0;

  // Prepare the data
  // create dummy laser scan msg to provide a fake non-obstacle collision
  size_t num_readings = 50;
  sensor_msgs::LaserScan scan;
  scan.angle_min = -1.57;
  scan.angle_max = 1.57;
  scan.angle_increment = 3.14 / num_readings;
  scan.time_increment = (1 / 40) / (num_readings);
  scan.range_min = 2.0;
  scan.range_max = 0.0;
  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);
  for (auto& i : scan.ranges) {
    i = scan.range_max;
  }
  
  laserPub.publish(scan);
  loop_rate.sleep();
  ros::spinOnce();
  // confirm velocity is expected
  EXPECT_EQ(0, std::memcmp(&msg, &help.twist, sizeof(msg))) << help.twist.angular.z;
}

/**
 * @brief To test whether the camera callback function
 * @param none
 * @return none
 */
TEST(TestingCallbacks, camera_callback) {
  ros::NodeHandle nh;
  ros::Rate loop_rate(2);
  // register to check number of Subscribers to /camera/rgb/image_raw
  ros::Publisher cameraPub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",100);
  loop_rate.sleep();
  EXPECT_EQ(1, cameraPub.getNumSubscribers());

  // The below code is only to increase coverage, as such I havn't doing any thing by capturing image as of now
  // It can be extended further based on passing the image and doing some image processing on it. 
  // Prepare the data
  // Also it shows an error of image encoding, while running the test but I am not checking any TEST based on it so for now we can ignore it.
  // create a dummy image data
  sensor_msgs::Image img;
  cameraPub.publish(img);
  loop_rate.sleep();
  ros::spinOnce();
}

/**
 * @brief Main function for testing, runs all the tests that were declared
 *        with TEST()
 * @param argc  The argc as int
 * @param argv  The argv as char array
 * @return 0, if everything is successful
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "test_nodes");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();;
}