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
 * @file       testService.cpp
 * @version    0.1
 * @author     Ashish Patel
 * @brief      This file is used to test ROS Services created
 * @date       12-15-2018
 */

// including gtest header file
#include <gtest/gtest.h>

// including ros header file
#include "ros/ros.h"
#include "ros/service_client.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"

// including user-defined Header files
#include "naivik.hpp"
#include "motionController.hpp"
#include "camera.hpp"

// including C++ header file associated with the service which defines datatype
// of request and response
#include "naivik_robot/changeThresholdService.h"
#include "naivik_robot/changeLinearSpeedService.h"
#include "naivik_robot/changeAngularSpeedService.h"
#include "naivik_robot/controlMotionService.h"
#include "naivik_robot/takeImageService.h"

/**
 * @brief Testing the changeThresholdService
 */
TEST(test_Services, change_threshold_service) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<naivik_robot::changeThresholdService>
    ("changeThresholdService");
  bool exists(client.waitForExistence(ros::Duration(2)));
  ASSERT_TRUE(exists);
  naivik_robot::changeThresholdService srv;
  srv.request.changeThreshold = 1.5;
  client.call(srv);
  EXPECT_TRUE(srv.response.response);
}

/**
 * @brief Testing the changeLinearSpeedService
 */
TEST(test_Services, change_linear_speed_service) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<naivik_robot::changeLinearSpeedService>
    ("changeLinearSpeedService");
  bool exists(client.waitForExistence(ros::Duration(2)));
  ASSERT_TRUE(exists);
  naivik_robot::changeLinearSpeedService srv;
  srv.request.changeLSpeed = 5.0;
  client.call(srv);
  EXPECT_TRUE(srv.response.response);
}

/**
 * @brief Testing the changeAngularSpeedService
 */
TEST(test_Services, change_angular_speed_service) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<naivik_robot::changeAngularSpeedService>
    ("changeAngularSpeedService");
  bool exists(client.waitForExistence(ros::Duration(2)));
  ASSERT_TRUE(exists);
  naivik_robot::changeAngularSpeedService srv;
  srv.request.changeASpeed = 5.0;
  client.call(srv);
  EXPECT_TRUE(srv.response.response);
}

/**
 * @brief Testing the controlMotionService
 */
TEST(test_Services, control_motion_service) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<naivik_robot::controlMotionService>
    ("controlMotionService");
  bool exists(client.waitForExistence(ros::Duration(2)));
  ASSERT_TRUE(exists);
  naivik_robot::controlMotionService srv;
  // Continue Motion
  srv.request.motion = false;
  client.call(srv);
  EXPECT_TRUE(srv.response.response);
  // Stop Motion
  srv.request.motion = true;
  client.call(srv);
  EXPECT_TRUE(srv.response.response);
}

/**
 * @brief Testing the takeImageService
 */
TEST(test_Services, take_image_service) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<naivik_robot::takeImageService>("takeImageService");
  bool exists(client.waitForExistence(ros::Duration(2)));
  ASSERT_TRUE(exists);
  naivik_robot::takeImageService srv;
  srv.request.request = true;
  client.call(srv);
  EXPECT_TRUE(srv.response.response);
}

/**
 * @brief Main function for testing, runs all the tests that were declared
 *        with TEST()
 * @param argc  The argc as int
 * @param argv  The argv as char array
 * @return 0, if everything is successful
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "test_services");
  ::testing::InitGoogleTest(&argc, argv);
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
