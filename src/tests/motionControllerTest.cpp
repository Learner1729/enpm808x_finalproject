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
 * @file       motionControllerTest.cpp
 * @version    1.0
 * @author     Ashish Patel
 * @brief      File to test MotionController class
 * @date       12-02-2018
 */

// including ros header file
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// including gtest header file
#include <gtest/gtest.h>

// including user-defined header file
#include "motionController.hpp"
#include "naivik_robot/changeThresholdService.h"

/**
 * @brief Test the ability to determineAction
 */
TEST(TestSuite, determine_action) {
  // work under progress...
  EXPECT_EQ(1, 1);
}

/**
 * @brief Test the ability to getVehicleAction
 */
TEST(TestSuite, get_vehicle_action) {
  std::shared_ptr<MotionController> mc(std::make_shared<MotionController>(1.0,1.0));
  // initializing a geometry_msgs variable
  geometry_msgs::Twist velocity = mc->getVehicleAction();
  // temporary variable
  auto flag = false;
  if(velocity.linear.x == 0.0 && velocity.linear.y == 0.0 && 
    velocity.linear.z == 0.0 && velocity.angular.x == 0.0 && 
    velocity.angular.y == 0.0 && velocity.angular.z == 0.0) {
    flag = true;
  }
  EXPECT_EQ(true,flag);
}

/**
 * @brief Test the ability to get & set linearSpeed_
 */
TEST(TestSuite, set_linear_speed) {
  std::shared_ptr<MotionController> mc(std::make_shared<MotionController>(1.0,1.0));
  ASSERT_DOUBLE_EQ(1.0, mc->getLinearSpeed());
  mc->setLinearSpeed(4.2);
  EXPECT_DOUBLE_EQ(4.2, mc->getLinearSpeed());
}

/**
 * @brief Test the ability to get & set AngularSpeed_
 */
TEST(TestSuite, get_linear_speed) {
  std::shared_ptr<MotionController> mc(std::make_shared<MotionController>(1.0,1.0));
  ASSERT_DOUBLE_EQ(1.0, mc->getAngularSpeed());
  mc->setAngularSpeed(4.2);
  EXPECT_DOUBLE_EQ(4.2, mc->getAngularSpeed());
}

 