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
 * @file       obstacleDetectionTest.cpp
 * @version    1.0
 * @author     Ashish Patel
 * @brief      File to test ObstacleDetection class
 * @date       12-02-2018
 */

// including C++ header file

// including ros header file
#include "ros/ros.h"

// including gtest header file
#include <gtest/gtest.h>

// including user-defined header file
#include "obstacleDetection.hpp"

/**
 * @brief Test the ability to get distThreshold_ variable
 */
TEST(TestSuite, get_distance_threshold) {
  std::shared_ptr<ObstacleDetection> obs(std::make_shared<ObstacleDetection>(10.0));
  ASSERT_DOUBLE_EQ(10.0, obs->getDistThreshold());
}

/**
 * @brief Test the ability to set distThreshold_ variable
 */
TEST(TestSuite, set_distance_threshold) {
  std::shared_ptr<ObstacleDetection> obs(std::make_shared<ObstacleDetection>(10.0));
  obs->setDistThreshold(12.1);
  EXPECT_DOUBLE_EQ(12.1, obs->getDistThreshold());
}

/**
 * @brief Test the ability to detect Obstacle
 */
TEST(TestSuite, detect_obstacle) {
  // work under progress...
  EXPECT_EQ(1, 1);
}