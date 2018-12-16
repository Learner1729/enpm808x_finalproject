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
 * @version    0.1
 * @author     Ashish Patel
 * @brief      Test cases for ObstacleDetection class
 * @date       12-15-2018
 */

// including gtest header file
#include <gtest/gtest.h>

// including C++ header file
#include <memory>

// including user-defined header file
#include "obstacleDetection.hpp"

// including ros header file
#include "ros/ros.h"

/**
 * @brief ObstacleDetection fixture class
 */
class ObstacleDetectionTest: public ::testing::Test {
 public:
  std::shared_ptr<ObstacleDetection> obs;
  void SetUp() {
    obs = std::make_shared<ObstacleDetection>(10.0);
  }
};

/**
 * @brief Test the ability to get distThreshold_ variable
 */
TEST_F(ObstacleDetectionTest, get_distance_threshold) {
  ASSERT_DOUBLE_EQ(10.0, obs->getDistThreshold());
}

/**
 * @brief Test the ability to set distThreshold_ variable
 */
TEST_F(ObstacleDetectionTest, set_distance_threshold) {
  obs->setDistThreshold(12.1);
  EXPECT_DOUBLE_EQ(12.1, obs->getDistThreshold());
}
