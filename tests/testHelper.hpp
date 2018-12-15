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
 * @file       testHelper.hpp
 * @version    0.1
 * @author     Ashish Patel
 * @brief      Header file for TestHelper class
 * @date       12-15-2018
 */

#ifndef TESTS_TESTHELPER_HPP_
#define TESTS_TESTHELPER_HPP_

// including ros header file
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief TestHelper class handles in testing the nodes publisher and
 *        subscriber
 */
class TestHelper {
public:
  /**
   * @brief TestHelper class constructor
   * @param none
   * @return none
   */
  TestHelper();
  
  /**
   * @brief TestHelper class destructor
   * @param none
   * @return none
   */
  ~TestHelper();

  /**
   * @brief Callback function for velocity messages
   * @param a reference to a variable of type geometry_msgs::Twist
   * @return none
   */
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);

  /**
   * @brief declare a container for storing robot velocity messages
   */  
  geometry_msgs::Twist twist;
};

#endif  // TESTS_TESTHELPER_HPP_
