/**
 * @file controller.hpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-12-03
 *
 * MIT License
 * Copyright (c) 2022 Akash Ravindra
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#ifndef TURTLE_WALKER_INCLUDE_TURTLE_WALKER_CONTROLLER_HPP_
#define TURTLE_WALKER_INCLUDE_TURTLE_WALKER_CONTROLLER_HPP_
#include <chrono>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

namespace walker {
class Walker : public rclcpp::Node {
 public:
  /// @brief Constructor for the Walker class
  /// @param name
  explicit Walker(std::string name);
  /// @brief Destructor for the Walker class
  ~Walker();
  /// @brief Callback function for the laser scan subscriber
  /// @param msg
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  /// @brief Move the turtlebot forward
  void moveForward();
  /// @brief  Rotate the turtlebot in the anticlockwise direction
  void rotate();
  /// @brief
  /// @param linear
  /// @param angular
  void setVelocity(float linear, float angular);
  /// @brief
  void stop();

 private:
  /// @brief
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  /// @brief
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  /// @brief
  geometry_msgs::msg::Twist vel_;
  /// @brief
  // cmd_vel publisher timer
  rclcpp::TimerBase::SharedPtr pub_timer_;
  /// @brief
  // rotate timer
  rclcpp::TimerBase::SharedPtr rotate_timer_;
  bool reverse_;
};
};  // namespace walker

#endif  // TURTLE_WALKER_INCLUDE_TURTLE_WALKER_CONTROLLER_HPP_
