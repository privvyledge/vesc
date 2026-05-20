// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
#define VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

class AckermannToVesc : public rclcpp::Node
{
public:
  explicit AckermannToVesc(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;

  // last commanded higher-order derivatives (4C)
  double cmd_accel_;
  double cmd_steering_rate_;
  double cmd_jerk_;

  // closed-loop speed PI control (4E)
  bool use_closed_loop_;
  double actual_erpm_;
  double integral_;
  double ff_gain_;
  double kp_;
  double ki_;
  double anti_windup_;
  rclcpp::Time prev_cmd_time_;

  // adaptive feedforward (4E)
  bool use_adaptive_ff_;
  double adaptive_ff_alpha_;
  double ff_gain_min_;
  double ff_gain_max_;

  // acceleration feedforward (4G)
  bool use_accel_ff_;
  double accel_to_erpm_gain_;
  bool use_cmd_accel_rate_limit_;

  // input saturation / rate limiting (4F); 0 = disabled
  double max_speed_;
  double max_steering_angle_;
  double max_accel_;
  double max_steering_rate_;
  double prev_cmd_speed_;
  double prev_cmd_steering_;
  bool cmd_initialized_;

  // ROS services
  rclcpp::Publisher<Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr state_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr filtered_odom_sub_;

  // ROS callbacks
  void ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd);
  void vescStateCallback(const VescStateStamped::SharedPtr state);
  void filteredOdomCallback(const Odometry::SharedPtr odom);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
