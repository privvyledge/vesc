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

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options),
  cmd_accel_(0.0),
  cmd_steering_rate_(0.0),
  cmd_jerk_(0.0),
  use_closed_loop_(false),
  actual_erpm_(0.0),
  integral_(0.0),
  ff_gain_(0.0),
  kp_(0.0),
  ki_(0.0),
  anti_windup_(1000.0),
  prev_cmd_time_(0, 0, RCL_ROS_TIME),
  use_adaptive_ff_(false),
  adaptive_ff_alpha_(0.95),
  ff_gain_min_(0.0),
  ff_gain_max_(0.0)
{
  // get conversion parameters
  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain");
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset");
  steering_to_servo_gain_ =
    declare_parameter<double>("steering_angle_to_servo_gain");
  steering_to_servo_offset_ =
    declare_parameter<double>("steering_angle_to_servo_offset");

  // closed-loop speed PI params (4E)
  use_closed_loop_ = declare_parameter("use_closed_loop_speed", false);
  kp_ = declare_parameter("speed_kp", 0.0);
  ki_ = declare_parameter("speed_ki", 0.0);
  anti_windup_ = declare_parameter("speed_anti_windup", 1000.0);

  // adaptive feedforward params (4E)
  use_adaptive_ff_ = declare_parameter("use_adaptive_ff", false);
  adaptive_ff_alpha_ = declare_parameter("adaptive_ff_alpha", 0.95);
  ff_gain_ = speed_to_erpm_gain_;
  ff_gain_min_ = declare_parameter("adaptive_ff_gain_min", speed_to_erpm_gain_ * 0.5);
  ff_gain_max_ = declare_parameter("adaptive_ff_gain_max", speed_to_erpm_gain_ * 2.0);

  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
  qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);

  rclcpp::QoS sensor_qos(rclcpp::KeepLast(1));
  sensor_qos.best_effort().durability_volatile();

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", qos_profile);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", qos_profile);

  // subscribe to ackermann topic (BUG-U2: use qos_profile instead of integer depth)
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", qos_profile, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

  // conditionally subscribe for closed-loop / adaptive-ff feedback (4E)
  if (use_closed_loop_ || use_adaptive_ff_) {
    state_sub_ = create_subscription<VescStateStamped>(
      "sensors/core", sensor_qos, std::bind(&AckermannToVesc::vescStateCallback, this, _1));
  }
  if (use_adaptive_ff_) {
    filtered_odom_sub_ = create_subscription<Odometry>(
      "odom/filtered", sensor_qos,
      std::bind(&AckermannToVesc::filteredOdomCallback, this, _1));
  }

  prev_cmd_time_ = get_clock()->now();
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // store higher-order derivatives (4C)
  cmd_accel_ = cmd->drive.acceleration;
  cmd_steering_rate_ = cmd->drive.steering_angle_velocity;
  cmd_jerk_ = cmd->drive.jerk;
  if (cmd_accel_ != 0.0 || cmd_steering_rate_ != 0.0 || cmd_jerk_ != 0.0) {
    RCLCPP_DEBUG(
      get_logger(), "higher-order cmd: accel=%.3f steer_rate=%.3f jerk=%.3f",
      cmd_accel_, cmd_steering_rate_, cmd_jerk_);
  }

  // calc vesc electric RPM (speed)
  double erpm = (use_adaptive_ff_ ? ff_gain_ : speed_to_erpm_gain_) *
    cmd->drive.speed + speed_to_erpm_offset_;

  // PI closed-loop correction (4E)
  if (use_closed_loop_) {
    auto now = get_clock()->now();
    double dt = (now - prev_cmd_time_).seconds();
    prev_cmd_time_ = now;
    if (dt > 0.0 && dt < 1.0) {
      double error = erpm - actual_erpm_;
      integral_ += error * dt;
      integral_ = std::clamp(integral_, -anti_windup_, anti_windup_);
      erpm += kp_ * error + ki_ * integral_;
    }
  }

  Float64 erpm_msg;
  erpm_msg.data = erpm;

  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // publish
  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
  }
}

void AckermannToVesc::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  actual_erpm_ = state->state.speed;
}

void AckermannToVesc::filteredOdomCallback(const Odometry::SharedPtr odom)
{
  double actual_speed = odom->twist.twist.linear.x;
  if (std::fabs(actual_speed) > 0.1) {
    double measured_gain = actual_erpm_ / actual_speed;
    ff_gain_ = adaptive_ff_alpha_ * ff_gain_ + (1.0 - adaptive_ff_alpha_) * measured_gain;
    ff_gain_ = std::clamp(ff_gain_, ff_gain_min_, ff_gain_max_);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
