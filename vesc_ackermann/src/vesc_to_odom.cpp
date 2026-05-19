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

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <cmath>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  odom_frame_("odom"),
  base_frame_("base_link"),
  use_servo_cmd_(true),
  publish_tf_(false),
  publish_actuator_feedback_(false),
  x_(0.0),
  y_(0.0),
  yaw_(0.0),
  use_imu_yaw_rate_(false),
  imu_yaw_rate_(0.0),
  imu_yaw_invert_(false),
  last_imu_stamp_(0, 0, RCL_ROS_TIME),
  imu_stamp_initialized_(false),
  prev_feedback_speed_(0.0),
  prev_feedback_steering_(0.0),
  prev_feedback_accel_(0.0),
  prev_feedback_stamp_(0, 0, RCL_ROS_TIME),
  feedback_initialized_(false)
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);

  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain");
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset");

  if (use_servo_cmd_) {
    steering_to_servo_gain_ =
      declare_parameter<double>("steering_angle_to_servo_gain");
    steering_to_servo_offset_ =
      declare_parameter<double>("steering_angle_to_servo_offset");
    wheelbase_ = declare_parameter<double>("wheelbase");
  }

  publish_tf_ = declare_parameter("publish_tf", publish_tf_);
  publish_actuator_feedback_ = declare_parameter("publish_actuator_feedback", false);
  use_imu_yaw_rate_ = declare_parameter("use_imu_yaw_rate", false);
  imu_yaw_invert_ = declare_parameter("imu_yaw_invert", false);

  // configurable odometry covariance diagonals
  auto pose_cov = declare_parameter<std::vector<double>>(
    "odom_pose_covariance_diagonal", {0.05, 0.05, 1e9, 1e9, 1e9, 0.1});
  auto twist_cov = declare_parameter<std::vector<double>>(
    "odom_twist_covariance_diagonal", {0.01, 0.1, 1e9, 1e9, 1e9, 0.05});
  for (size_t i = 0; i < 6; ++i) {
    odom_pose_cov_diag_[i] = pose_cov[i];
    odom_twist_cov_diag_[i] = twist_cov[i];
  }

  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
  qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);

  rclcpp::QoS sensor_qos(rclcpp::KeepLast(1));
  sensor_qos.best_effort().durability_volatile();

  // create odom publisher and, optionally, actuator feedback publisher
  odom_pub_ = create_publisher<Odometry>("odom", sensor_qos);
  if (publish_actuator_feedback_) {
    feedback_pub_ = create_publisher<AckermannDriveStamped>("sensors/actuator_feedback", qos_profile);
  }

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  // subscribe to vesc state and, optionally, servo command
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", sensor_qos, std::bind(&VescToOdom::vescStateCallback, this, _1));

  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", sensor_qos,
      std::bind(&VescToOdom::servoCmdCallback, this, _1));
  }

  if (use_imu_yaw_rate_) {
    imu_sub_ = create_subscription<Imu>(
      "sensors/imu/raw", sensor_qos,
      std::bind(&VescToOdom::imuCallback, this, _1));
  }
}

void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (use_servo_cmd_ && !last_servo_cmd_) {
    return;
  }

  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  if (use_servo_cmd_) {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_) {
    last_state_ = state;
  }

  // calc elapsed time
  auto dt = rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp);

  // select yaw rate (used for twist report; IMU path integrates yaw_ in imuCallback)
  double yaw_rate = use_imu_yaw_rate_ ? imu_yaw_rate_ : current_angular_velocity;

  // propagate odometry
  double x_dot = current_speed * cos(yaw_);
  double y_dot = current_speed * sin(yaw_);
  x_ += x_dot * dt.seconds();
  y_ += y_dot * dt.seconds();
  if (!use_imu_yaw_rate_ && use_servo_cmd_) {
    yaw_ += yaw_rate * dt.seconds();
  }

  // save state for next time
  last_state_ = state;

  // publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = state->header.stamp;
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

  // Pose covariance diagonal (indices 0,7,14,21,28,35 = x,y,z,roll,pitch,yaw)
  const int cov_diag_idx[6] = {0, 7, 14, 21, 28, 35};
  for (int i = 0; i < 6; ++i) {
    odom.pose.covariance[cov_diag_idx[i]] = odom_pose_cov_diag_[i];
    odom.twist.covariance[cov_diag_idx[i]] = odom_twist_cov_diag_[i];
  }

  // Velocity in child_frame_id
  odom.twist.twist.linear.x = current_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = yaw_rate;

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = state->header.stamp;  // match odom stamp (BUG-U3)
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }

  if (publish_actuator_feedback_) {
    double fb_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    double fb_steer = use_servo_cmd_ ?
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_ :
      0.0;

    AckermannDriveStamped fb_msg;
    fb_msg.header.stamp = state->header.stamp;
    fb_msg.header.frame_id = base_frame_;
    fb_msg.drive.speed = fb_speed;
    fb_msg.drive.steering_angle = fb_steer;

    if (feedback_initialized_) {
      double fdt = (rclcpp::Time(state->header.stamp) - prev_feedback_stamp_).seconds();
      if (fdt > 0.0) {
        double accel = (fb_speed - prev_feedback_speed_) / fdt;
        double steer_rate = (fb_steer - prev_feedback_steering_) / fdt;
        double jerk = (accel - prev_feedback_accel_) / fdt;
        fb_msg.drive.acceleration = accel;
        fb_msg.drive.steering_angle_velocity = steer_rate;
        fb_msg.drive.jerk = jerk;
        prev_feedback_accel_ = accel;
      }
    }
    prev_feedback_speed_ = fb_speed;
    prev_feedback_steering_ = fb_steer;
    prev_feedback_stamp_ = state->header.stamp;
    feedback_initialized_ = true;
    feedback_pub_->publish(fb_msg);
  }
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

void VescToOdom::imuCallback(const Imu::SharedPtr imu)
{
  imu_yaw_rate_ = imu_yaw_invert_ ? -imu->angular_velocity.z : imu->angular_velocity.z;

  if (!imu_stamp_initialized_) {
    last_imu_stamp_ = imu->header.stamp;
    imu_stamp_initialized_ = true;
    return;
  }

  const double dt = (rclcpp::Time(imu->header.stamp) - last_imu_stamp_).seconds();
  last_imu_stamp_ = imu->header.stamp;

  if (dt > 0.0 && dt < 1.0) {
    yaw_ += imu_yaw_rate_ * dt;
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
