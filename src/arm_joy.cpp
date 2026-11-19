// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2024-2026 Crobotic Solutions d.o.o.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
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

/*      Title       : arm_joy.cpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : Joystick control code.
 */

#include "arm_api2/arm_joy.hpp"

#include <algorithm>
// Axis/button mapping and topic are ROS parameters (defaults: original mapping).
// arm_api2 consumes twists on <ns>/moveit2_iface/servo_twist_cmd in SERVO_CTL; each message
// is one servo step, so the arm stops as soon as messages stop (no watchdog needed).

JoyCtl::JoyCtl()
: Node("joy_ctl")
{
  init();

  setScaleFactor(1);
  enableJoy_ = true;


}

void JoyCtl::init()
{
  const auto topic = this->declare_parameter<std::string>("twist_topic",
                                                                "moveit2_iface/servo_twist_cmd");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "");
  axis_x_ = this->declare_parameter<int>("axis_x", 4);
  axis_y_ = this->declare_parameter<int>("axis_y", 3);
  axis_z_ = this->declare_parameter<int>("axis_z", 7);
  axis_yaw_ = this->declare_parameter<int>("axis_yaw", 6);
  axis_scale_ = this->declare_parameter<int>("axis_scale", 1);
  deadman_button_ = this->declare_parameter<int>("deadman_button", 5);
  step_ = this->declare_parameter<double>("step", 0.01);

  cmdVelPub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(topic, 1);
  joySub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
                                                                   std::bind(&JoyCtl::joy_callback,
    this, _1));

  RCLCPP_INFO(this->get_logger(), "Initialized joy_ctl -> %s (hold button %d to move)",
                    cmdVelPub_->get_topic_name(), deadman_button_);
}

void JoyCtl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  const int max_axis = std::max({axis_x_, axis_y_, axis_z_, axis_yaw_, axis_scale_});
  if (static_cast<int>(msg->axes.size()) <= max_axis ||
    static_cast<int>(msg->buttons.size()) <= deadman_button_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock_, 5000,
                                     "Joy message has %zu axes / %zu buttons; mapping needs axis %d and button %d",
                                     msg->axes.size(), msg->buttons.size(), max_axis,
      deadman_button_);
    return;
  }
  const bool enabled = msg->buttons.at(deadman_button_) == 1;
  if (enabled != was_enabled_) {
    RCLCPP_INFO(this->get_logger(), enabled ? "ON" : "OFF");
  }
  setEnableJoy(enabled);

  int sF = getScaleFactor();
  if (msg->axes.at(axis_scale_) == 1 || msg->axes.at(axis_scale_) == -1) {
    sF += msg->axes.at(axis_scale_) > 0 ? 1 : -1;
    sF = std::clamp(sF, 1, 99);
    RCLCPP_INFO_STREAM(this->get_logger(), "Scale factor: " << sF);
  }
  setScaleFactor(sF);

        // arm_api2 ignores all-zero twists, so only motion is sent; one zero on release is
        // harmless and documents the stop in a bag.
  if (!enabled && !was_enabled_) {return;}
  was_enabled_ = enabled;

  auto teleop_msg = geometry_msgs::msg::TwistStamped();
  teleop_msg.header.stamp = this->get_clock()->now();
  teleop_msg.header.frame_id = frame_id_;
  if (enabled) {
    const double k = sF * step_;
    teleop_msg.twist.linear.z = msg->axes.at(axis_x_) * k;
    teleop_msg.twist.linear.y = msg->axes.at(axis_y_) * k;
    teleop_msg.twist.linear.x = -msg->axes.at(axis_z_) * k;
    teleop_msg.twist.angular.z = msg->axes.at(axis_yaw_) * k;
  }
  cmdVelPub_->publish(teleop_msg);
}

void JoyCtl::setScaleFactor(int value)
{
  scale_factor = value;
}

int JoyCtl::getScaleFactor() const
{
  return scale_factor;
}

void JoyCtl::setEnableJoy(bool val)
{
  enableJoy_ = val;
}

bool JoyCtl::getEnableJoy() const
{
  return enableJoy_;
}
