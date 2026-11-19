// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2024 KIT
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

/*      Title     : servo_keyboard_input.cpp
 *      Project   : arm_api2
 *      Created   : 14/11/2024
 *      Author    : Guanqi Chen
 *      Description : Keyboard control code for Servoing
 */

// Code modified from https://github.com/moveit/moveit2_tutorials/blob/humble/doc/examples/realtime_servo/src/servo_keyboard_input.cpp

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/gripper_command.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_PLUS 0x2B
#define KEYCODE_MINUS 0x2D
#define KEYCODE_SPACE 0x20
#define KEYCODE_L 0x6C
#define KEYCODE_K 0x6B
#define KEYCODE_J 0x6A
#define KEYCODE_I 0x69
#define KEYCODE_M 0x6D
#define KEYCODE_COMMA 0x2C
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64

// Some constants used in the Servo Teleop demo
// Defaults (all overridable as ROS parameters). Relative names, so a robot namespace
// (__ns:=/ur1) applies: arm_api2 listens on ~/servo_twist_cmd of its node "moveit2_iface".
const char * DEFAULT_TWIST_TOPIC = "moveit2_iface/servo_twist_cmd";
const char * DEFAULT_GRIPPER_ACTION = "arm/gripper_control";
// Empty frame: arm_api2 uses its configured end-effector link.
const char * DEFAULT_EEF_FRAME = "";
const char * DEFAULT_BASE_FRAME = "base_link";
const size_t ROS_QUEUE_SIZE = 10;

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader()
  : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  void readOne(char * c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0) {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();
  void send_gripper_command(double position);

private:
  void spin();

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;

  std::string frame_to_publish_;
  std::string eef_frame_;
  std::string base_frame_;
  std::string gripper_action_;
  double vel_scale_;
};

KeyboardServo::KeyboardServo()
: vel_scale_(0.5)
{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");
  const auto twist_topic = nh_->declare_parameter<std::string>("twist_topic", DEFAULT_TWIST_TOPIC);
  gripper_action_ = nh_->declare_parameter<std::string>("gripper_action", DEFAULT_GRIPPER_ACTION);
  eef_frame_ = nh_->declare_parameter<std::string>("eef_frame", DEFAULT_EEF_FRAME);
  base_frame_ = nh_->declare_parameter<std::string>("base_frame", DEFAULT_BASE_FRAME);
  vel_scale_ = nh_->declare_parameter<double>("vel_scale", vel_scale_);
  frame_to_publish_ = base_frame_;
  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic, ROS_QUEUE_SIZE);
  gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(nh_,
    gripper_action_);
  RCLCPP_INFO(nh_->get_logger(), "Twist -> %s, gripper -> %s",
    twist_pub_->get_topic_name(), gripper_action_.c_str());
}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

void KeyboardServo::spin()
{
  while (rclcpp::ok()) {
    rclcpp::spin_some(nh_);
  }
}

int KeyboardServo::keyLoop()
{
  char c;
  bool publish_twist = false;
  std::thread{std::bind(&KeyboardServo::spin, this)}.detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
  puts(
    "Use 'I' and 'K' to pitch(y rotate), 'J' and 'L' to yaw(z rotate), and 'M' and ',' to roll(x rotate)");
  puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
  puts("Use '+' and '-' to increase/decrease the speed.");
  puts("Use 'A' to close the gripper, 'D' to open the gripper.");
  puts("Press Space to stop all motions (motion also stops as soon as keys are released).");
  puts("arm_api2 must be in SERVO_CTL (arm/change_state) for the arm to move.");
  puts("'Q' to quit.");
  puts("  ");

  for (;; ) {
    // get the next event from the keyboard
    try {
      input.readOne(&c);
    } catch (const std::runtime_error &) {
      perror("read():");
      return -1;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

    // // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();


    // Use read key-press
    switch (c) {
      // Cartesian motions
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        twist_msg->twist.linear.y = -1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        twist_msg->twist.linear.y = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        twist_msg->twist.linear.x = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        twist_msg->twist.linear.x = -1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_PERIOD:
        RCLCPP_DEBUG(nh_->get_logger(), "PERIOD");
        twist_msg->twist.linear.z = -1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_SEMICOLON:
        RCLCPP_DEBUG(nh_->get_logger(), "SEMICOLON");
        twist_msg->twist.linear.z = 1.0 * vel_scale_;
        publish_twist = true;
        break;

      // rotational motions
      case KEYCODE_J:
        RCLCPP_DEBUG(nh_->get_logger(), "J");
        twist_msg->twist.angular.z = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_L:
        RCLCPP_DEBUG(nh_->get_logger(), "L");
        twist_msg->twist.angular.z = -1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_I:
        RCLCPP_DEBUG(nh_->get_logger(), "I");
        twist_msg->twist.angular.y = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_K:
        RCLCPP_DEBUG(nh_->get_logger(), "K");
        twist_msg->twist.angular.y = -1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_M:
        RCLCPP_DEBUG(nh_->get_logger(), "M");
        twist_msg->twist.angular.x = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_COMMA:
        RCLCPP_DEBUG(nh_->get_logger(), "COMMA");
        twist_msg->twist.angular.x = -1.0 * vel_scale_;
        publish_twist = true;
        break;

      // Frame selection
      case KEYCODE_E:
        RCLCPP_DEBUG(nh_->get_logger(), "E");
        frame_to_publish_ = eef_frame_;
        break;
      case KEYCODE_W:
        RCLCPP_DEBUG(nh_->get_logger(), "W");
        frame_to_publish_ = base_frame_;
        break;

      // Stop actions
      case KEYCODE_Q:
        RCLCPP_INFO(nh_->get_logger(), "quit");
        return 0;
      case KEYCODE_SPACE:
        RCLCPP_INFO(nh_->get_logger(), "STOP");
        twist_msg->twist.linear.x = 0.0;
        twist_msg->twist.linear.y = 0.0;
        twist_msg->twist.linear.z = 0.0;
        twist_msg->twist.angular.x = 0.0;
        twist_msg->twist.angular.y = 0.0;
        twist_msg->twist.angular.z = 0.0;
        twist_pub_->publish(std::move(twist_msg));
        publish_twist = false;
        break;

      // Velocity control
      case KEYCODE_PLUS:
        RCLCPP_DEBUG(nh_->get_logger(), "PLUS");
        vel_scale_ += 0.01;
        RCLCPP_INFO(nh_->get_logger(), "Velocity scale: %f", vel_scale_);
        break;
      case KEYCODE_MINUS:
        RCLCPP_DEBUG(nh_->get_logger(), "MINUS");
        vel_scale_ -= 0.01;
        if (vel_scale_ < 0.01) {
          vel_scale_ = 0.01;
        }
        RCLCPP_INFO(nh_->get_logger(), "Velocity scale: %f", vel_scale_);
        break;

      // Gripper control
      case KEYCODE_A:
        RCLCPP_INFO(nh_->get_logger(), "close gripper");
        send_gripper_command(0.8); // close gripper
        break;
      case KEYCODE_D:
        RCLCPP_INFO(nh_->get_logger(), "open gripper");
        send_gripper_command(0.0); // open gripper
        break;
    }

    // If a key requiring a publish was pressed, publish the message now
    if (publish_twist) {
      twist_msg->header.stamp = nh_->now();
      twist_msg->header.frame_id = frame_to_publish_;
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
    }
  }
  return 0;
}

void KeyboardServo::send_gripper_command(double position)
{
  if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_WARN(nh_->get_logger(), "Gripper action %s not available", gripper_action_.c_str());
    return;
  }
  auto goal = control_msgs::action::GripperCommand::Goal();
  goal.command.position = position;
  goal.command.max_effort = 140.0;

  auto result = gripper_client_->async_send_goal(goal);
}
