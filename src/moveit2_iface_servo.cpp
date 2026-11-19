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

/*      Title       : moveit2_iface_servo.cpp
 *      Project     : arm_api2
 *      Created     : 06/08/2025
 *      Author      : Filip Zoric
 *      Contributors: Edgar Welte
 *      Description : MoveIt Servo teleoperation and controller switching.
 */

#include "arm_api2/moveit2_iface.hpp"
#include "moveit2_iface_internal.hpp"
#include <algorithm>
#include <cstdio>
#include <limits>
#include <sstream>
#include <cctype>
#include <cmath>
#include <future>
#include <optional>
#include <vector>
#include <Eigen/Geometry>
#include <tf2/exceptions.hpp>

std::unique_ptr<moveit_servo::Servo> m2Iface::init_servo()
{
    // New Jazzy API - use ParamListener instead of ServoParameters
  servo_param_listener_ = std::make_shared<servo::ParamListener>(node_, "moveit_servo");
  auto servo_params = servo_param_listener_->get_params();
  RCLCPP_INFO_STREAM(this->get_logger(), "Servo move_group_name: " << servo_params.move_group_name);

  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_param_listener_,
    m_pSceneMonitorPtr);
  RCLCPP_INFO(this->get_logger(), "Servo initialized!");
  return servo;
}

void m2Iface::servo_twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_twist_cmd_ = *msg;
  new_twist_cmd_ = true;
}

void m2Iface::processServoCommand()
{
  if (!servoPtr || !new_twist_cmd_ || !m_moveGroupPtr) {return;}

    // Wait 0.5s after entering servo mode to avoid processing old buffered commands
  auto time_since_servo_entered = (this->now() - servo_entered_time_).seconds();
  if (time_since_servo_entered < 0.5) {
    new_twist_cmd_ = false;
    return;
  }

  new_twist_cmd_ = false;

    // Ignore commands with all zero velocities to prevent old buffered commands
  bool all_zero = (std::abs(latest_twist_cmd_.twist.linear.x) < 1e-6 &&
    std::abs(latest_twist_cmd_.twist.linear.y) < 1e-6 &&
    std::abs(latest_twist_cmd_.twist.linear.z) < 1e-6 &&
    std::abs(latest_twist_cmd_.twist.angular.x) < 1e-6 &&
    std::abs(latest_twist_cmd_.twist.angular.y) < 1e-6 &&
    std::abs(latest_twist_cmd_.twist.angular.z) < 1e-6);
  if (all_zero) {
    return;
  }

  try {
        // Create TwistCommand for servo
    moveit_servo::TwistCommand twist_cmd;
    twist_cmd.frame_id =
      latest_twist_cmd_.header.frame_id.empty() ? EE_LINK_NAME : latest_twist_cmd_.header.frame_id;
    twist_cmd.velocities[0] = latest_twist_cmd_.twist.linear.x;
    twist_cmd.velocities[1] = latest_twist_cmd_.twist.linear.y;
    twist_cmd.velocities[2] = latest_twist_cmd_.twist.linear.z;
    twist_cmd.velocities[3] = latest_twist_cmd_.twist.angular.x;
    twist_cmd.velocities[4] = latest_twist_cmd_.twist.angular.y;
    twist_cmd.velocities[5] = latest_twist_cmd_.twist.angular.z;

        // Get current robot state
    auto current_state = m_moveGroupPtr->getCurrentState(1.0);
    if (!current_state) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Could not get current robot state");
      return;
    }

        // Set command type
    servoPtr->setCommandType(moveit_servo::CommandType::TWIST);

        // Get next joint state from servo
    moveit_servo::KinematicState next_state = servoPtr->getNextJointState(current_state, twist_cmd);

        // Check servo status and publish it
    auto status = servoPtr->getStatus();
    auto status_msg_str = servoPtr->getStatusMessage();

        // Publish ServoStatus
    moveit_msgs::msg::ServoStatus status_msg;
    status_msg.code = static_cast<int8_t>(status);
    status_msg.message = status_msg_str;
    servo_status_pub_->publish(status_msg);

    if (status == moveit_servo::StatusCode::INVALID ||
      status == moveit_servo::StatusCode::HALT_FOR_SINGULARITY ||
      status == moveit_servo::StatusCode::HALT_FOR_COLLISION)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Servo status: %s", status_msg_str.c_str());
      return;
    }

        // Check if we got valid output
    if (next_state.joint_names.empty() || next_state.positions.size() == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Servo returned empty state");
      return;
    }

    if (servo_use_forward_position_ && servo_forward_position_pub_) {
            // Forward position controller consumes raw joint positions
      std_msgs::msg::Float64MultiArray command;
      for (Eigen::Index i = 0; i < next_state.positions.size(); ++i) {
        command.data.push_back(next_state.positions[i]);
      }
      servo_forward_position_pub_->publish(command);
    } else {
            // Create and publish trajectory
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.header.stamp = this->now();
      traj_msg.header.frame_id = PLANNING_FRAME;
      traj_msg.joint_names = next_state.joint_names;

      trajectory_msgs::msg::JointTrajectoryPoint point;
      for (Eigen::Index i = 0; i < next_state.positions.size(); ++i) {
        point.positions.push_back(next_state.positions[i]);
        if (i < next_state.velocities.size()) {
          point.velocities.push_back(next_state.velocities[i]);
        }
      }
      point.time_from_start = rclcpp::Duration::from_seconds(0.1);
      traj_msg.points.push_back(point);

      servo_trajectory_pub_->publish(traj_msg);
    }
    last_servo_state_ = next_state;

  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Servo command processing failed: %s", e.what());
  }
}

bool m2Iface::loadController(const std::string & controller_name)
{
  if (!load_controller_client_) {
    RCLCPP_ERROR(this->get_logger(), "load_controller client is not initialized");
    return false;
  }
  if (!load_controller_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "controller_manager/load_controller service not available");
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  request->name = controller_name;
  auto future = load_controller_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "load_controller('%s') timed out", controller_name.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    RCLCPP_WARN(
                        this->get_logger(),
            "load_controller('%s') returned false; continuing in case it is already loaded",
                        controller_name.c_str());
  }
  return true;
}

bool m2Iface::configureController(const std::string & controller_name)
{
  if (!configure_controller_client_) {
    RCLCPP_ERROR(this->get_logger(), "configure_controller client is not initialized");
    return false;
  }
  if (!configure_controller_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
      "controller_manager/configure_controller service not available");
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
  request->name = controller_name;
  auto future = configure_controller_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "configure_controller('%s') timed out",
                             controller_name.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    RCLCPP_WARN(
                        this->get_logger(),
            "configure_controller('%s') returned false; continuing in case it is already configured",
                        controller_name.c_str());
  }
  return true;
}

bool m2Iface::switchControllers(
  const std::vector<std::string> & activate,
  const std::vector<std::string> & deactivate)
{
  if (!switch_controller_client_) {
    RCLCPP_ERROR(this->get_logger(), "switch_controller client is not initialized");
    return false;
  }
  if (!switch_controller_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "controller_manager/switch_controller service not available");
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  request->activate_asap = true;
  request->timeout.sec = 2;
  request->timeout.nanosec = 0;

  auto future = switch_controller_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "switch_controller call timed out");
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    RCLCPP_ERROR(
                        this->get_logger(),
            "switch_controller failed (activate=%zu, deactivate=%zu)",
            activate.size(),
            deactivate.size());
    return false;
  }
  return true;
}

bool m2Iface::enterServoControllerMode()
{
  if (!servo_use_forward_position_) {
    return true;
  }
  if (forward_position_controller_active_) {
    return true;
  }
  (void)loadController(servo_forward_controller_);
  (void)configureController(servo_forward_controller_);
  const bool ok = switchControllers({servo_forward_controller_}, {servo_default_controller_});
  if (ok) {
    forward_position_controller_active_ = true;
    RCLCPP_INFO(this->get_logger(), "%s active for SERVO_CTL", servo_forward_controller_.c_str());
  }
  return ok;
}

bool m2Iface::leaveServoControllerMode()
{
  if (!servo_use_forward_position_) {
    return true;
  }
  if (!forward_position_controller_active_) {
    return true;
  }
  (void)loadController(servo_default_controller_);
  (void)configureController(servo_default_controller_);
  const bool ok = switchControllers({servo_default_controller_}, {servo_forward_controller_});
  if (ok) {
    forward_position_controller_active_ = false;
    RCLCPP_INFO(this->get_logger(), "%s restored", servo_default_controller_.c_str());
  }
  return ok;
}
