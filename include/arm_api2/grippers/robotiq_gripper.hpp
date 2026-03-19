/*******************************************************************************
*
 * Copyright (c) 2024, Crobotic Solutions d.o.o. (www.crobotics.tech)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef ROBOTIQ_GRIPPER_H
#define ROBOTIQ_GRIPPER_H

#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <memory>
#include <string>

#include "gripper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

struct RobotiqGripperConfig
{
  enum class BackendKind
  {
    LegacyGripperCommand,
    ParallelGripperCommand,
  };

  /// Action base name or fully-qualified action (leading '/' skips namespace prefixing).
  std::string backend_action{"robotiq_2f_urcap_adapter/gripper_command"};
  BackendKind backend{RobotiqGripperConfig::BackendKind::LegacyGripperCommand};
  /// Joint name for ParallelGripperCommand goals (must match ros2_control / URDF).
  std::string parallel_joint_name{"robotiq_85_left_knuckle_joint"};
};

class RobotiqGripper : public Gripper
{
public:
  explicit RobotiqGripper(std::shared_ptr<rclcpp::Node> node);

  void configure(const RobotiqGripperConfig & config);

  void open() override;

  void close() override;

  bool send_gripper_command(double position, double max_effort = 140.0);

  float get_position();

  float get_effort();

  bool is_stalled();

  bool reached_goal();

  ~RobotiqGripper() override = default;

private:
  std::string resolve_action_name(const std::string & action) const;

  bool send_gripper_command_legacy(double position, double max_effort);

  bool send_gripper_command_parallel(double position, double max_effort);

  std::shared_ptr<rclcpp::Node> node_;
  bool isOpen{false};

  float last_position{0.0F};
  float last_effort{0.0F};
  bool last_stalled{false};
  bool last_reached_goal{false};
  bool success{false};
  bool is_done{false};

  RobotiqGripperConfig config_;
  bool configured_{false};

  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr legacy_client_;
  rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SharedPtr parallel_client_;
};

#endif  // ROBOTIQ_GRIPPER_H
