/*******************************************************************************
 * Copyright (c) 2025, Crobotic Solutions d.o.o.
 * SPDX-License-Identifier: BSD-3-Clause
 *******************************************************************************/

#include "arm_api2/grippers/robotiq_gripper.hpp"

#include <chrono>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

namespace
{
double first_joint_position(const sensor_msgs::msg::JointState & js)
{
  if (!js.position.empty()) {
    return static_cast<double>(js.position[0]);
  }
  return 0.0;
}
}  // namespace

RobotiqGripper::RobotiqGripper(std::shared_ptr<rclcpp::Node> node)
: node_(std::move(node))
{
}

void RobotiqGripper::configure(const RobotiqGripperConfig & config)
{
  config_ = config;
  configured_ = true;
  legacy_client_.reset();
  parallel_client_.reset();
  const std::string resolved = resolve_action_name(config_.backend_action);
  if (config_.backend == RobotiqGripperConfig::BackendKind::ParallelGripperCommand) {
    parallel_client_ = rclcpp_action::create_client<control_msgs::action::ParallelGripperCommand>(
      node_, resolved);
  } else {
    legacy_client_ =
      rclcpp_action::create_client<control_msgs::action::GripperCommand>(node_, resolved);
  }
  RCLCPP_INFO(
    node_->get_logger(), "RobotiqGripper configured: backend=%s action=%s joint=%s",
    (config_.backend == RobotiqGripperConfig::BackendKind::ParallelGripperCommand) ? "parallel"
                                                                                     : "legacy",
    resolved.c_str(), config_.parallel_joint_name.c_str());
}

std::string RobotiqGripper::resolve_action_name(const std::string & action) const
{
  if (action.empty()) {
    return action;
  }
  if (action.front() == '/') {
    return action;
  }
  std::string ns = node_->get_namespace();
  if (ns.empty()) {
    return std::string("/") + action;
  }
  if (ns.back() == '/') {
    return ns + action;
  }
  return ns + "/" + action;
}

void RobotiqGripper::open()
{
  send_gripper_command(0.0);
  RCLCPP_INFO(node_->get_logger(), "Gripper opened.");
  isOpen = true;
}

void RobotiqGripper::close()
{
  send_gripper_command(0.8);
  RCLCPP_INFO(node_->get_logger(), "Gripper closed.");
  isOpen = false;
}

bool RobotiqGripper::send_gripper_command(double position, double max_effort)
{
  if (!configured_) {
    RCLCPP_ERROR(node_->get_logger(), "RobotiqGripper::configure() was not called.");
    return false;
  }
  if (config_.backend == RobotiqGripperConfig::BackendKind::ParallelGripperCommand) {
    return send_gripper_command_parallel(position, max_effort);
  }
  return send_gripper_command_legacy(position, max_effort);
}

bool RobotiqGripper::send_gripper_command_legacy(double position, double max_effort)
{
  if (!legacy_client_) {
    RCLCPP_ERROR(node_->get_logger(), "Legacy gripper action client is null.");
    return false;
  }
  if (!legacy_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available!");
    return false;
  }
  auto goal = control_msgs::action::GripperCommand::Goal();
  goal.command.position = position;
  goal.command.max_effort = max_effort;

  RCLCPP_INFO(node_->get_logger(), "Sending gripper command (legacy) ...");
  using GoalHandle = rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>;
  auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](GoalHandle::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  };
  send_goal_options.feedback_callback =
    [this](GoalHandle::SharedPtr goal_handle,
           const std::shared_ptr<const control_msgs::action::GripperCommand::Feedback> feedback) {
      (void)goal_handle;
      RCLCPP_INFO(
        node_->get_logger(), "Got feedback: position = %f, effort = %f, stalled = %d, reached_goal = %d",
        feedback->position, feedback->effort, feedback->stalled, feedback->reached_goal);
    };
  send_goal_options.result_callback = [this](const GoalHandle::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");
        if (result.result) {
          RCLCPP_INFO(
            node_->get_logger(), "Result: position = %f, effort = %f, stalled = %d, reached_goal = %d",
            result.result->position, result.result->effort, result.result->stalled,
            result.result->reached_goal);
          last_position = static_cast<float>(result.result->position);
          last_effort = static_cast<float>(result.result->effort);
          last_stalled = result.result->stalled;
          last_reached_goal = result.result->reached_goal;
        }
        success = true;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
        success = false;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
        success = false;
        break;
      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
        break;
    }
    is_done = true;
  };

  legacy_client_->async_send_goal(goal, send_goal_options);

  is_done = false;
  while (!is_done) {
  }

  RCLCPP_INFO(node_->get_logger(), "Gripper moved to position %f", last_position);
  return success;
}

bool RobotiqGripper::send_gripper_command_parallel(double position, double max_effort)
{
  if (!parallel_client_) {
    RCLCPP_ERROR(node_->get_logger(), "Parallel gripper action client is null.");
    return false;
  }
  if (!parallel_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Parallel gripper action server not available!");
    return false;
  }

  control_msgs::action::ParallelGripperCommand::Goal goal;
  goal.command.name = {config_.parallel_joint_name};
  goal.command.position = {position};
  goal.command.velocity = {};
  goal.command.effort = {max_effort};

  RCLCPP_INFO(node_->get_logger(), "Sending gripper command (parallel) ...");
  using GoalHandle = rclcpp_action::ClientGoalHandle<control_msgs::action::ParallelGripperCommand>;
  auto send_goal_options =
    rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](GoalHandle::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Parallel gripper goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Parallel gripper goal accepted, waiting for result");
    }
  };
  send_goal_options.feedback_callback =
    [this](GoalHandle::SharedPtr goal_handle,
           const std::shared_ptr<const control_msgs::action::ParallelGripperCommand::Feedback> feedback) {
      (void)goal_handle;
      const double pos = first_joint_position(feedback->state);
      RCLCPP_INFO(node_->get_logger(), "Parallel gripper feedback: joint0 position = %f", pos);
    };
  send_goal_options.result_callback = [this](const GoalHandle::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "Parallel gripper goal succeeded!");
        if (result.result) {
          last_position = static_cast<float>(first_joint_position(result.result->state));
          last_effort = 0.0F;
          if (!result.result->state.effort.empty()) {
            last_effort = static_cast<float>(result.result->state.effort[0]);
          }
          last_stalled = result.result->stalled;
          last_reached_goal = result.result->reached_goal;
          RCLCPP_INFO(
            node_->get_logger(), "Result: pos=%f stalled=%d reached_goal=%d", last_position, last_stalled,
            last_reached_goal);
        }
        success = true;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "Parallel gripper goal was aborted");
        success = false;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_->get_logger(), "Parallel gripper goal was canceled");
        success = false;
        break;
      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown parallel gripper result code");
        break;
    }
    is_done = true;
  };

  parallel_client_->async_send_goal(goal, send_goal_options);

  is_done = false;
  while (!is_done) {
  }

  RCLCPP_INFO(node_->get_logger(), "Gripper moved to position %f", last_position);
  return success;
}

float RobotiqGripper::get_position()
{
  return last_position;
}

float RobotiqGripper::get_effort()
{
  return last_effort;
}

bool RobotiqGripper::is_stalled()
{
  return last_stalled;
}

bool RobotiqGripper::reached_goal()
{
  return last_reached_goal;
}
