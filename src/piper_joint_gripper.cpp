/*******************************************************************************
 * Copyright (c) 2025, Crobotic Solutions d.o.o.
 * SPDX-License-Identifier: BSD-3-Clause
 ******************************************************************************/

#include "arm_api2/grippers/piper_joint_gripper.hpp"

#include <cmath>

#include <algorithm>
#include <chrono>
#include <functional>
#include <thread>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{
double map_robotiq_to_stroke(double normalized_robotiq, double open_m, double close_m)
{
  double t = std::clamp(normalized_robotiq / 0.8, 0.0, 1.0);
  return open_m + t * (close_m - open_m);
}

double map_stroke_to_robotiq(double stroke_m, double open_m, double close_m)
{
  double span = close_m - open_m;
  if (std::abs(span) < 1e-9) {
    return 0.0;
  }
  double t = (stroke_m - open_m) / span;
  t = std::clamp(t, 0.0, 1.0);
  return t * 0.8;
}

double effort_to_piper_effort_axis(double robotiq_effort_nominal_max_140ish)
{
  // Piper driver clips effort[6] to [0.5, 3] (N·m scale in bridge trajectory).
  (void)robotiq_effort_nominal_max_140ish;
  return 2.0;
}

size_t find_gripper_index(const std::vector<std::string> & names)
{
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i] == "gripper") {
      return i;
    }
  }
  if (names.size() == 7) {
    return 6;
  }
  return static_cast<size_t>(-1);
}
}  // namespace

PiperJointGripper::PiperJointGripper(std::shared_ptr<rclcpp::Node> node)
: node_(std::move(node))
{
}

std::string PiperJointGripper::resolve_action_name(const std::string & action) const
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

size_t PiperJointGripper::stroke_feedback_index(const std::vector<std::string> & names) const
{
  auto find_name = [&](const std::string & n) -> size_t {
    for (size_t i = 0; i < names.size(); ++i) {
      if (names[i] == n) {
        return i;
      }
    }
    return static_cast<size_t>(-1);
  };

  if (!cfg_.trajectory_joint_name.empty()) {
    const size_t ix = find_name(cfg_.trajectory_joint_name);
    if (ix != static_cast<size_t>(-1)) {
      return ix;
    }
  }

  const size_t g = find_name("gripper");
  if (g != static_cast<size_t>(-1)) {
    return g;
  }

  if (names.size() >= 7) {
    return 6;
  }

  return static_cast<size_t>(-1);
}

void PiperJointGripper::configure(const PiperJointGripperConfig & config)
{
  cfg_ = config;
  configured_ = true;
  state_sub_.reset();
  cmd_pub_.reset();
  fjt_client_.reset();
  have_state_ = false;

  rclcpp::QoS cmd_qos{rclcpp::KeepLast{10}};
  cmd_qos.reliable();
  cmd_qos.durability_volatile();

  rclcpp::QoS state_qos{rclcpp::KeepLast{50}};
  state_qos.reliable();
  state_qos.durability_volatile();

  if (cfg_.command_mode == PiperGripperCommandMode::JointState) {
    cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      cfg_.cmd_topic, cmd_qos);
  } else {
    const std::string resolved = resolve_action_name(cfg_.trajectory_action);
    fjt_client_ = rclcpp_action::create_client<FollowJointTrajectory>(node_, resolved);
    RCLCPP_INFO(
      node_->get_logger(),
      "PiperJointGripper trajectory client: %s joint=%s",
      resolved.c_str(), cfg_.trajectory_joint_name.c_str());
  }

  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    cfg_.state_topic,
    state_qos,
    std::bind(&PiperJointGripper::joint_state_cb, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "PiperJointGripper configured: mode=%s state=%s cmd=%s stroke_open=%f m stroke_close=%f m",
    (cfg_.command_mode == PiperGripperCommandMode::JointState) ? "joint_state" : "follow_joint_trajectory",
    cfg_.state_topic.c_str(),
    (cfg_.command_mode == PiperGripperCommandMode::JointState) ? cfg_.cmd_topic.c_str()
                                                                 : cfg_.trajectory_action.c_str(),
    cfg_.open_stroke_m,
    cfg_.close_stroke_m);
}

void PiperJointGripper::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg || msg->name.empty()) {
    return;
  }
  std::scoped_lock lk(state_mtx_);
  last_state_ = *msg;
  have_state_ = true;
}

bool PiperJointGripper::try_wait_for_feedback(std::chrono::milliseconds total_wait) const
{
  auto step = std::chrono::milliseconds(20);
  auto waited = std::chrono::milliseconds(0);
  while (waited < total_wait) {
    std::this_thread::sleep_for(step);
    waited += step;
    {
      std::scoped_lock lk(state_mtx_);
      if (have_state_) {
        return true;
      }
    }
  }
  return false;
}

bool PiperJointGripper::send_gripper_command_trajectory(double stroke_m, double max_effort)
{
  (void)max_effort;
  if (!fjt_client_) {
    RCLCPP_ERROR(node_->get_logger(), "PiperJointGripper: trajectory action client not created.");
    return false;
  }
  if (!fjt_client_->wait_for_action_server(std::chrono::seconds(8))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "PiperJointGripper: FollowJointTrajectory server not ready (%s)",
      cfg_.trajectory_action.c_str());
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.header.stamp = node_->now();
  goal.trajectory.joint_names = {cfg_.trajectory_joint_name};
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions.push_back(stroke_m);
  // ros2_joint_trajectory_controller rejects goals whose points include `effort` when
  // it does not use effort interpolation (typical Piper sim + ros2_control). Piper’s
  // hardware bridge defaults effort when omitted.
  const double T = std::max(0.05, cfg_.trajectory_time_from_start_sec);
  pt.time_from_start.sec = static_cast<int32_t>(std::trunc(T));
  pt.time_from_start.nanosec = static_cast<uint32_t>((T - pt.time_from_start.sec) * 1e9);
  goal.trajectory.points.push_back(std::move(pt));

  traj_done_.store(false);
  traj_success_.store(false);

  auto opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  opts.goal_response_callback = [this](std::shared_ptr<GoalHandleFj> goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "PiperJointGripper: gripper trajectory goal rejected.");
      traj_success_.store(false);
      traj_done_.store(true, std::memory_order_release);
    }
  };

  opts.result_callback = [this](const GoalHandleFj::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        traj_success_.store(true);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "PiperJointGripper: trajectory aborted (%s)",
          result.result ? result.result->error_string.c_str() : "");
        traj_success_.store(false);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(node_->get_logger(), "PiperJointGripper: trajectory canceled.");
        traj_success_.store(false);
        break;
      default:
        traj_success_.store(false);
        break;
    }
    traj_done_.store(true, std::memory_order_release);
  };

  fjt_client_->async_send_goal(goal, opts);

  while (!traj_done_.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return traj_success_.load();
}

bool PiperJointGripper::send_gripper_command_joint_state(
  double normalized_position_robotiq, double max_effort)
{
  if (!cmd_pub_) {
    RCLCPP_ERROR(node_->get_logger(), "PiperJointGripper: JointState publisher not created.");
    return false;
  }

  sensor_msgs::msg::JointState cmd_msg;
  {
    std::scoped_lock lk(state_mtx_);
    cmd_msg.header.stamp = node_->now();
    cmd_msg.name = last_state_.name;
    cmd_msg.position = last_state_.position;
    cmd_msg.velocity.clear();
    cmd_msg.effort = last_state_.effort;
  }

  if (cmd_msg.name.empty() || cmd_msg.position.size() != cmd_msg.name.size()) {
    RCLCPP_ERROR(node_->get_logger(), "PiperJointGripper: invalid cached JointState (names/size).");
    return false;
  }

  const size_t idx = find_gripper_index(cmd_msg.name);
  if (idx == static_cast<size_t>(-1) || idx >= cmd_msg.position.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "PiperJointGripper: JointState cmd path needs `gripper` joint (index 6) in names.");
    return false;
  }

  const double stroke =
    map_robotiq_to_stroke(normalized_position_robotiq, cfg_.open_stroke_m, cfg_.close_stroke_m);

  cmd_msg.position[idx] = stroke;

  if (cmd_msg.effort.size() < cmd_msg.name.size()) {
    cmd_msg.effort.resize(cmd_msg.name.size(), 0.0);
  }
  cmd_msg.effort[idx] = effort_to_piper_effort_axis(max_effort);

  cmd_pub_->publish(cmd_msg);
  return true;
}

bool PiperJointGripper::send_gripper_command(double normalized_position_robotiq, double max_effort)
{
  if (!configured_) {
    RCLCPP_ERROR(node_->get_logger(), "PiperJointGripper::configure() was not called.");
    return false;
  }

  if (!have_state_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "PiperJointGripper: no JointState feedback yet on %s; waiting...",
      cfg_.state_topic.c_str());
    if (!try_wait_for_feedback(std::chrono::milliseconds(2500))) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "PiperJointGripper: timed out waiting for JointState feedback on %s",
        cfg_.state_topic.c_str());
      return false;
    }
  }

  const double stroke =
    map_robotiq_to_stroke(normalized_position_robotiq, cfg_.open_stroke_m, cfg_.close_stroke_m);

  bool ok = false;
  if (cfg_.command_mode == PiperGripperCommandMode::JointState) {
    ok = send_gripper_command_joint_state(normalized_position_robotiq, max_effort);
  } else {
    ok = send_gripper_command_trajectory(stroke, max_effort);
  }

  if (ok) {
    last_cmd_normalized_.store(static_cast<float>(normalized_position_robotiq));
    last_effort_.store(static_cast<float>(max_effort));
  }
  return ok;
}

float PiperJointGripper::get_position() const
{
  std::scoped_lock lk(state_mtx_);
  const size_t idx = stroke_feedback_index(last_state_.name);
  if (
    !have_state_ || idx == static_cast<size_t>(-1) || idx >= last_state_.position.size()) {
    return last_cmd_normalized_.load();
  }
  const float stroke =
    static_cast<float>(last_state_.position[idx]);
  return static_cast<float>(
    map_stroke_to_robotiq(static_cast<double>(stroke), cfg_.open_stroke_m, cfg_.close_stroke_m));
}

float PiperJointGripper::get_effort() const
{
  std::scoped_lock lk(state_mtx_);
  const size_t idx = stroke_feedback_index(last_state_.name);
  if (
    !have_state_ || idx == static_cast<size_t>(-1) || idx >= last_state_.effort.size()) {
    return last_effort_.load();
  }
  return static_cast<float>(last_state_.effort[idx]);
}

bool PiperJointGripper::is_stalled() const
{
  return false;
}

bool PiperJointGripper::reached_goal() const
{
  return true;
}
