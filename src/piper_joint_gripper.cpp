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

/** Reflect Robotiq-normalized axis used by arm_api2 open/close services (domain [0, 0.8]). */
double invert_robotiq_normalized(double normalized_robotiq)
{
  const double c = std::clamp(normalized_robotiq, 0.0, 0.8);
  return 0.8 - c;
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
  mirror_fjt_client_.reset();
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
    if (!cfg_.mirror_trajectory_action.empty()) {
      const std::string mirror_resolved = resolve_action_name(cfg_.mirror_trajectory_action);
      mirror_fjt_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(node_, mirror_resolved);
      RCLCPP_INFO(
        node_->get_logger(),
        "PiperJointGripper mirror trajectory client: %s joint=%s sign=%f",
        mirror_resolved.c_str(),
        cfg_.mirror_trajectory_joint_name.c_str(),
        cfg_.trajectory_mirror_sign);
    }
  }

  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    cfg_.state_topic,
    state_qos,
    std::bind(&PiperJointGripper::joint_state_cb, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "PiperJointGripper configured: mode=%s state=%s cmd=%s stroke_open=%f m stroke_close=%f m "
    "invert_robotiq_normalized=%s",
    (cfg_.command_mode ==
    PiperGripperCommandMode::JointState) ? "joint_state" : "follow_joint_trajectory",
    cfg_.state_topic.c_str(),
    (cfg_.command_mode == PiperGripperCommandMode::JointState) ? cfg_.cmd_topic.c_str() :
                                                                   cfg_.trajectory_action.c_str(),
    cfg_.open_stroke_m,
    cfg_.close_stroke_m,
    cfg_.invert_robotiq_normalized ? "true" : "false");
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

  const bool use_split_mirror =
    mirror_fjt_client_ && !cfg_.mirror_trajectory_joint_name.empty();

  if (use_split_mirror) {
    if (!fjt_client_->wait_for_action_server(std::chrono::seconds(8))) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "PiperJointGripper: FollowJointTrajectory server not ready (%s)",
        cfg_.trajectory_action.c_str());
      return false;
    }
    if (!mirror_fjt_client_->wait_for_action_server(std::chrono::seconds(8))) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "PiperJointGripper: mirror FollowJointTrajectory server not ready (%s)",
        cfg_.mirror_trajectory_action.c_str());
      return false;
    }

    std::atomic<bool> main_done{false};
    std::atomic<bool> main_success{false};
    std::atomic<bool> mirror_done{false};
    std::atomic<bool> mirror_success{false};

    const double mirror_stroke = cfg_.trajectory_mirror_sign * stroke_m;

    FollowJointTrajectory::Goal main_goal;
    main_goal.trajectory.header.stamp = node_->now();
    main_goal.trajectory.joint_names = {cfg_.trajectory_joint_name};
    trajectory_msgs::msg::JointTrajectoryPoint main_pt;
    main_pt.positions.push_back(stroke_m);
    const double T = std::max(0.05, cfg_.trajectory_time_from_start_sec);
    main_pt.time_from_start.sec = static_cast<int32_t>(std::trunc(T));
    main_pt.time_from_start.nanosec =
      static_cast<uint32_t>((T - main_pt.time_from_start.sec) * 1e9);
    main_goal.trajectory.points.push_back(std::move(main_pt));

    FollowJointTrajectory::Goal mirror_goal;
    mirror_goal.trajectory.header.stamp = node_->now();
    mirror_goal.trajectory.joint_names = {cfg_.mirror_trajectory_joint_name};
    trajectory_msgs::msg::JointTrajectoryPoint mirror_pt;
    mirror_pt.positions.push_back(mirror_stroke);
    mirror_pt.time_from_start.sec = static_cast<int32_t>(std::trunc(T));
    mirror_pt.time_from_start.nanosec =
      static_cast<uint32_t>((T - mirror_pt.time_from_start.sec) * 1e9);
    mirror_goal.trajectory.points.push_back(std::move(mirror_pt));

    main_done.store(false);
    main_success.store(false);
    mirror_done.store(false);
    mirror_success.store(false);

    auto make_opts = [this](
      std::atomic<bool> & done, std::atomic<bool> & success, const char * label) {
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions opts;
        opts.goal_response_callback = [this, label, &done, &success](
          std::shared_ptr<GoalHandleFj> goal_handle) {
            if (!goal_handle) {
              RCLCPP_ERROR(
            node_->get_logger(), "PiperJointGripper: %s trajectory goal rejected.", label);
              success.store(false);
              done.store(true, std::memory_order_release);
            }
          };
        opts.result_callback = [this, label, &done, &success](
          const GoalHandleFj::WrappedResult & result) {
            switch (result.code) {
              case rclcpp_action::ResultCode::SUCCEEDED:
                success.store(true);
                break;
              case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(
              node_->get_logger(), "PiperJointGripper: %s trajectory aborted (%s)", label,
              result.result ? result.result->error_string.c_str() : "");
                success.store(false);
                break;
              case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(node_->get_logger(), "PiperJointGripper: %s trajectory canceled.",
            label);
                success.store(false);
                break;
              default:
                success.store(false);
                break;
            }
            done.store(true, std::memory_order_release);
          };
        return opts;
      };

    fjt_client_->async_send_goal(main_goal, make_opts(main_done, main_success, "joint7"));
    mirror_fjt_client_->async_send_goal(
      mirror_goal, make_opts(mirror_done, mirror_success, "joint8"));

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(std::max(1.0, T + 8.0) * 1000.0));
    while (!main_done.load(std::memory_order_acquire) ||
      !mirror_done.load(std::memory_order_acquire))
    {
      if (std::chrono::steady_clock::now() > deadline) {
        RCLCPP_ERROR(node_->get_logger(), "PiperJointGripper: split trajectory result timed out.");
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return main_success.load() && mirror_success.load();
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
  // Drive the mirror finger in the same goal. `gripper_controller` typically
  // owns both prismatic joints and refuses partial-joint goals, so a
  // single-joint goal is rejected outright (gripper never actuates). Appending
  // the mirror joint (joint8 = -joint7 on Piper) makes the goal complete and
  // moves both fingers symmetrically.
  if (!cfg_.trajectory_mirror_joint_name.empty()) {
    goal.trajectory.joint_names.push_back(cfg_.trajectory_mirror_joint_name);
    pt.positions.push_back(cfg_.trajectory_mirror_sign * stroke_m);
  }
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

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int>(std::max(1.0, T + 8.0) * 1000.0));
  while (!traj_done_.load(std::memory_order_acquire)) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(node_->get_logger(), "PiperJointGripper: trajectory result timed out.");
      return false;
    }
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

  double n_cmd = normalized_position_robotiq;
  if (cfg_.invert_robotiq_normalized) {
    n_cmd = invert_robotiq_normalized(normalized_position_robotiq);
  }
  const double stroke = map_robotiq_to_stroke(n_cmd, cfg_.open_stroke_m, cfg_.close_stroke_m);

  bool ok = false;
  if (cfg_.command_mode == PiperGripperCommandMode::JointState) {
    ok = send_gripper_command_joint_state(n_cmd, max_effort);
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
    !have_state_ || idx == static_cast<size_t>(-1) || idx >= last_state_.position.size())
  {
    return last_cmd_normalized_.load();
  }
  const float stroke =
    static_cast<float>(last_state_.position[idx]);
  double r =
    map_stroke_to_robotiq(static_cast<double>(stroke), cfg_.open_stroke_m, cfg_.close_stroke_m);
  if (cfg_.invert_robotiq_normalized) {
    r = invert_robotiq_normalized(r);
  }
  return static_cast<float>(r);
}

float PiperJointGripper::get_effort() const
{
  std::scoped_lock lk(state_mtx_);
  const size_t idx = stroke_feedback_index(last_state_.name);
  if (
    !have_state_ || idx == static_cast<size_t>(-1) || idx >= last_state_.effort.size())
  {
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
