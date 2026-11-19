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

/*      Title       : moveit2_iface_execution.cpp
 *      Project     : arm_api2
 *      Created     : 06/08/2025
 *      Author      : Filip Zoric
 *      Contributors: Edgar Welte
 *      Description : Trajectory execution through move_group ExecuteTrajectory: start/cancel/stop, progress, goal check, path deviation watchdog.
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

bool m2Iface::latestJoints(std::unordered_map<std::string, double> & out, double max_age_s)
{
  std::lock_guard<std::mutex> lock(exec_js_mutex_);
  if (exec_js_.empty()) {return false;}
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - exec_js_time_).count();
  if (age > max_age_s) {return false;}
  out = exec_js_;
  return true;
}

void m2Iface::onExecutionSucceeded(uint64_t id, const std::string & moveit_code)
{
  std::unordered_map<std::string, double> js;
  const bool have = latestJoints(js, 1.0);
  double err = 0.0;
  std::string joint;
  bool measured = false;
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);
    if (active_exec_.id != id) {return;}
    const auto & t = active_exec_.traj;
    if (have && !t.points.empty()) {
      measured = jointError(js, t.joint_names, t.points.back().positions, err, joint);
    }
    if (measured) {
      active_exec_.final_error = err;
      active_exec_.final_error_joint = joint;
    }
    if (goal_joint_tolerance_rad_ > 0.0 && measured && err > goal_joint_tolerance_rad_) {
      // the controller may still be settling: execProgressTick re-checks until the deadline
      active_exec_.verifying = true;
      active_exec_.result_code = moveit_code;
      active_exec_.verify_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(goal_settle_sec_));
      return;
    }
  }
  std::string code = moveit_code;
  if (measured) {
    code += ", final joint error " + fmtRad(err) + " rad (" + joint + ")";
  } else {
    code += ", final joint error not measured (no fresh joint_states)";
    if (goal_joint_tolerance_rad_ > 0.0) {
      RCLCPP_WARN(this->get_logger(), "Goal tolerance check skipped: no fresh joint_states");
    }
  }
  finishExecution(id, true, code);
}

void m2Iface::init_execution()
{
  exec_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  stop_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Same name MoveGroupInterface uses: <move_group_ns>/execute_trajectory
  std::string ns = MOVE_GROUP_NS;
  while (!ns.empty() && ns.back() == '/') {ns.pop_back();}
  const std::string exec_action = ns.empty() ? "execute_trajectory" : ns + "/execute_trajectory";
  exec_client_ = rclcpp_action::create_client<ExecTraj>(node_, exec_action, exec_cb_group_);
  exec_progress_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
      [this]() {
        execProgressTick();
        }, exec_cb_group_);

  exec_js_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions js_opts;
  js_opts.callback_group = exec_js_cb_group_;
  exec_js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    config["topic"]["sub"]["joint_states"]["name"].as<std::string>(), 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(exec_js_mutex_);
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        exec_js_[msg->name[i]] = msg->position[i];
      }
      exec_js_time_ = std::chrono::steady_clock::now();
    }, js_opts);

  std::string stop_name = "arm/stop";
  if (config["srv"]["stop"] && config["srv"]["stop"]["name"]) {
    stop_name = config["srv"]["stop"]["name"].as<std::string>();
  }
  stop_srv_ = node_->create_service<std_srvs::srv::Trigger>(stop_name,
                                                                  std::bind(&m2Iface::stop_cb, this,
    _1, _2), rclcpp::ServicesQoS(), stop_cb_group_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Execution via action '" << exec_action <<
                           "', stop service '" << stop_name << "', async_actions: " <<
    async_actions_ <<
                           ", on state change while moving: " <<
    (stop_on_state_change_ ? "stop" : "reject") <<
                           ", goal tolerance " << goal_joint_tolerance_rad_ <<
    " rad, path deviation limit " <<
    path_deviation_max_rad_ << " rad (0 = off)");
}

bool m2Iface::startExecution(
  const moveit_msgs::msg::RobotTrajectory & trajectory, ExecDone on_done,
  ExecProgress on_progress, const void * owner)
{
  if (trajectory.joint_trajectory.points.empty()) {
    RCLCPP_WARN(this->get_logger(), "startExecution: empty trajectory, not executing");
    return false;
  }
  bool expected = false;
  if (!trajectory_executing_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Trajectory already executing; not starting another one.");
    return false;
  }
  if (!exec_client_ || !exec_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "ExecuteTrajectory action server not available");
    trajectory_executing_.store(false);
    return false;
  }

  const auto & last = trajectory.joint_trajectory.points.back().time_from_start;
  MotionRecord rec = takeMotionForExecution();
  uint64_t id = 0;
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);
    id = ++exec_seq_;
    active_exec_ = ActiveExec();
    active_exec_.id = id;
    active_exec_.owner = owner;
    active_exec_.start = std::chrono::steady_clock::now();
    active_exec_.planned_duration = rclcpp::Duration(last).seconds();
    active_exec_.on_done = std::move(on_done);
    active_exec_.on_progress = std::move(on_progress);
    active_exec_.traj = trajectory.joint_trajectory;
    active_exec_.record = std::move(rec);
  }

  ExecTraj::Goal goal;
  goal.trajectory = trajectory;
  auto opts = rclcpp_action::Client<ExecTraj>::SendGoalOptions();
  opts.goal_response_callback =
    [this, id](const rclcpp_action::ClientGoalHandle<ExecTraj>::SharedPtr & gh) {
      if (!gh) {
        RCLCPP_ERROR(this->get_logger(), "move_group rejected the ExecuteTrajectory goal");
        finishExecution(id, false, "GOAL_REJECTED");
        return;
      }
      bool cancel_now = false;
      {
        std::lock_guard<std::mutex> lock(exec_mutex_);
        if (active_exec_.id != id) {return;}
        active_exec_.handle = gh;
        cancel_now = active_exec_.cancel_requested && !active_exec_.cancel_sent;
        if (cancel_now) {active_exec_.cancel_sent = true;}
      }
      if (cancel_now) {exec_client_->async_cancel_goal(gh);}
    };
  opts.result_callback =
    [this, id](const rclcpp_action::ClientGoalHandle<ExecTraj>::WrappedResult & r) {
      std::string moveit_code = "NO_RESULT";
      bool moveit_ok = false;
      if (r.result) {
        const moveit::core::MoveItErrorCode ec(r.result->error_code);
        moveit_code = utils::moveItErrorCodeToString(ec);
        moveit_ok = ec == moveit::core::MoveItErrorCode::SUCCESS;
      }
      switch (r.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (moveit_ok) {
            onExecutionSucceeded(id, moveit_code);
          } else {
            finishExecution(id, false, moveit_code);
          }
          break;
        case rclcpp_action::ResultCode::CANCELED:
          finishExecution(id, false, "CANCELED (" + moveit_code + ")");
          break;
        default:
          finishExecution(id, false, "ABORTED (" + moveit_code + ")");
          break;
      }
    };
  exec_client_->async_send_goal(goal, opts);
  RCLCPP_INFO(this->get_logger(), "Execution %lu sent (%zu points, %.2f s planned)",
    static_cast<unsigned long>(id), trajectory.joint_trajectory.points.size(),
    rclcpp::Duration(last).seconds());
  return true;
}

bool m2Iface::runExecutionBlocking(
  const moveit_msgs::msg::RobotTrajectory & trajectory, ExecProgress on_progress,
  const void * owner, std::string & code)
{
  auto prom = std::make_shared<std::promise<std::pair<bool, std::string>>>();
  auto fut = prom->get_future();
  if (!startExecution(trajectory,
    [prom](bool ok, const std::string & c) {
      prom->set_value({ok, c});
        },
      std::move(on_progress), owner))
  {
    code = "NOT_STARTED";
    return false;
  }
  while (fut.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (!rclcpp::ok()) {
      cancelExecution("shutdown");
      code = "SHUTDOWN";
      return false;
    }
  }
  const auto r = fut.get();
  code = r.second;
  return r.first;
}

bool m2Iface::cancelExecution(const std::string & why, const void * owner)
{
  rclcpp_action::ClientGoalHandle<ExecTraj>::SharedPtr gh;
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);
    if (active_exec_.id == 0) {return false;}
    if (owner && active_exec_.owner != owner) {return false;}
    if (!active_exec_.cancel_requested) {
      active_exec_.cancel_requested = true;
      active_exec_.cancel_reason = why;
    }
    if (active_exec_.handle && !active_exec_.cancel_sent) {
      active_exec_.cancel_sent = true;
      gh = active_exec_.handle;
    }
  }
  RCLCPP_WARN_STREAM(this->get_logger(), "Stopping execution: " << why);
  // Without a handle yet the goal response callback sends the cancel.
  if (gh) {exec_client_->async_cancel_goal(gh);}
  // move_group (Jazzy) does not stop on an ExecuteTrajectory cancel (mock test 2026-11-13:
  // the goal ran to the end); the trajectory_execution_event "stop" does, within ~0.05 s.
  // While our goal is active move_group is executing exactly that trajectory.
  if (m_moveGroupPtr) {m_moveGroupPtr->stop();}
  return true;
}

void m2Iface::finishExecution(uint64_t id, bool ok, std::string code)
{
  ExecDone done;
  MotionRecord rec;
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);
    if (id == 0 || active_exec_.id != id) {return;}
    done = std::move(active_exec_.on_done);
    if (!ok && active_exec_.cancel_requested) {
      code = "STOPPED: " + active_exec_.cancel_reason + " (" + code + ")";
    }
    if (!active_exec_.max_deviation_joint.empty()) {
      code += ", max path deviation " + fmtRad(active_exec_.max_deviation) + " rad (" +
        active_exec_.max_deviation_joint + ")";
    }
    rec = std::move(active_exec_.record);
    rec.executed = true;
    rec.actual_duration_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - active_exec_.start).count();
    rec.planned_duration_sec = active_exec_.planned_duration;
    rec.final_joint_error = active_exec_.final_error;
    rec.final_error_joint = active_exec_.final_error_joint;
    if (!active_exec_.max_deviation_joint.empty()) {
      rec.max_path_deviation = active_exec_.max_deviation;
      rec.max_deviation_joint = active_exec_.max_deviation_joint;
    }
    rec.success = ok;
    rec.code = execErrorCode(ok, code);
    rec.reason = code;
    exec_stats_.has_last = true;
    exec_stats_.last_ok = ok;
    exec_stats_.last_code = rec.code;
    exec_stats_.last_reason = code;
    exec_stats_.last_time = std::chrono::steady_clock::now();
    ++(ok ? exec_stats_.succeeded : exec_stats_.failed);
    active_exec_ = ActiveExec();
  }
  writeMotion(rec);
  trajectory_executing_.store(false, std::memory_order_release);
  if (ok) {
    RCLCPP_INFO(this->get_logger(), "Execution %lu finished: %s",
      static_cast<unsigned long>(id), code.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Execution %lu did not complete: %s",
      static_cast<unsigned long>(id), code.c_str());
  }
  if (done) {done(ok, code);}
}

void m2Iface::execProgressTick()
{
  uint64_t timeout_id = 0;
  std::string timeout_code;
  rclcpp_action::ClientGoalHandle<ExecTraj>::SharedPtr cancel_gh;
  uint64_t verify_id = 0;
  bool verify_ok = false;
  std::string verify_code;
  std::string deviation_stop;
  std::unordered_map<std::string, double> js;
  const bool have_js = latestJoints(js, 0.3);
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);
    if (active_exec_.id == 0) {return;}
    if (active_exec_.verifying) {
      double err = 0.0;
      std::string joint;
      const bool measured = have_js && !active_exec_.traj.points.empty() &&
        jointError(js, active_exec_.traj.joint_names,
                                                         active_exec_.traj.points.back().positions,
        err, joint);
      if (measured) {
        active_exec_.final_error = err;
        active_exec_.final_error_joint = joint;
      }
      const auto check = arm_api2::logic::goalCheck(measured, err, goal_joint_tolerance_rad_,
                                                                      std::chrono::steady_clock::now()
        >
          active_exec_.verify_deadline);
      if (check == arm_api2::logic::GoalCheck::PASS) {
        verify_id = active_exec_.id;
        verify_ok = true;
        verify_code = active_exec_.result_code + ", final joint error " + fmtRad(err) +
          " rad (" + joint + ") after settling";
      } else if (check == arm_api2::logic::GoalCheck::FAIL) {
        verify_id = active_exec_.id;
        verify_code = "GOAL_NOT_REACHED: final joint error " +
          (measured ? fmtRad(err) + " rad (" + joint + ")" : std::string("not measured")) +
          " > tolerance " + fmtRad(goal_joint_tolerance_rad_) + " rad after " +
          fmtRad(goal_settle_sec_) + " s (move_group reported " + active_exec_.result_code + ")";
      }
    } else if (active_exec_.handle && have_js) {
      double dev = 0.0;
      std::string joint;
      if (pathDeviation(js, active_exec_.traj, dev, joint)) {
        if (dev > active_exec_.max_deviation || active_exec_.max_deviation_joint.empty()) {
          active_exec_.max_deviation = dev;
          active_exec_.max_deviation_joint = joint;
        }
        if (path_deviation_max_rad_ > 0.0 && dev > path_deviation_max_rad_ &&
          !active_exec_.cancel_requested)
        {
          deviation_stop = "PATH_DEVIATION " + fmtRad(dev) + " rad (" + joint + ") > " +
            fmtRad(path_deviation_max_rad_) + " rad";
          active_exec_.cancel_requested = true;
          active_exec_.cancel_reason = deviation_stop;
          active_exec_.cancel_sent = true;
          cancel_gh = active_exec_.handle;
        }
      }
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - active_exec_.start).count();
    const double total = active_exec_.planned_duration;
    // Called under the lock so no feedback is published after the goal finished.
    if (active_exec_.on_progress) {
      try {
        active_exec_.on_progress(elapsed, total);
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Progress feedback failed: %s", e.what());
      }
    }
    // Never stay "executing" forever when move_group does not answer
    // (died, or execution_duration_monitoring disabled on a stuck controller).
    const auto wd = arm_api2::logic::execWatchdog(static_cast<bool>(active_exec_.handle),
                                                              elapsed, total);
    if (wd == arm_api2::logic::Watchdog::NO_GOAL_RESPONSE) {
      timeout_id = active_exec_.id;
      timeout_code = "NO_GOAL_RESPONSE";
    } else if (wd == arm_api2::logic::Watchdog::RESULT_TIMEOUT) {
      timeout_id = active_exec_.id;
      timeout_code = "RESULT_TIMEOUT";
      if (!active_exec_.cancel_sent) {
        active_exec_.cancel_sent = true;
        cancel_gh = active_exec_.handle;
      }
    }
  }
  if (!deviation_stop.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Stopping execution: " << deviation_stop);
    if (cancel_gh) {exec_client_->async_cancel_goal(cancel_gh);}
    if (m_moveGroupPtr) {m_moveGroupPtr->stop();}
    return;
  }
  if (verify_id != 0) {
    finishExecution(verify_id, verify_ok, verify_code);
    return;
  }
  if (timeout_id != 0) {
    RCLCPP_ERROR(this->get_logger(), "Execution %lu: %s, giving up",
      static_cast<unsigned long>(timeout_id), timeout_code.c_str());
    if (cancel_gh) {exec_client_->async_cancel_goal(cancel_gh);}
    if (m_moveGroupPtr) {m_moveGroupPtr->stop();}
    finishExecution(timeout_id, false, timeout_code);
  }
}

void m2Iface::stop_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  const bool ours = cancelExecution("arm/stop service");
  // Also stops trajectories other clients sent to move_group (e.g. weld ExecuteTrajectory).
  if (m_moveGroupPtr) {m_moveGroupPtr->stop();}
  stop_requested_.store(true);
  res->success = true;
  res->message = ours ?
    "arm_api2 execution canceled; MoveIt stop sent" :
    "no arm_api2 execution active; MoveIt stop sent";
  RCLCPP_WARN_STREAM(this->get_logger(), "arm/stop: " << res->message);
}

bool m2Iface::execPlan_with_plan(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  bool async_flag, ExecDone on_done)
{
  return execTrajectory(plan.trajectory, async_flag, std::move(on_done));
}

bool m2Iface::execTrajectory(
  moveit_msgs::msg::RobotTrajectory trajectory, bool async_flag,
  ExecDone on_done)
{
  if (async_flag) {
    return startExecution(trajectory, std::move(on_done));
  }
  std::string code;
  const bool ok = runExecutionBlocking(trajectory, nullptr, nullptr, code);
  if (on_done) {on_done(ok, code);}
  return ok;
}

void m2Iface::stopBeforeAsyncExecute()
{
  if (!m_moveGroupPtr) {
    return;
  }
  m_moveGroupPtr->stop();
    /* Allow the trajectory execution action to settle; short sleeps alone were not always enough. */
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
}
