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

/*      Title       : moveit2_iface_observability.cpp
 *      Project     : arm_api2
 *      Description : Permanent motion log (JSONL, one line per motion) and /diagnostics
 *                    (move_group, planning scene monitor, joint_states, execution, gripper,
 *                    controllers, arm state).
 */

#include "arm_api2/moveit2_iface.hpp"
#include "moveit2_iface_internal.hpp"

#include <cstdlib>

using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
std::string defaultMotionLogDir()
{
  if (const char * e = std::getenv("ARM_API2_MOTION_LOG_DIR"); e && *e) {return e;}
  if (const char * r = std::getenv("ROS_HOME"); r && *r) {
    return std::string(r) + "/arm_api2/motion_log";
  }
  const char * h = std::getenv("HOME");
  return std::string(h && *h ? h : ".") + "/.ros/arm_api2/motion_log";
}

double secondsSince(std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}
}  // namespace

/* ------------------------------------------------------------------ motion log */

void m2Iface::init_motion_log()
{
  // automatically_declare_parameters_from_overrides: read, never declare (declare throws)
  std::string dir;
  this->get_parameter("motion_log_dir", dir);
  if (dir == "none" || dir == "off" || dir == "false") {
    RCLCPP_INFO(this->get_logger(), "Motion log disabled (motion_log_dir: %s)", dir.c_str());
    return;
  }
  if (dir.empty()) {dir = defaultMotionLogDir();}
  std::string tag = ns_;
  tag.erase(0, tag.find_first_not_of('/'));
  for (auto & c : tag) {
    if (c == '/') {
      c = '_';
    }
  }
  if (tag.empty()) {tag = "arm";}
  std::string err;
  if (!motion_log_.open(dir, tag, err)) {
    RCLCPP_ERROR(this->get_logger(), "Motion log disabled: %s", err.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Motion log: %s", motion_log_.currentPath().c_str());
}

void m2Iface::motionBegin(const std::string & command, const std::string & source)
{
  MotionRecord r;
  r.active = true;
  r.time = arm_api2::isoTimeNow();
  r.robot = ns_;
  r.command = command;
  r.source = source;
  r.mode = stateNames[robotState];
  r.frame = PLANNING_FRAME;
  r.vel_scale = max_vel_scaling_factor;
  r.acc_scale = max_acc_scaling_factor;
  std::unordered_map<std::string, double> js;
  if (m_moveGroupPtr && latestJoints(js, 1.0)) {
    for (const auto & name : m_moveGroupPtr->getJointNames()) {
      const auto it = js.find(name);
      if (it == js.end()) {r.joint_names.clear(); r.start_joints.clear(); break;}
      r.joint_names.push_back(name);
      r.start_joints.push_back(it->second);
    }
  }
  last_planner_.clear();
  std::lock_guard<std::mutex> lock(motion_mutex_);
  motion_ = std::move(r);
  motion_owner_ = std::this_thread::get_id();
}

void m2Iface::motionPlanned(
  const std::string & planner, double plan_time_sec, double fraction,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const geometry_msgs::msg::PoseStamped * target)
{
  std::lock_guard<std::mutex> lock(motion_mutex_);
  if (!motion_.active || motion_owner_ != std::this_thread::get_id()) {return;}
  motion_.planner = planner;
  motion_.plan_time_sec = plan_time_sec;
  motion_.fraction = fraction;
  const auto & jt = trajectory.joint_trajectory;
  motion_.planned_points = static_cast<int>(jt.points.size());
  if (!jt.points.empty()) {
    motion_.planned_duration_sec = rclcpp::Duration(jt.points.back().time_from_start).seconds();
    motion_.goal_joints = jt.points.back().positions;
    if (motion_.joint_names != jt.joint_names) {
                        // keep start/goal in the same joint order as the plan
      std::vector<double> start;
      for (const auto & n : jt.joint_names) {
        const auto it = std::find(motion_.joint_names.begin(), motion_.joint_names.end(), n);
        if (it == motion_.joint_names.end()) {start.clear(); break;}
        start.push_back(motion_.start_joints[it - motion_.joint_names.begin()]);
      }
      motion_.start_joints = start;
      motion_.joint_names = jt.joint_names;
    }
  }
  if (target) {
    motion_.has_target_pose = true;
    const auto & p = target->pose;
    const double v[7] = {p.position.x, p.position.y, p.position.z,
      p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
    std::copy(v, v + 7, motion_.target_pose);
    if (!target->header.frame_id.empty()) {motion_.frame = target->header.frame_id;}
  }
}

void m2Iface::motionStatus(
  bool success, const std::string & code, const std::string & reason,
  const geometry_msgs::msg::PoseStamped & pose, double plan_time_sec)
{
  MotionRecord r;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (!motion_.active || motion_owner_ != std::this_thread::get_id()) {return;}
    r = std::move(motion_);
    motion_ = MotionRecord();
  }
  r.success = success;
  r.code = code;
  r.reason = reason;
  if (std::isnan(r.plan_time_sec) && plan_time_sec > 0.0) {r.plan_time_sec = plan_time_sec;}
  if (!r.has_target_pose && r.command != "move_to_joint") {
    r.has_target_pose = true;
    const auto & p = pose.pose;
    const double v[7] = {p.position.x, p.position.y, p.position.z,
      p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
    std::copy(v, v + 7, r.target_pose);
  }
  writeMotion(r);
}

MotionRecord m2Iface::takeMotionForExecution()
{
  std::lock_guard<std::mutex> lock(motion_mutex_);
  MotionRecord r;
  if (motion_.active && motion_owner_ == std::this_thread::get_id()) {
    r = std::move(motion_);
    motion_ = MotionRecord();
  } else {
                // execution without a planning context (should not happen): still log it
    r.time = arm_api2::isoTimeNow();
    r.robot = ns_;
    r.command = "execute";
    r.mode = stateNames[robotState];
  }
  r.active = true;
  return r;
}

void m2Iface::writeMotion(MotionRecord & r)
{
  if (!r.active) {return;}
  r.active = false;
  if (motion_log_.enabled() && !motion_log_.write(r)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                     "Motion log write failed (%s)",
      motion_log_.currentPath().c_str());
  }
}

/* ------------------------------------------------------------------ diagnostics */

void m2Iface::init_diagnostics()
{
  diag_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  list_controllers_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
                "controller_manager/list_controllers", rclcpp::ServicesQoS(), diag_cb_group_);

  diag_updater_ = std::make_shared<diagnostic_updater::Updater>(node_, 1.0);
  std::string hw = ns_ == "/" ? PLANNING_GROUP : ns_ + "/" + PLANNING_GROUP;
  diag_updater_->setHardwareID(hw);
  diag_updater_->add("move_group", this, &m2Iface::diagMoveGroup);
  diag_updater_->add("planning_scene_monitor", this, &m2Iface::diagSceneMonitor);
  diag_updater_->add("joint_states", this, &m2Iface::diagJointStates);
  diag_updater_->add("execution", this, &m2Iface::diagExecution);
  diag_updater_->add("gripper", this, &m2Iface::diagGripper);
  diag_updater_->add("controllers", this, &m2Iface::diagControllers);
  diag_updater_->add("arm_state", this, &m2Iface::diagArmState);
  RCLCPP_INFO(this->get_logger(), "Diagnostics on /diagnostics (hardware_id %s)", hw.c_str());
}

void m2Iface::diagMoveGroup(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const bool iface = moveGroupInit && m_moveGroupPtr;
  const bool exec_ready = exec_client_ && exec_client_->action_server_is_ready();
  stat.add("move_group_interface", iface ? "ready" : "not initialized");
  stat.add("execute_trajectory_action", exec_ready ? "available" : "not available");
  stat.add("move_group_ns", MOVE_GROUP_NS.empty() ? "/" : MOVE_GROUP_NS);
  if (iface) {
    stat.add("planning_group", PLANNING_GROUP);
    stat.add("planning_frame", PLANNING_FRAME);
    stat.add("ee_link", EE_LINK_NAME);
  }
  if (!iface) {
    stat.summary(DiagnosticStatus::ERROR, "MoveGroupInterface not initialized");
  } else if (!exec_ready) {
    stat.summary(DiagnosticStatus::ERROR, "move_group execute_trajectory action not available");
  } else {
    stat.summary(DiagnosticStatus::OK, "move_group connected");
  }
}

void m2Iface::diagSceneMonitor(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!m_pSceneMonitorPtr) {
    stat.summary(DiagnosticStatus::ERROR, "planning scene monitor not created");
    return;
  }
  const auto sm = m_pSceneMonitorPtr->getStateMonitor();
  if (!sm) {
    stat.summary(DiagnosticStatus::ERROR, "state monitor not started");
    return;
  }
  std::vector<std::string> missing;
  const bool complete = sm->haveCompleteState(missing);
  stat.add("complete_state", complete ? "yes" : "no");
  if (!missing.empty()) {
    std::string m;
    for (const auto & j : missing) {
      m += (m.empty() ? "" : ", ") + j;
    }
    stat.add("missing_joints", m);
  }
  try {
    // the monitor stamps with its own clock type; compare against the same clock
    const rclcpp::Time t = m_pSceneMonitorPtr->getLastUpdateTime();
    if (t.nanoseconds() == 0) {
      stat.add("last_scene_update_age_sec", "never");
    } else {
      stat.addf("last_scene_update_age_sec", "%.2f",
        (rclcpp::Clock(t.get_clock_type()).now() - t).seconds());
    }
  } catch (const std::exception & e) {
    stat.add("last_scene_update_age_sec", std::string("n/a: ") + e.what());
  }
  if (!complete) {
    stat.summary(DiagnosticStatus::ERROR, "robot state incomplete (missing joints)");
  } else {
    stat.summary(DiagnosticStatus::OK, "planning scene monitor running");
  }
}

void m2Iface::diagJointStates(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  size_t n = 0;
  double age = -1.0;
  {
    std::lock_guard<std::mutex> lock(exec_js_mutex_);
    n = exec_js_.size();
    if (n > 0) {age = secondsSince(exec_js_time_);}
  }
  stat.add("topic", config["topic"]["sub"]["joint_states"]["name"].as<std::string>());
  stat.add("joints", static_cast<int>(n));
  if (n == 0) {
    stat.summary(DiagnosticStatus::ERROR, "no joint_states received");
    return;
  }
  stat.addf("age_sec", "%.3f", age);
  if (age > 2.0) {
    stat.summary(DiagnosticStatus::ERROR, "joint_states stale");
  } else if (age > 0.5) {
    stat.summary(DiagnosticStatus::WARN, "joint_states delayed");
  } else {
    stat.summary(DiagnosticStatus::OK, "joint_states fresh");
  }
}

void m2Iface::diagExecution(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(exec_mutex_);
  stat.add("succeeded", static_cast<int>(exec_stats_.succeeded));
  stat.add("failed", static_cast<int>(exec_stats_.failed));
  stat.addf("goal_joint_tolerance_rad", "%g", goal_joint_tolerance_rad_);
  stat.addf("path_deviation_max_rad", "%g", path_deviation_max_rad_);
  stat.add("motion_log", motion_log_.enabled() ? motion_log_.currentPath() : "disabled");
  if (exec_stats_.has_last) {
    stat.add("last_result", exec_stats_.last_code);
    stat.add("last_reason", exec_stats_.last_reason);
    stat.addf("last_result_age_sec", "%.1f", secondsSince(exec_stats_.last_time));
  }
  if (active_exec_.id != 0) {
    const double elapsed = secondsSince(active_exec_.start);
    stat.add("execution_id", static_cast<int>(active_exec_.id));
    stat.addf("elapsed_sec", "%.2f", elapsed);
    stat.addf("planned_duration_sec", "%.2f", active_exec_.planned_duration);
    if (!active_exec_.max_deviation_joint.empty()) {
      stat.addf("max_path_deviation_rad", "%.4f", active_exec_.max_deviation);
    }
    const char * phase = active_exec_.verifying ? "verifying goal" :
      active_exec_.cancel_requested ? "stopping" : "executing";
    stat.summaryf(DiagnosticStatus::OK, "%s (%.1f / %.1f s)", phase, elapsed,
                              active_exec_.planned_duration);
    return;
  }
  if (exec_stats_.has_last && !exec_stats_.last_ok) {
                // stays WARN until the next successful execution
    stat.summaryf(DiagnosticStatus::WARN, "idle; last execution: %s",
                              exec_stats_.last_code.c_str());
  } else {
    stat.summary(DiagnosticStatus::OK, exec_stats_.has_last ? "idle; last execution: SUCCESS" :
                             "idle");
  }
}

void m2Iface::diagGripper(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (gripper_none_) {
    stat.add("type", "none");
    stat.summary(DiagnosticStatus::OK, "no gripper on this robot/tool");
    return;
  }
  stat.add("type", piper_joint_gripper_ ? "piper_joint" : "robotiq/gripper_command");
  stat.addf("position_normalized", "%.3f", gripperMeasuredPositionNormalized());
  stat.addf("effort", "%.3f", gripperMeasuredEffort());
  stat.add("stalled", gripperMeasuredStalled() ? "yes" : "no");
  stat.summary(DiagnosticStatus::OK, "gripper configured");
}

void m2Iface::diagControllers(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(diag_mutex_);
        // refresh asynchronously: the result is shown on the next update
  if (list_controllers_client_ && !diag_controllers_pending_ &&
    list_controllers_client_->service_is_ready())
  {
    diag_controllers_pending_ = true;
    auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    list_controllers_client_->async_send_request(req,
      [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture f) {
        std::lock_guard<std::mutex> l(diag_mutex_);
        diag_controllers_pending_ = false;
        try {
          diag_controllers_ = f.get()->controller;
          diag_controllers_valid_ = true;
          diag_controllers_time_ = std::chrono::steady_clock::now();
        } catch (const std::exception &) {
          diag_controllers_valid_ = false;
        }
                });
  }
  if (!diag_controllers_valid_ || secondsSince(diag_controllers_time_) > 5.0) {
                // a request that never returns must not block refreshing forever
    if (diag_controllers_pending_ && diag_controllers_valid_ &&
      secondsSince(diag_controllers_time_) > 10.0)
    {
      diag_controllers_pending_ = false;
    }
    stat.summary(DiagnosticStatus::ERROR,
                             "controller_manager/list_controllers not answering");
    return;
  }
  int active = 0;
  bool trajectory_active = false;
  for (const auto & c : diag_controllers_) {
    stat.add(c.name, c.state + " (" + c.type + ")");
    if (c.state == "active") {
      ++active;
      if (c.type.find("JointTrajectoryController") != std::string::npos) {
        trajectory_active = true;
      }
    }
  }
  const bool servo_mode = robotState == SERVO_CTL;
  if (active == 0) {
    stat.summary(DiagnosticStatus::ERROR, "no active controllers");
  } else if (!trajectory_active && !servo_mode) {
    stat.summaryf(DiagnosticStatus::WARN, "%d active, no JointTrajectoryController active",
                              active);
  } else {
    stat.summaryf(DiagnosticStatus::OK, "%d active", active);
  }
}

void m2Iface::diagArmState(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const int idx = published_state_.load();
  stat.add("state", (idx >= 0 && idx <= 3) ? stateNames[idx] : "unknown");
  stat.add("executing", trajectory_executing_.load() ? "yes" : "no");
  stat.add("async_actions", async_actions_ ? "yes" : "no");
  stat.add("plan_only", planOnly ? "yes" : "no");
  stat.addf("vel_scale", "%.2f", max_vel_scaling_factor);
  stat.addf("acc_scale", "%.2f", max_acc_scaling_factor);
  const int64_t last = last_state_pub_ns_.load();
  if (last == 0) {
    stat.summary(DiagnosticStatus::WARN, "run loop not started");
    return;
  }
  const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const double busy = (now - last) * 1e-9;
  stat.addf("run_loop_last_tick_sec", "%.2f", busy);
        // run() blocks while planning (and executing in sync mode); long silences hide commands
  if (busy > 30.0) {
    stat.summaryf(DiagnosticStatus::ERROR, "run loop blocked for %.0f s", busy);
  } else if (busy > 5.0) {
    stat.summaryf(DiagnosticStatus::WARN, "run loop busy for %.1f s", busy);
  } else {
    stat.summary(DiagnosticStatus::OK, stateNames[idx]);
  }
}
