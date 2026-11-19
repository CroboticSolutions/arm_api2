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

/*      Title       : moveit2_iface_motion.cpp
 *      Project     : arm_api2
 *      Created     : 06/08/2025
 *      Author      : Filip Zoric
 *      Contributors: Edgar Welte
 *      Description : Planning and execution of joint/pose/path commands (actions and topics), plan status reporting.
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

template<class ActionT>
m2Iface::ExecProgress m2Iface::actionProgress(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
{
  return [goal_handle](double elapsed, double total) {
           if (!goal_handle->is_active()) {return;}
           auto fb = std::make_shared<typename ActionT::Feedback>();
           fb->status = goal_handle->is_canceling() ? "stopping" : "executing";
           fb->elapsed_sec = elapsed;
           fb->planned_duration_sec = total;
           const auto p = arm_api2::logic::execProgress(elapsed, total);
           fb->remaining_sec = p.remaining;
           fb->progress = p.progress;
           goal_handle->publish_feedback(fb);
         };
}

template<class ActionT>
m2Iface::ExecDone m2Iface::actionDone(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle,
  const std::string & label, const geometry_msgs::msg::PoseStamped & pose,
  double plan_time_sec, const std::string & mode)
{
  return [this, goal_handle, label, pose, plan_time_sec, mode](bool ok, const std::string & code) {
           publishPlanStatus(ok, execErrorCode(ok, code), label + ": " +
             (ok ? "Execution finished (" + code + ")" :
             "Execution did not complete (" + code + ")"), pose, plan_time_sec, mode);
           if (!goal_handle->is_active()) {return;}
           auto result = std::make_shared<typename ActionT::Result>();
           result->success = ok;
           if (ok) {
             RCLCPP_INFO_STREAM(this->get_logger(),
        label << ": execution succeeded (" << code << ")");
             goal_handle->succeed(result);
           } else if (goal_handle->is_canceling()) {
             RCLCPP_WARN_STREAM(this->get_logger(), label << ": canceled (" << code << ")");
             goal_handle->canceled(result);
           } else {
             RCLCPP_ERROR_STREAM(this->get_logger(),
        label << ": execution failed (" << code << ")");
             goal_handle->abort(result);
           }
         };
}

template<class ActionT>
void m2Iface::executeForGoal(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle,
  const moveit_msgs::msg::RobotTrajectory & trajectory, const std::string & label,
  const geometry_msgs::msg::PoseStamped & pose, double plan_time_sec)
{
  auto feedback = std::make_shared<typename ActionT::Feedback>();
  feedback->status = "executing";
  goal_handle->publish_feedback(feedback);
  const std::string mode = stateNames[robotState];
  auto done = actionDone<ActionT>(goal_handle, label, pose, plan_time_sec, mode);
  auto progress = actionProgress<ActionT>(goal_handle);
  auto not_started = [&]() {
      publishActionStatus(label, false, "EXECUTE_FAILED",
                                                       "Plan succeeded but execution did not start",
      pose, plan_time_sec);
      auto result = std::make_shared<typename ActionT::Result>();
      result->success = false;
      goal_handle->abort(result);
    };
  if (async_actions_) {
    if (!startExecution(trajectory, done, progress, goal_handle.get())) {
      not_started();
      return;
    }
    publishActionStatus(label, true, "SUCCESS", "Plan accepted; executing", pose, plan_time_sec,
                                    false);
    return;
  }
  publishActionStatus(label, true, "SUCCESS", "Plan accepted; executing", pose, plan_time_sec,
                            false);
  std::string code;
  const bool ok = runExecutionBlocking(trajectory, progress, goal_handle.get(), code);
  if (!ok && code == "NOT_STARTED") {
    not_started();
    return;
  }
  done(ok, code);
}

void m2Iface::publishActionStatus(
  const std::string & label, bool success, const std::string & code,
  const std::string & reason, const geometry_msgs::msg::PoseStamped & pose,
  double plan_time_sec, bool final_status)
{
  publishPlanStatus(success, code, label + ": " + reason, pose, plan_time_sec,
                          stateNames[robotState], final_status);
}

template<class ActionT>
bool m2Iface::finishIfCanceling(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
{
  if (!goal_handle || !goal_handle->is_canceling()) {return false;}
  auto result = std::make_shared<typename ActionT::Result>();
  result->success = false;
  goal_handle->canceled(result);
  RCLCPP_INFO(this->get_logger(), "Queued goal was canceled before planning");
  return true;
}

void m2Iface::pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (robotState != CART_TRAJ_CTL && robotState != JOINT_TRAJ_CTL) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Ignoring cmd_pose: set robot to JOINT_TRAJ_CTL or CART_TRAJ_CTL first.");
    return;
  }
  geometry_msgs::msg::PoseStamped cmd_plan;
  const std::string source_frame = msg->header.frame_id;
  if (source_frame.empty() || source_frame == PLANNING_FRAME) {
    cmd_plan = *msg;
    cmd_plan.header.frame_id = PLANNING_FRAME;
  } else {
    try {
      cmd_plan = tf_buffer_->transform(*msg, PLANNING_FRAME);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
                                this->get_logger(),
                "cmd_pose: cannot transform '%s' -> '%s': %s",
                source_frame.c_str(), PLANNING_FRAME.c_str(), ex.what());
      return;
    }
  }
  std::lock_guard<std::mutex> lock(pose_topic_mutex_);
  m_topicPoseCmd = cmd_plan;
  m_topicPoseCmd.header.frame_id = PLANNING_FRAME;
    /* Ignore a repeat of the last successfully started pose — the arm is
     * already going there; a *failed* pose is not stored, so a re-click retries. */
  if (!utils::comparePose(m_topicPoseCmd, m_oldTopicPoseCmd)) {
    received_topic_pose_cmd_ = true;
  } else {
    publishPlanStatus(true, "ALREADY_COMMANDED",
      "Same pose as the last started command (within 1 mm / 0.1 deg); not replanned",
                                  m_topicPoseCmd, 0.0,
      robotState == JOINT_TRAJ_CTL ? "JOINT_TRAJ_CTL" : "CART_TRAJ_CTL");
  }
}

void m2Iface::cart_poses_cb(const arm_api2_msgs::msg::CartesianWaypoints::SharedPtr msg)
{
  if (robotState != CART_TRAJ_CTL) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Ignoring cmd_traj: set robot to Cartesian control (CART_TRAJ_CTL / GUI CARTESIAN_CONTROL) first.");
    return;
  }
  if (!msg || msg->poses.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Ignoring cmd_traj: at least two waypoints are required.");
    return;
  }
  m_cartesianWaypoints = msg->poses;
  received_topic_traj_cmd_ = true;
}

rclcpp_action::GoalResponse m2Iface::move_to_joint_goal_cb(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const arm_api2_msgs::action::MoveJoint::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Received goal request for joint control! Number of joints: " <<
    goal->joint_state.position.size());
  if(robotState != JOINT_TRAJ_CTL) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in joint control mode!");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if(goal->joint_state.position.size() != m_currJointPosition.size()) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Number of joint positions does not match the number of joints!");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (trajectory_executing_.load()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "A trajectory is still executing; goal rejected.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_joint_cancel_cb(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>>
  goal_handle)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel joint control!");
  // Running: cancel its execution (the done callback reports canceled). Queued:
  // run() sees is_canceling() and finishes it without planning.
  cancelExecution("action goal canceled", goal_handle.get());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_joint_accepted_cb(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>> goal_handle)
{
  m_moveToJointGoalHandle_ = goal_handle;
  received_cmd_ = true;
}

rclcpp_action::GoalResponse m2Iface::move_to_pose_goal_cb(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const arm_api2_msgs::action::MoveCartesian::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for Cartesian control!");
  if(robotState != CART_TRAJ_CTL) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in Cartesian control mode!");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if(goal->goal.header.frame_id != PLANNING_FRAME) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Pose frame_id is not planning frame! PLANNING_FRAME: " <<
      PLANNING_FRAME);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (trajectory_executing_.load()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "A trajectory is still executing; goal rejected.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_pose_cancel_cb(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>>
  goal_handle)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel Cartesian control!");
  // Running: cancel its execution (the done callback reports canceled). Queued:
  // run() sees is_canceling() and finishes it without planning.
  cancelExecution("action goal canceled", goal_handle.get());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_pose_accepted_cb(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>> goal_handle)
{
  m_moveToPoseGoalHandle_ = goal_handle;
  received_cmd_ = true;
}

rclcpp_action::GoalResponse m2Iface::move_to_pose_path_goal_cb(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const arm_api2_msgs::action::MoveCartesianPath::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for Cartesian path control!");
  if(robotState != CART_TRAJ_CTL) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in Cartesian control mode!");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if(goal->poses.size() < 2) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Number of poses in path is less than 2!");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if(goal->poses[0].header.frame_id != PLANNING_FRAME) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Path frame_id is not planning frame! PLANNING_FRAME: " <<
      PLANNING_FRAME);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (trajectory_executing_.load()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "A trajectory is still executing; goal rejected.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_pose_path_cancel_cb(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>>
  goal_handle)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel Cartesian path control!");
  // Running: cancel its execution (the done callback reports canceled). Queued:
  // run() sees is_canceling() and finishes it without planning.
  cancelExecution("action goal canceled", goal_handle.get());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_pose_path_accepted_cb(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>>
  goal_handle)
{
  m_moveToPosePathGoalHandle_ = goal_handle;
  received_traj_ = true;
}

m2Iface::ExecDone m2Iface::topicExecDone(
  const geometry_msgs::msg::PoseStamped & pose, double plan_time_sec,
  const std::string & mode, bool forget_pose)
{
  return [this, pose, plan_time_sec, mode, forget_pose](bool ok, const std::string & code) {
           if (ok) {
             publishPlanStatus(true, "SUCCESS", "Execution finished (" + code + ")", pose,
             plan_time_sec, mode);
             return;
           }
           if (forget_pose) {
             std::lock_guard<std::mutex> lock(pose_topic_mutex_);
             m_oldTopicPoseCmd = geometry_msgs::msg::PoseStamped();
           }
           publishPlanStatus(false, execErrorCode(false, code),
                                         "Execution did not complete (" + code + ")", pose,
             plan_time_sec, mode);
         };
}

void m2Iface::planAndExecJoint()
{
  const std::string label = "move_to_joint";
  auto gh = m_moveToJointGoalHandle_;
  const auto feedback = std::make_shared<arm_api2_msgs::action::MoveJoint::Feedback>();
  const auto result = std::make_shared<arm_api2_msgs::action::MoveJoint::Result>();
  geometry_msgs::msg::PoseStamped no_pose;  // joint goal: no requested pose
  no_pose.header.frame_id = PLANNING_FRAME;
  if (finishIfCanceling(gh)) {return;}
  motionBegin(label, "action");
  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(this->get_logger(), "Trajectory still executing; aborting joint goal.");
    publishActionStatus(label, false, "BUSY_TRAJECTORY_EXECUTING",
      "Trajectory still executing; goal aborted", no_pose, 0.0);
    result->success = false;
    gh->abort(result);
    return;
  }
  feedback->set__status("planning");
  gh->publish_feedback(feedback);

  m_moveGroupPtr->setJointValueTarget(gh->get_goal()->joint_state);
  m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

  const auto plan_t0 = std::chrono::steady_clock::now();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = planWithPlanner(plan, eager_execution);
  const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() -
    plan_t0).count();
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Planning to joint space goal: " << (success ? "SUCCEEDED" : "FAILED"));
  if (success) {motionPlanned(last_planner_, plan_dt, NAN, plan.trajectory, nullptr);}

  if (!success) {
    publishActionStatus(label, false, "PLAN_NOT_FOUND",
                                    "No configured planner found a plan for this joint goal",
      no_pose, plan_dt);
    result->success = false;
    gh->abort(result);
  } else if (planOnly) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Plan only mode!");
    publishActionStatus(label, true, "SUCCESS", "Plan only mode; not executing", no_pose, plan_dt);
    result->success = true;
    gh->succeed(result);
  } else {
    executeForGoal(gh, plan.trajectory, label, no_pose, plan_dt);
  }
}

void m2Iface::planAndExecPose()
{
  const std::string label = "move_to_pose";
  auto gh = m_moveToPoseGoalHandle_;
  const auto feedback = std::make_shared<arm_api2_msgs::action::MoveCartesian::Feedback>();
  const auto result = std::make_shared<arm_api2_msgs::action::MoveCartesian::Result>();
  if (finishIfCanceling(gh)) {return;}
  motionBegin(label, "action");
  auto goalPose = gh->get_goal()->goal;
  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(this->get_logger(), "Trajectory still executing; aborting pose goal.");
    publishActionStatus(label, false, "BUSY_TRAJECTORY_EXECUTING",
      "Trajectory still executing; goal aborted", goalPose, 0.0);
    result->success = false;
    gh->abort(result);
    return;
  }
  feedback->set__status("planning");
  gh->publish_feedback(feedback);

  RCLCPP_INFO_STREAM(this->get_logger(),
    "Planning to Cartesian pose " << goalPose.pose.position.x << " " <<
                           goalPose.pose.position.y << " " << goalPose.pose.position.z <<
    " from " <<
                           m_currPoseState.pose.position.x << " " <<
    m_currPoseState.pose.position.y << " " <<
    m_currPoseState.pose.position.z);

  m_moveGroupPtr->setPoseTarget(goalPose);
  m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

  const auto plan_t0 = std::chrono::steady_clock::now();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = planWithPlanner(plan, eager_execution);
  const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() -
    plan_t0).count();
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Planning to pose goal: " << (success ? "SUCCEEDED" : "FAILED"));
  if (success) {motionPlanned(last_planner_, plan_dt, NAN, plan.trajectory, &goalPose);}

  if (!success) {
    publishActionStatus(label, false, "PLAN_NOT_FOUND",
                                    "No configured planner found a plan for this pose", goalPose,
      plan_dt);
    result->success = false;
    gh->abort(result);
  } else if (planOnly) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Plan only mode!");
    publishActionStatus(label, true, "SUCCESS", "Plan only mode; not executing", goalPose,
      plan_dt);
    result->success = true;
    gh->succeed(result);
  } else {
    executeForGoal(gh, plan.trajectory, label, goalPose, plan_dt);
  }
}

void m2Iface::planAndExecTopicPose()
{
  motionBegin("topic_pose", "topic");
  geometry_msgs::msg::PoseStamped goalPose;
  {
    std::lock_guard<std::mutex> lock(pose_topic_mutex_);
    goalPose = m_topicPoseCmd;
  }

  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory still executing; ignoring new pose command until it finishes.");
    publishPlanStatus(false, "BUSY_TRAJECTORY_EXECUTING",
                          "Trajectory still executing; ignored new pose command",
                          goalPose, 0.0, "CART_TRAJ_CTL");
    return;
  }

  stopBeforeAsyncExecute();

  RCLCPP_INFO_STREAM(this->get_logger(), "Planning to Cartesian Pose from cmd_pose topic!");
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Current pose is: " << m_currPoseState.pose.position.x << " " <<
                           m_currPoseState.pose.position.y << " " <<
    m_currPoseState.pose.position.z);
  RCLCPP_INFO_STREAM(this->get_logger(),
                           "Target pose is: " << goalPose.pose.position.x << " " <<
    goalPose.pose.position.y << " " <<
    goalPose.pose.position.z);

  m_moveGroupPtr->setPoseTarget(goalPose);
  m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

  const auto plan_t0 = std::chrono::steady_clock::now();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = planTopicPoseFast(plan);
  const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() -
    plan_t0).count();
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Planning to topic pose goal: " << (success ? "SUCCEEDED" : "FAILED"));
  if (success) {motionPlanned(last_planner_, plan_dt, NAN, plan.trajectory, &goalPose);}

  if (!success) {
    publishPlanStatus(false, "PLAN_NOT_FOUND",
                          "MoveIt planner did not return SUCCESS for this pose",
                          goalPose, plan_dt, "CART_TRAJ_CTL");
    return;
  }
  if (planOnly) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Plan only mode!");
    publishPlanStatus(true, "SUCCESS", "Plan only mode; not executing", goalPose, plan_dt,
      "CART_TRAJ_CTL");
    return;
  }

  {
    // Before execute: a fast failure must be able to clear it again (retry).
    std::lock_guard<std::mutex> lock(pose_topic_mutex_);
    m_oldTopicPoseCmd = goalPose;
  }
  const bool ok = execPlan_with_plan(plan, async,
    topicExecDone(goalPose, plan_dt, "CART_TRAJ_CTL", true));
  if (ok) {
    publishPlanStatus(true, "SUCCESS", "Plan accepted; executing", goalPose, plan_dt,
                                  "CART_TRAJ_CTL", false);
  } else {
    {
      std::lock_guard<std::mutex> lock(pose_topic_mutex_);
      m_oldTopicPoseCmd = geometry_msgs::msg::PoseStamped();
    }
    publishPlanStatus(false, "EXECUTE_FAILED",
                          "Plan succeeded but execute() did not start",
                          goalPose, plan_dt, "CART_TRAJ_CTL");
  }
}

void m2Iface::planAndExecTopicPoseJoint()
{
  motionBegin("topic_pose_joint", "topic");
  geometry_msgs::msg::PoseStamped pose_snap;
  {
    std::lock_guard<std::mutex> lock(pose_topic_mutex_);
    pose_snap = m_topicPoseCmd;
  }

  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory still executing; ignoring new pose command until it finishes.");
    publishPlanStatus(false, "BUSY_TRAJECTORY_EXECUTING",
                          "Trajectory still executing; ignored new pose command",
                          pose_snap, 0.0, "JOINT_TRAJ_CTL");
    return;
  }

  stopBeforeAsyncExecute();
  geometry_msgs::msg::PoseStamped cmdPose_ = utils::normalizeOrientation(pose_snap);
  m_moveGroupPtr->clearPoseTargets();
  m_moveGroupPtr->setPoseTarget(cmdPose_.pose, EE_LINK_NAME);
  m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
  if (pose_plan_time_sec_ > 0.0) {
    m_moveGroupPtr->setPlanningTime(pose_plan_time_sec_);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "poseTarget is: "    << cmdPose_.pose.position.x << " "
                                                              << cmdPose_.pose.position.y << " "
                                                              << cmdPose_.pose.position.z);
  const auto plan_t0 = std::chrono::steady_clock::now();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_err = m_moveGroupPtr->plan(plan);
  const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() -
    plan_t0).count();

  if (plan_err != moveit::core::MoveItErrorCode::SUCCESS) {
    const std::string code = utils::moveItErrorCodeToString(plan_err);
    RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Planning failed (" << code << ") in " << plan_dt <<
      " s; not executing.");
    publishPlanStatus(false, code,
                          "MoveIt planner did not return SUCCESS for this pose",
                          cmdPose_, plan_dt, "JOINT_TRAJ_CTL");
    return;
  }
  motionPlanned(m_moveGroupPtr->getPlanningPipelineId() + "/" + m_moveGroupPtr->getPlannerId(),
                      plan_dt, NAN, plan.trajectory, &cmdPose_);

  if (planOnly) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Plan only mode!");
    publishPlanStatus(true, "SUCCESS", "Plan only mode; not executing", cmdPose_, plan_dt,
      "JOINT_TRAJ_CTL");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(pose_topic_mutex_);
    m_oldTopicPoseCmd = cmdPose_;
  }
  const bool ok = execPlan_with_plan(plan, async,
                                           topicExecDone(cmdPose_, plan_dt, "JOINT_TRAJ_CTL",
    true));
  if (ok) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Plan succeeded; executing commanded path.");
    publishPlanStatus(true, "SUCCESS", "Plan accepted; executing", cmdPose_, plan_dt,
                                  "JOINT_TRAJ_CTL", false);
  } else {
    {
      std::lock_guard<std::mutex> lock(pose_topic_mutex_);
      m_oldTopicPoseCmd = geometry_msgs::msg::PoseStamped();
    }
    publishPlanStatus(false, "EXECUTE_FAILED",
                          "Plan succeeded but execute() did not start",
                          cmdPose_, plan_dt, "JOINT_TRAJ_CTL");
  }
}

bool m2Iface::planTopicPoseFast(moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  // YAML planning.topic_pose_planners (default Pilz LIN 0.25 s, then OMPL EST 0.75 s)
  const double original_planning_time = m_moveGroupPtr->getPlanningTime();
  last_planner_.clear();
  for (const auto & a : topic_pose_planners_) {
    m_moveGroupPtr->setPlanningPipelineId(a.pipeline);
    m_moveGroupPtr->setPlannerId(a.planner);
    m_moveGroupPtr->setPlanningTime(a.time > 0.0 ? a.time : original_planning_time);
    for (int t = 0; t < a.attempts; ++t) {
      moveit::planning_interface::MoveGroupInterface::Plan candidate;
      if (static_cast<bool>(m_moveGroupPtr->plan(candidate))) {
        RCLCPP_INFO(this->get_logger(), "%s/%s found topic pose plan with %d points",
          a.pipeline.c_str(), a.planner.c_str(),
          int(candidate.trajectory.joint_trajectory.points.size()));
        last_planner_ = a.pipeline + "/" + a.planner;
        plan = candidate;
        m_moveGroupPtr->setPlanningTime(original_planning_time);
        return true;
      }
      RCLCPP_INFO(this->get_logger(), "%s/%s failed topic pose plan (attempt %d)",
                                    a.pipeline.c_str(), a.planner.c_str(), t + 1);
    }
  }
  m_moveGroupPtr->setPlanningTime(original_planning_time);
  RCLCPP_WARN(this->get_logger(), "Topic pose planning failed with all configured planners.");
  return false;
}

void m2Iface::planAndExecPosePath()
{
  const std::string label = "move_to_pose_path";
  auto gh = m_moveToPosePathGoalHandle_;
  const auto feedback = std::make_shared<arm_api2_msgs::action::MoveCartesianPath::Feedback>();
  const auto result = std::make_shared<arm_api2_msgs::action::MoveCartesianPath::Result>();
  if (finishIfCanceling(gh)) {return;}
  motionBegin(label, "action");
  const auto goalPoseStampeds = gh->get_goal()->poses;
  const geometry_msgs::msg::PoseStamped target = goalPoseStampeds.back();
  auto fail = [&](const std::string & code, const std::string & reason, double plan_dt) {
      RCLCPP_ERROR_STREAM(this->get_logger(), label << ": " << reason);
      publishActionStatus(label, false, code, reason, target, plan_dt);
      result->success = false;
      gh->abort(result);
    };
  if (trajectory_executing_.load(std::memory_order_acquire)) {
    fail("BUSY_TRAJECTORY_EXECUTING", "Trajectory still executing; goal aborted", 0.0);
    return;
  }
  feedback->set__status("planning");
  gh->publish_feedback(feedback);

  RCLCPP_INFO_STREAM(this->get_logger(), "Planning Cartesian path with " <<
                           goalPoseStampeds.size() << " waypoints to " << target.pose.position.x <<
    " " <<
    target.pose.position.y << " " << target.pose.position.z);

  std::vector<geometry_msgs::msg::Pose> goalPoses;
  for (const auto & pose : goalPoseStampeds) {
    goalPoses.push_back(pose.pose);
  }

  const auto plan_t0 = std::chrono::steady_clock::now();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = m_moveGroupPtr->computeCartesianPath(goalPoses, eef_step_, trajectory,
    true);
  const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() -
    plan_t0).count();
  motionPlanned("cartesian", plan_dt, fraction, trajectory, &target);

  // The action keeps its contract: only a complete path is executed.
  if (fraction <= 0.0 || trajectory.joint_trajectory.points.empty()) {
    fail("PLAN_NOT_FOUND", "Cartesian path could not be computed (fraction=" +
      std::to_string(fraction) + ")", plan_dt);
    return;
  }
  if (fraction < 1.0) {
    fail("PARTIAL_PATH", "Cartesian path only partial (fraction=" + std::to_string(fraction) +
      ", the action requires 1.0); not executed", plan_dt);
    return;
  }
  std::string jumpReason;
  if (!jointPathContinuous(trajectory, jumpReason)) {
    fail("PLAN_NOT_FOUND", "Cartesian path rejected: " + jumpReason, plan_dt);
    return;
  }
  if (planOnly) {
    publishActionStatus(label, true, "SUCCESS", "Plan only mode; not executing", target, plan_dt);
    result->success = true;
    gh->succeed(result);
    return;
  }
  if (!addTimestampsToTrajectory(trajectory)) {
    fail("EXECUTE_FAILED", "Cartesian plan ready but timestamping failed (no current robot state)",
      plan_dt);
    return;
  }
  executeForGoal(gh, trajectory, label, target, plan_dt);
}

void m2Iface::planAndExecTopicPosePath()
{
  if (m_cartesianWaypoints.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Topic Cartesian path needs at least two waypoints.");
    return;
  }

  RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Planning Cartesian path from cmd_traj topic with " <<
      m_cartesianWaypoints.size() <<
      " waypoints.");

  motionBegin("topic_path", "topic");
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = PLANNING_FRAME;
  target_pose.pose = m_cartesianWaypoints.back();

  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory still executing; ignoring Cartesian waypoint trajectory.");
    publishPlanStatus(false, "BUSY_TRAJECTORY_EXECUTING",
                          "Trajectory still executing; ignored Cartesian waypoint trajectory",
                          target_pose, 0.0, "CART_TRAJ_CTL");
    return;
  }

  stopBeforeAsyncExecute();
  m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

  const auto plan_t0 = std::chrono::steady_clock::now();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction =
    m_moveGroupPtr->computeCartesianPath(m_cartesianWaypoints, eef_step_, trajectory, true);
  const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() -
    plan_t0).count();
  motionPlanned("cartesian", plan_dt, fraction, trajectory, &target_pose);

  if (fraction <= 0.0 || trajectory.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(
                        this->get_logger(),
            "Planning Cartesian path from topic failed (fraction=%f)",
            fraction);
    publishPlanStatus(false, "PLAN_NOT_FOUND",
                          "Cartesian path could not be computed (fraction=" +
      std::to_string(fraction) + ")",
                          target_pose, plan_dt, "CART_TRAJ_CTL");
    return;
  }

  if (fraction < cartesian_path_min_fraction_) {
    // A partial path stops short of the target: never execute it as if it succeeded.
    RCLCPP_ERROR(
                        this->get_logger(),
            "Topic Cartesian path only partial (fraction=%f < %f); not executing",
            fraction, cartesian_path_min_fraction_);
    publishPlanStatus(false, "PARTIAL_PATH",
                                  "Cartesian path only partial (fraction=" +
      std::to_string(fraction) +
                                  ", minimum " + std::to_string(cartesian_path_min_fraction_) +
      "); not executed",
                          target_pose, plan_dt, "CART_TRAJ_CTL");
    return;
  }

  std::string jumpReason;
  if (!jointPathContinuous(trajectory, jumpReason)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Topic Cartesian path rejected: " << jumpReason);
    publishPlanStatus(false, "PLAN_NOT_FOUND", "Cartesian path rejected: " + jumpReason,
                          target_pose, plan_dt, "CART_TRAJ_CTL");
    return;
  }

  if (planOnly) {
    RCLCPP_INFO(
                        this->get_logger(),
            "Plan-only mode: topic Cartesian path planned (fraction=%f), not executing.",
            fraction);
    publishPlanStatus(true, "SUCCESS", "Plan only mode; not executing", target_pose, plan_dt,
      "CART_TRAJ_CTL");
    return;
  }

  if (!addTimestampsToTrajectory(trajectory)) {
    publishPlanStatus(false, "EXECUTE_FAILED",
                          "Cartesian plan ready but timestamping failed (no current robot state)",
                          target_pose, plan_dt, "CART_TRAJ_CTL");
    return;
  }
  const bool ok = execTrajectory(std::move(trajectory), async,
    topicExecDone(target_pose, plan_dt, "CART_TRAJ_CTL", false));
  if (ok) {
    const std::string reason = (fraction >= 0.999) ?
      "Cartesian plan executing (full path)" :
      "Cartesian plan executing (partial fraction=" + std::to_string(fraction) + ")";
    publishPlanStatus(true, "SUCCESS", reason, target_pose, plan_dt, "CART_TRAJ_CTL", false);
  } else {
    publishPlanStatus(false, "EXECUTE_FAILED",
                          "Cartesian plan ready but execute() did not start",
                          target_pose, plan_dt, "CART_TRAJ_CTL");
  }
}

void m2Iface::printTimestamps(const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  trajectory_msgs::msg::JointTrajectory jointTrajectory = trajectory.joint_trajectory;
  std::vector<std::string> joint_names = jointTrajectory.joint_names;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points = jointTrajectory.points;
  for (long unsigned int i = 0; i < points.size(); i++) {
    trajectory_msgs::msg::JointTrajectoryPoint point = points[i];
    rclcpp::Duration duration = point.time_from_start;
    RCLCPP_INFO_STREAM(this->get_logger(),
                                   "Point " << i << " - time_from_start [s]: " <<
      duration.seconds());
  }
}

bool m2Iface::addTimestampsToTrajectory(moveit_msgs::msg::RobotTrajectory & trajectory)
{
    // The trajectory created with computeCartesianPath() needs to be modified so it will include velocities as well.
    // reference: https://groups.google.com/g/moveit-users/c/MOoFxy2exT4
  if (!m_moveGroupPtr) {
    RCLCPP_ERROR(this->get_logger(),
      "addTimestampsToTrajectory: move group is not ready");
    return false;
  }
  auto current_state = m_moveGroupPtr->getCurrentState(1.0);
  if (!current_state) {
    RCLCPP_ERROR(this->get_logger(),
      "addTimestampsToTrajectory: failed to get current robot state");
    return false;
  }
    // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(current_state->getRobotModel(), PLANNING_GROUP);
    // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*current_state, trajectory);
    // Thrid create a TimeOptimalTrajectoryGeneration object
  trajectory_processing::TimeOptimalTrajectoryGeneration iptp;
    // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt, max_vel_scaling_factor, max_acc_scaling_factor);
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Computed time stamp " << (success ? "SUCCEEDED" : "FAILED"));
  if (!success) {
    return false;
  }
    // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory);
  return true;
}

bool m2Iface::planWithPlanner(
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  bool eagerExecution)
{
  // YAML planning.pose_planners, in order (default Pilz LIN, OMPL EST, OMPL PRM, 3 attempts each).
  // eagerExecution: the first successful plan wins; otherwise the one with fewest points.
  const double original_planning_time = m_moveGroupPtr->getPlanningTime();
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> all_plans;
  std::vector<std::string> all_planners;
  last_planner_.clear();
  for (const auto & a : pose_planners_) {
    m_moveGroupPtr->setPlanningPipelineId(a.pipeline);
    m_moveGroupPtr->setPlannerId(a.planner);
    m_moveGroupPtr->setPlanningTime(a.time > 0.0 ? a.time : original_planning_time);
    for (int t = 0; t < a.attempts; ++t) {
      moveit::planning_interface::MoveGroupInterface::Plan candidate;
      if (static_cast<bool>(m_moveGroupPtr->plan(candidate))) {
        RCLCPP_INFO(this->get_logger(), "%s/%s found plan with %d points",
          a.pipeline.c_str(), a.planner.c_str(),
          int(candidate.trajectory.joint_trajectory.points.size()));
        if (eagerExecution) {
          last_planner_ = a.pipeline + "/" + a.planner;
          plan = candidate;
          m_moveGroupPtr->setPlanningTime(original_planning_time);
          return true;
        }
        all_plans.push_back(candidate);
        all_planners.push_back(a.pipeline + "/" + a.planner);
      } else {
        RCLCPP_INFO(this->get_logger(), "%s/%s failed to find a plan (attempt %d)",
                                            a.pipeline.c_str(), a.planner.c_str(), t + 1);
      }
    }
  }
  m_moveGroupPtr->setPlanningTime(original_planning_time);

  if (all_plans.empty()) {
    RCLCPP_INFO_STREAM(this->get_logger(), "All planners failed!");
    return false;
  }
  auto best_plan = std::min_element(all_plans.begin(), all_plans.end(),
      [](auto const & a, auto const & b){
        return a.trajectory.joint_trajectory.points.size() <
               b.trajectory.joint_trajectory.points.size();
        });
  plan = *best_plan;
  last_planner_ = all_planners[static_cast<size_t>(best_plan - all_plans.begin())];
  RCLCPP_INFO(this->get_logger(), "Best of %zu plans selected with %d points.", all_plans.size(),
    int(best_plan->trajectory.joint_trajectory.points.size()));
  return true;
}

void m2Iface::publishPlanStatus(
  bool success,
  const std::string & error_code,
  const std::string & reason,
  const geometry_msgs::msg::PoseStamped & requested_pose,
  double plan_time_sec,
  const std::string & mode,
  bool final_status)
{
  if (final_status) {motionStatus(success, error_code, reason, requested_pose, plan_time_sec);}
  if (!plan_status_pub_) {return;}
  arm_api2_msgs::msg::PlanStatus msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = PLANNING_FRAME;
  msg.success = success;
  msg.error_code = error_code;
  msg.reason = reason;
  msg.requested_pose = requested_pose;
  msg.plan_time_sec = plan_time_sec;
  msg.mode = mode;
  plan_status_pub_->publish(msg);
}

bool m2Iface::jointPathContinuous(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  std::string & reason) const
{
  return arm_api2::logic::jointPathContinuous(trajectory.joint_trajectory, max_joint_step_rad_,
                                                    reason);
}
