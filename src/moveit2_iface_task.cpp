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

/*      Title       : moveit2_iface_task.cpp
 *      Project     : arm_api2
 *
 *      Description : Tasks without MTC. A task (list of TaskStep) is planned as a whole
 *                    in a virtual copy of the planning scene: each motion step starts
 *                    where the previous one ended and attach/detach steps change the
 *                    copy. Only a complete plan is stored (plan_id) or executed.
 *                    Execution runs segment by segment on a worker thread through
 *                    startExecution (arm/stop, cancel, goal checks, motion log).
 */

#include "arm_api2/moveit2_iface.hpp"
#include "moveit2_iface_internal.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <type_traits>

#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.hpp>

namespace
{
using TaskStepMsg = arm_api2_msgs::msg::TaskStep;
using ExecuteTask = arm_api2_msgs::action::ExecuteTask;
using ExecutePlan = arm_api2_msgs::action::ExecutePlan;
using MoveRelative = arm_api2_msgs::action::MoveRelative;
using MoveToNamed = arm_api2_msgs::action::MoveToNamed;
using MoveGroupAction = moveit_msgs::action::MoveGroup;

const char * stepTypeName(uint8_t t)
{
  switch (t) {
    case TaskStepMsg::MOVE_JOINT: return "MOVE_JOINT";
    case TaskStepMsg::MOVE_POSE: return "MOVE_POSE";
    case TaskStepMsg::MOVE_CARTESIAN_PATH: return "MOVE_CARTESIAN_PATH";
    case TaskStepMsg::MOVE_RELATIVE: return "MOVE_RELATIVE";
    case TaskStepMsg::MOVE_NAMED: return "MOVE_NAMED";
    case TaskStepMsg::GRIPPER_OPEN: return "GRIPPER_OPEN";
    case TaskStepMsg::GRIPPER_CLOSE: return "GRIPPER_CLOSE";
    case TaskStepMsg::GRIPPER_POSITION: return "GRIPPER_POSITION";
    case TaskStepMsg::ATTACH: return "ATTACH";
    case TaskStepMsg::DETACH: return "DETACH";
    case TaskStepMsg::WAIT: return "WAIT";
    case TaskStepMsg::SET_SPEED: return "SET_SPEED";
    default: return "UNKNOWN";
  }
}

bool isMotion(uint8_t t)
{
  return t == TaskStepMsg::MOVE_JOINT || t == TaskStepMsg::MOVE_POSE ||
         t == TaskStepMsg::MOVE_CARTESIAN_PATH || t == TaskStepMsg::MOVE_RELATIVE ||
         t == TaskStepMsg::MOVE_NAMED;
}

constexpr double kGripperNominalSec = 1.0;   // progress weight of a gripper step
constexpr const char * kPilz = "pilz_industrial_motion_planner";

double trajDuration(const moveit_msgs::msg::RobotTrajectory & t)
{
  if (t.joint_trajectory.points.empty()) {return 0.0;}
  return rclcpp::Duration(t.joint_trajectory.points.back().time_from_start).seconds();
}

template<class T, class = void>
struct HasPlanId : std::false_type {};
template<class T>
struct HasPlanId<T, std::void_t<decltype(std::declval<T>().plan_id)>>: std::true_type {};
template<class T, class = void>
struct HasFailedStep : std::false_type {};
template<class T>
struct HasFailedStep<T, std::void_t<decltype(std::declval<T>().failed_step)>>: std::true_type {};
template<class T, class = void>
struct HasCurrentStep : std::false_type {};
template<class T>
struct HasCurrentStep<T, std::void_t<decltype(std::declval<T>().current_step)>>: std::true_type {};
}  // namespace

/* ------------------------------------------------------------------ helpers */

std::string m2Iface::interfaceName(
  const char * kind, const std::string & key, const std::string & fallback) const
{
  if (config[kind] && config[kind][key] && config[kind][key]["name"]) {
    return config[kind][key]["name"].as<std::string>();
  }
  return fallback;
}

std::string m2Iface::stepLabel(const TaskStep & step, int index) const
{
  std::string s = std::to_string(index) + ":" + stepTypeName(step.type);
  if (!step.name.empty()) {s += " '" + step.name + "'";}
  return s;
}

bool m2Iface::resolvePoseInModel(
  const geometry_msgs::msg::PoseStamped & in, const moveit::core::RobotState & state,
  Eigen::Isometry3d & out, std::string & error)
{
  Eigen::Isometry3d p;
  tf2::fromMsg(in.pose, p);
  const std::string frame = in.header.frame_id.empty() ? PLANNING_FRAME : in.header.frame_id;
  bool found = false;
  // robot links, the model frame and attached-body (sub)frames: exact for the virtual state
  const Eigen::Isometry3d & f = state.getFrameTransform(frame, &found);
  if (found) {
    out = f * p;
    return true;
  }
  try {
    geometry_msgs::msg::PoseStamped ps = in;
    ps.header.frame_id = frame;
    ps.header.stamp = rclcpp::Time(0);
    const auto t = tf_buffer_->transform(ps, state.getRobotModel()->getModelFrame(),
        tf2::durationFromSec(0.5));
    tf2::fromMsg(t.pose, out);
    return true;
  } catch (const tf2::TransformException & ex) {
    error = "cannot transform '" + frame + "' -> '" + state.getRobotModel()->getModelFrame() +
      "': " + ex.what();
    return false;
  }
}

/* ------------------------------------------------------------------ planning */

bool m2Iface::planPoseOrJoint(
  planning_scene::PlanningScenePtr vscene, const moveit::core::RobotState & start,
  const std::vector<double> * joint_target, const geometry_msgs::msg::PoseStamped * pose_target,
  const std::string & planner_spec, double vel, double acc, PlannedSegment & seg,
  std::string & code, std::string & error)
{
  // Which planners: the configured list, or one given by the step.
  std::vector<PlannerAttempt> attempts;
  if (planner_spec.empty()) {
    for (const auto & a : pose_planners_) {
      // Pilz does not enforce path constraints: skip it while constraints are set.
      if (hasPathConstraints() && a.pipeline == kPilz) {continue;}
      attempts.push_back(a);
    }
  } else {
    const auto us = planner_spec.find('_');
    const std::string prefix = planner_spec.substr(0, us);
    const std::string id = (us == std::string::npos) ? "" : planner_spec.substr(us + 1);
    if (id.empty() || (prefix != "pilz" && prefix != "ompl")) {
      code = "INVALID_STEP";
      error = "planner '" + planner_spec + "': expected pilz_PTP, pilz_LIN or ompl_<id>";
      return false;
    }
    if (prefix == "pilz" && hasPathConstraints()) {
      code = "INVALID_STEP";
      error = "planner '" + planner_spec + "' ignores path constraints; clear them or use ompl";
      return false;
    }
    attempts.push_back({prefix == "pilz" ? kPilz : "ompl", id, pose_plan_time_sec_, 1});
  }
  if (attempts.empty()) {
    code = "PLAN_NOT_FOUND";
    error = "no planner left (only Pilz configured and path constraints are set)";
    return false;
  }
  if (!move_group_plan_client_ ||
    !move_group_plan_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    code = "NOT_READY";
    error = "move_group action server not available";
    return false;
  }

  // Scene for move_group: the live scene + the task's virtual changes (attached objects,
  // robot state). Start state: joints only, as a diff on that scene.
  moveit_msgs::msg::PlanningScene diff;
  vscene->getPlanningSceneDiffMsg(diff);
  diff.is_diff = true;
  moveit_msgs::msg::RobotState start_msg;
  moveit::core::robotStateToRobotStateMsg(start, start_msg, false);
  start_msg.is_diff = true;

  const double original_time = m_moveGroupPtr->getPlanningTime();
  auto restore = [&]() {
      m_moveGroupPtr->clearPoseTargets();
      m_moveGroupPtr->setStartStateToCurrentState();
      m_moveGroupPtr->setPlanningTime(original_time);
      m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
      m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
    };
  std::string last_error = "no plan";
  bool ok = false;
  for (const auto & a : attempts) {
    for (int t = 0; t < a.attempts && !ok; ++t) {
      m_moveGroupPtr->clearPoseTargets();
      m_moveGroupPtr->setStartState(start);
      if (joint_target) {
        if (!m_moveGroupPtr->setJointValueTarget(*joint_target)) {
          code = "INVALID_STEP";
          error = "joint target outside the joint limits";
          restore();
          return false;
        }
      } else {
        m_moveGroupPtr->setPoseTarget(*pose_target, EE_LINK_NAME);
      }
      m_moveGroupPtr->setPlanningPipelineId(a.pipeline);
      m_moveGroupPtr->setPlannerId(a.planner);
      m_moveGroupPtr->setPlanningTime(a.time > 0.0 ? a.time : original_time);
      m_moveGroupPtr->setMaxVelocityScalingFactor(vel);
      m_moveGroupPtr->setMaxAccelerationScalingFactor(acc);
      MoveGroupAction::Goal goal;
      m_moveGroupPtr->constructMotionPlanRequest(goal.request);
      goal.request.start_state = start_msg;
      goal.request.path_constraints = path_constraints_;
      goal.planning_options.plan_only = true;
      goal.planning_options.planning_scene_diff = diff;

      auto gh_future = move_group_plan_client_->async_send_goal(goal);
      if (gh_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
        !gh_future.get())
      {
        last_error = a.pipeline + "/" + a.planner + ": move_group did not accept the goal";
        continue;
      }
      auto res_future = move_group_plan_client_->async_get_result(gh_future.get());
      const double wait_s = (a.time > 0.0 ? a.time : original_time) + 10.0;
      if (res_future.wait_for(std::chrono::duration<double>(wait_s)) !=
        std::future_status::ready)
      {
        last_error = a.pipeline + "/" + a.planner + ": no planning result in time";
        continue;
      }
      const auto wr = res_future.get();
      if (!wr.result || wr.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
        wr.result->planned_trajectory.joint_trajectory.points.empty())
      {
        const int val = wr.result ? wr.result->error_code.val : 0;
        last_error = a.pipeline + "/" + a.planner + ": " +
          utils::moveItErrorCodeToString(moveit::core::MoveItErrorCode(val));
        RCLCPP_INFO(this->get_logger(), "task plan attempt failed (%s)", last_error.c_str());
        continue;
      }
      std::string jump;
      if (!jointPathContinuous(wr.result->planned_trajectory, jump)) {
        last_error = a.pipeline + "/" + a.planner + ": " + jump;
        continue;
      }
      seg.trajectory = wr.result->planned_trajectory;
      seg.planner = a.pipeline + "/" + a.planner;
      ok = true;
    }
    if (ok) {break;}
  }
  restore();
  if (!ok) {
    code = "PLAN_NOT_FOUND";
    error = "no planner found a plan (last: " + last_error + ")";
    return false;
  }
  seg.duration = trajDuration(seg.trajectory);
  return true;
}

bool m2Iface::planCartesian(
  planning_scene::PlanningScenePtr vscene, const moveit::core::RobotState & start,
  const EigenSTL::vector_Isometry3d & waypoints, double vel, double acc,
  PlannedSegment & seg, std::string & code, std::string & error)
{
  const auto * jmg = start.getJointModelGroup(PLANNING_GROUP);
  const auto * link = start.getLinkModel(EE_LINK_NAME);
  if (!jmg || !link) {
    code = "NOT_READY";
    error = "planning group or end-effector link not in the robot model";
    return false;
  }
  auto state_valid = [&](moveit::core::RobotState * s, const moveit::core::JointModelGroup * g,
    const double * values) -> bool {
      if (g && values) {s->setJointGroupPositions(g, values);}
      s->update();
      return vscene->isStateValid(*s, path_constraints_, PLANNING_GROUP, false);
    };
  moveit::core::RobotState st(start);
  std::vector<moveit::core::RobotStatePtr> states;
  const auto pct = moveit::core::CartesianInterpolator::computeCartesianPath(
    &st, jmg, states, link, waypoints, true, moveit::core::MaxEEFStep(eef_step_),
    moveit::core::CartesianPrecision(), state_valid, kinematics::KinematicsQueryOptions());
  if (pct.value < 0.999) {
    code = pct.value > 0.0 ? "PARTIAL_PATH" : "PLAN_NOT_FOUND";
    error = "Cartesian path " + std::string(pct.value > 0.0 ? "only partial" : "not possible") +
      " (fraction=" + std::to_string(pct.value) + ", collision, IK or path constraints)";
    return false;
  }
  seg.planner = "cartesian";
  if (states.size() < 2) {
    seg.trajectory = moveit_msgs::msg::RobotTrajectory();   // already there: nothing to move
    seg.duration = 0.0;
    return true;
  }
  robot_trajectory::RobotTrajectory rt(start.getRobotModel(), PLANNING_GROUP);
  for (const auto & s : states) {
    rt.addSuffixWayPoint(s, 0.0);
  }
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt, vel, acc)) {
    code = "PLAN_NOT_FOUND";
    error = "time parameterization of the Cartesian path failed";
    return false;
  }
  rt.getRobotTrajectoryMsg(seg.trajectory);
  std::string jump;
  if (!jointPathContinuous(seg.trajectory, jump)) {
    code = "PLAN_NOT_FOUND";
    error = "Cartesian path rejected: " + jump;
    return false;
  }
  seg.duration = trajDuration(seg.trajectory);
  return true;
}

m2Iface::TaskPlanResult m2Iface::planTask(
  const std::vector<TaskStep> & steps,
  const std::function<void(int, const std::string &)> & on_step)
{
  TaskPlanResult res;
  const auto t0 = std::chrono::steady_clock::now();
  auto fail = [&](int i, const std::string & code, const std::string & msg) {
      res.ok = false;
      res.failed_step = i;
      res.code = code;
      res.message = (i >= 0 ? "step " + stepLabel(steps[static_cast<size_t>(i)], i) + ": " :
        std::string()) + msg;
      res.plan_time_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
        .count();
      RCLCPP_WARN_STREAM(this->get_logger(), "Task planning failed: " << res.message);
      return res;
    };
  if (steps.empty()) {return fail(-1, "INVALID_STEP", "the task has no steps");}
  if (!moveGroupInit || !pSceneMonitorInit || !robotModelInit || !m_pSceneMonitorPtr) {
    return fail(-1, "NOT_READY", "MoveIt not fully initialized");
  }

  // Base scene = move_group's scene right now (get_planning_scene). The monitored scene
  // can lag: diffs still in flight (e.g. a clear just before an add) would make the task
  // miss objects. Fall back to the monitored scene if the service does not answer.
  planning_scene::PlanningScenePtr vscene;
  if (get_scene_client_ && get_scene_client_->wait_for_service(std::chrono::seconds(1))) {
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components = 0x3FF;   // everything (settings, state, objects, ACM, ...)
    auto fut = get_scene_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
      auto base = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
      if (base->setPlanningSceneMsg(fut.get()->scene)) {vscene = base->diff();}
    }
  }
  if (!vscene) {
    RCLCPP_WARN(this->get_logger(),
      "Task planning: move_group get_planning_scene failed; using the monitored scene");
    planning_scene_monitor::LockedPlanningSceneRO ls(m_pSceneMonitorPtr);
    if (!ls) {return fail(-1, "NOT_READY", "planning scene unavailable");}
    vscene = planning_scene::PlanningScene::clone(ls)->diff();
  }
  moveit::core::RobotState vstate(vscene->getCurrentState());
  // Start from the freshest joint_states (the scene monitor may lag a little).
  std::unordered_map<std::string, double> js;
  if (!latestJoints(js, 1.0)) {return fail(-1, "NO_JOINT_STATE", "no recent joint_states");}
  for (const auto & kv : js) {
    if (vstate.getRobotModel()->hasJointModel(kv.first) &&
      vstate.getRobotModel()->getJointModel(kv.first)->getVariableCount() == 1)
    {
      vstate.setVariablePosition(kv.first, kv.second);
    }
  }
  vstate.update();
  vscene->setCurrentState(vstate);
  const auto * jmg = vstate.getJointModelGroup(PLANNING_GROUP);
  if (!jmg) {return fail(-1, "NOT_READY", "planning group '" + PLANNING_GROUP + "' not found");}
  const std::vector<std::string> group_vars = jmg->getVariableNames();

  auto plan = std::make_shared<StoredPlan>();
  moveit::core::robotStateToRobotStateMsg(vstate, plan->start_state);
  double vel = max_vel_scaling_factor;
  double acc = max_acc_scaling_factor;

  for (size_t k = 0; k < steps.size(); ++k) {
    const int i = static_cast<int>(k);
    const auto & step = steps[k];
    if (on_step) {on_step(i, stepLabel(step, i));}
    PlannedSegment seg;
    seg.step = step;
    seg.index = i;
    std::string code;
    std::string err;
    std::vector<double> current;
    vstate.copyJointGroupPositions(jmg, current);

    switch (step.type) {
      case TaskStepMsg::MOVE_JOINT:
      case TaskStepMsg::MOVE_NAMED: {
          arm_api2::logic::NamedPose pose;
          if (step.type == TaskStepMsg::MOVE_NAMED) {
            if (!lookupNamedPose(step.named_pose, pose, err)) {
              return fail(i, "UNKNOWN_NAMED_POSE", err);
            }
          } else {
            pose.joint_names = step.joint_state.name;
            pose.positions = step.joint_state.position;
            if (pose.joint_names.empty()) {
              if (pose.positions.size() != group_vars.size()) {
                return fail(i, "INVALID_STEP", "joint_state without names needs " +
                  std::to_string(group_vars.size()) + " positions (group order)");
              }
              pose.joint_names = group_vars;
            } else if (pose.joint_names.size() != pose.positions.size()) {
              return fail(i, "INVALID_STEP", "joint_state name/position sizes differ");
            }
          }
          std::vector<double> target;
          if (!arm_api2::logic::namedPoseToGroup(pose, group_vars, current, target, err)) {
            return fail(i, "INVALID_STEP", err);
          }
          if (!planPoseOrJoint(vscene, vstate, &target, nullptr, step.planner, vel, acc, seg, code,
            err))
          {
            return fail(i, code, err);
          }
          break;
        }
      case TaskStepMsg::MOVE_POSE: {
          Eigen::Isometry3d target;
          if (!resolvePoseInModel(step.pose, vstate, target, err)) {
            return fail(i, "TF_ERROR", err);
          }
          geometry_msgs::msg::PoseStamped ps;
          ps.header.frame_id = vstate.getRobotModel()->getModelFrame();
          ps.pose = tf2::toMsg(target);
          if (!planPoseOrJoint(vscene, vstate, nullptr, &ps, step.planner, vel, acc, seg, code,
            err))
          {
            return fail(i, code, err);
          }
          break;
        }
      case TaskStepMsg::MOVE_CARTESIAN_PATH: {
          if (step.waypoints.empty()) {return fail(i, "INVALID_STEP", "no waypoints");}
          EigenSTL::vector_Isometry3d wps;
          for (const auto & w : step.waypoints) {
            Eigen::Isometry3d t;
            if (!resolvePoseInModel(w, vstate, t, err)) {return fail(i, "TF_ERROR", err);}
            wps.push_back(t);
          }
          if (!planCartesian(vscene, vstate, wps, vel, acc, seg, code, err)) {
            return fail(i, code, err);
          }
          break;
        }
      case TaskStepMsg::MOVE_RELATIVE: {
          const auto frame = arm_api2::logic::parseRelFrame(step.relative.frame);
          if (!frame) {
            return fail(i, "INVALID_STEP", "relative.frame '" + step.relative.frame +
              "': use tcp or base");
          }
          const auto & tr = step.relative.translation;
          const auto & rpy = step.relative.rotation_rpy;
          const Eigen::Isometry3d target = arm_api2::logic::relativeTarget(
            vstate.getGlobalLinkTransform(EE_LINK_NAME), *frame,
            Eigen::Vector3d(tr.x, tr.y, tr.z), Eigen::Vector3d(rpy.x, rpy.y, rpy.z));
          if (step.relative.linear) {
            if (!planCartesian(vscene, vstate, {target}, vel, acc, seg, code, err)) {
              return fail(i, code, err);
            }
          } else {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = vstate.getRobotModel()->getModelFrame();
            ps.pose = tf2::toMsg(target);
            if (!planPoseOrJoint(vscene, vstate, nullptr, &ps, step.planner, vel, acc, seg, code,
              err))
            {
              return fail(i, code, err);
            }
          }
          break;
        }
      case TaskStepMsg::GRIPPER_OPEN:
      case TaskStepMsg::GRIPPER_CLOSE:
      case TaskStepMsg::GRIPPER_POSITION:
        if (gripper_none_) {return fail(i, "INVALID_STEP", "no gripper on this robot/tool");}
        if (step.type == TaskStepMsg::GRIPPER_POSITION &&
          (step.gripper_position < 0.0 || step.gripper_position > 1.0))
        {
          return fail(i, "INVALID_STEP", "gripper_position must be 0..1");
        }
        seg.duration = kGripperNominalSec;
        break;
      case TaskStepMsg::ATTACH: {
          if (step.object_id.empty()) {return fail(i, "INVALID_STEP", "object_id is empty");}
          if (vstate.hasAttachedBody(step.object_id)) {
            return fail(i, "INVALID_STEP", "'" + step.object_id + "' is already attached");
          }
          if (!vscene->getWorld()->hasObject(step.object_id)) {
            return fail(i, "UNKNOWN_OBJECT", "'" + step.object_id +
              "' is not a collision object in the scene");
          }
          const std::string link = step.attach_link.empty() ? EE_LINK_NAME : step.attach_link;
          if (!vstate.getRobotModel()->hasLinkModel(link)) {
            return fail(i, "INVALID_STEP", "link '" + link + "' not in the robot model");
          }
          moveit_msgs::msg::AttachedCollisionObject aco;
          aco.link_name = link;
          aco.object.id = step.object_id;
          aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
          aco.touch_links = touchLinksFor(link, step.touch_links);
          vscene->setCurrentState(vstate);
          // attaches at the virtual (grasp) state and removes the object from the world copy
          if (!vscene->processAttachedCollisionObjectMsg(aco)) {
            return fail(i, "SCENE_UPDATE_FAILED", "attaching '" + step.object_id + "' failed");
          }
          vstate = vscene->getCurrentState();
          break;
        }
      case TaskStepMsg::DETACH: {
          std::vector<const moveit::core::AttachedBody *> bodies;
          vstate.getAttachedBodies(bodies);
          bool any = false;
          for (const auto * b : bodies) {
            if (!step.object_id.empty() && b->getName() != step.object_id) {continue;}
            moveit_msgs::msg::AttachedCollisionObject aco;
            aco.link_name = b->getAttachedLinkName();
            aco.object.id = b->getName();
            aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            vscene->setCurrentState(vstate);
            // back into the world copy where the virtual state put it
            if (!vscene->processAttachedCollisionObjectMsg(aco)) {
              return fail(i, "SCENE_UPDATE_FAILED", "detaching '" + aco.object.id + "' failed");
            }
            vstate = vscene->getCurrentState();
            any = true;
          }
          if (!any && !step.object_id.empty()) {
            return fail(i, "UNKNOWN_OBJECT", "'" + step.object_id + "' is not attached");
          }
          break;
        }
      case TaskStepMsg::WAIT:
        if (!(step.duration_sec >= 0.0 && step.duration_sec <= 3600.0)) {
          return fail(i, "INVALID_STEP", "duration_sec must be 0..3600");
        }
        seg.duration = step.duration_sec;
        break;
      case TaskStepMsg::SET_SPEED:
        if (step.vel_scaling < 0.0 || step.vel_scaling > 1.0 || step.acc_scaling < 0.0 ||
          step.acc_scaling > 1.0)
        {
          return fail(i, "INVALID_STEP", "vel_scaling/acc_scaling must be 0..1 (0 = keep)");
        }
        if (step.vel_scaling > 0.0) {vel = step.vel_scaling;}
        if (step.acc_scaling > 0.0) {acc = step.acc_scaling;}
        break;
      default:
        return fail(i, "INVALID_STEP", "unknown step type " + std::to_string(step.type));
    }

    if (isMotion(step.type) && !seg.trajectory.joint_trajectory.points.empty()) {
      const auto & jt = seg.trajectory.joint_trajectory;
      vstate.setVariablePositions(jt.joint_names, jt.points.back().positions);
      vstate.update();
      vscene->setCurrentState(vstate);
    }
    RCLCPP_INFO(this->get_logger(), "Task step %s planned (%s, %.2f s)",
      stepLabel(step, i).c_str(), seg.planner.empty() ? "-" : seg.planner.c_str(), seg.duration);
    plan->duration += seg.duration;
    plan->segments.push_back(std::move(seg));
  }
  res.ok = true;
  res.plan = plan;
  res.code = "SUCCESS";
  res.plan_time_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  res.message = "planned " + std::to_string(steps.size()) + " steps, " +
    std::to_string(plan->duration).substr(0, 5) + " s, in " +
    std::to_string(res.plan_time_sec).substr(0, 5) + " s";
  RCLCPP_INFO_STREAM(this->get_logger(), "Task " << res.message);
  return res;
}

/* ------------------------------------------------------------------ execution */

void m2Iface::cancelTask(const std::string & why)
{
  if (!task_active_.load()) {return;}
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    if (!task_cancel_.load()) {task_cancel_reason_ = why;}
    task_cancel_.store(true);
  }
  cancelExecution(why);
}

bool m2Iface::startTask(
  std::shared_ptr<StoredPlan> plan, const void * owner, TaskFeedbackFn fb, TaskDoneFn done,
  std::string & why)
{
  if (trajectory_executing_.load()) {
    why = "a trajectory is executing";
    return false;
  }
  bool expected = false;
  if (!task_active_.compare_exchange_strong(expected, true)) {
    why = "another task is running";
    return false;
  }
  if (task_thread_.joinable()) {task_thread_.join();}   // previous worker already finished
  task_owner_.store(owner);
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    task_cancel_.store(false);
    task_cancel_reason_.clear();
  }
  task_thread_ = std::thread(&m2Iface::taskWorker, this, std::move(plan), owner, std::move(fb),
      std::move(done));
  return true;
}

void m2Iface::taskWorker(
  std::shared_ptr<StoredPlan> plan, const void * owner, TaskFeedbackFn fb, TaskDoneFn done)
{
  std::vector<double> durations;
  for (const auto & s : plan->segments) {
    durations.push_back(s.duration);
  }
  const int total = static_cast<int>(plan->segments.size());
  auto finish = [&](bool ok, const std::string & code, const std::string & msg, int failed) {
      task_active_.store(false);
      if (ok) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Task finished: " << msg);
      } else {
        RCLCPP_WARN_STREAM(this->get_logger(), "Task stopped at step " << failed << ": " << code <<
          " " << msg);
      }
      if (done) {done(ok, code, msg, failed);}
    };
  auto cancel_reason = [&]() {
      std::lock_guard<std::mutex> lock(task_mutex_);
      return task_cancel_reason_;
    };

  for (int i = 0; i < total; ++i) {
    const auto & seg = plan->segments[static_cast<size_t>(i)];
    const std::string label = stepLabel(seg.step, i);
    if (task_cancel_.load()) {
      finish(false, "CANCELED", "canceled before " + label + ": " + cancel_reason(), i);
      return;
    }
    TaskProgress p;
    p.step = i;
    p.total = total;
    p.name = seg.step.name.empty() ? stepTypeName(seg.step.type) : seg.step.name;
    p.step_planned = seg.duration;
    p.progress = arm_api2::logic::taskProgress(durations, static_cast<size_t>(i), 0.0);
    auto report = [&](const std::string & status, double sp, double elapsed) {
        p.status = status;
        p.step_progress = sp;
        p.step_elapsed = elapsed;
        p.progress = arm_api2::logic::taskProgress(durations, static_cast<size_t>(i), sp);
        if (fb) {fb(p);}
      };

    const uint8_t type = seg.step.type;
    if (isMotion(type)) {
      if (seg.trajectory.joint_trajectory.points.empty()) {continue;}   // nothing to move
      report("executing", 0.0, 0.0);
      auto traj = seg.trajectory;
      auto & jt = traj.joint_trajectory;
      // The plan starts where the robot was (first segment) or where the previous segment
      // ended. Refuse when the robot is elsewhere (moved since planning, or the previous
      // segment ended off target); within tolerance, start from the measured joints.
      std::unordered_map<std::string, double> js;
      if (!latestJoints(js, 0.5)) {
        finish(false, "NO_JOINT_STATE", label + ": no recent joint_states", i);
        return;
      }
      double err = 0.0;
      std::string joint;
      if (!arm_api2::logic::jointError(js, jt.joint_names, jt.points.front().positions, err,
        joint))
      {
        finish(false, "NO_JOINT_STATE", label + ": joint_states miss joint " + joint, i);
        return;
      }
      if (err > plan_start_tolerance_rad_) {
        finish(false, "START_STATE_MISMATCH", label + ": robot is " +
          arm_api2::logic::fmtRad(err) + " rad from the plan start (joint " + joint +
          ", plans.start_tolerance_rad " + arm_api2::logic::fmtRad(plan_start_tolerance_rad_) +
          "); replan", i);
        return;
      }
      for (size_t j = 0; j < jt.joint_names.size(); ++j) {
        jt.points.front().positions[j] = js[jt.joint_names[j]];
      }
      motionBegin("task " + label, "task");
      motionPlanned(seg.planner, 0.0, NAN, traj, nullptr);
      std::string code;
      const bool ok = runExecutionBlocking(traj,
          [&](double elapsed, double planned) {
            const auto pr = arm_api2::logic::execProgress(elapsed, planned);
            report("executing", pr.progress, elapsed);
          }, owner, code);
      if (!ok) {
        if (task_cancel_.load()) {
          finish(false, "CANCELED", label + ": " + cancel_reason() + " (" + code + ")", i);
        } else {
          finish(false, arm_api2::logic::execErrorCode(false, code), label + ": " + code, i);
        }
        return;
      }
      report("executing", 1.0, seg.duration);
      continue;
    }

    switch (type) {
      case TaskStepMsg::GRIPPER_OPEN:
      case TaskStepMsg::GRIPPER_CLOSE:
      case TaskStepMsg::GRIPPER_POSITION: {
          report("gripper", 0.0, 0.0);
          const double pos = type == TaskStepMsg::GRIPPER_OPEN ? 0.0 :
            type == TaskStepMsg::GRIPPER_CLOSE ? 0.8 : seg.step.gripper_position;
          const double effort = seg.step.gripper_max_effort > 0.0 ?
            seg.step.gripper_max_effort : 140.0;
          if (!sendGripperCmd(pos, effort)) {
            finish(false, "GRIPPER_FAILED", label + ": gripper command failed", i);
            return;
          }
          if (gripper_state_pub_ && type != TaskStepMsg::GRIPPER_POSITION) {
            std_msgs::msg::String m;
            m.data = type == TaskStepMsg::GRIPPER_OPEN ? "open" : "closed";
            gripper_state_pub_->publish(m);
          }
          break;
        }
      case TaskStepMsg::ATTACH: {
          report("attach", 0.0, 0.0);
          std::string msg;
          const std::string link = seg.step.attach_link.empty() ? EE_LINK_NAME :
            seg.step.attach_link;
          if (!applyAttach(seg.step.object_id, link, touchLinksFor(link, seg.step.touch_links),
            msg))
          {
            finish(false, "SCENE_UPDATE_FAILED", label + ": " + msg, i);
            return;
          }
          break;
        }
      case TaskStepMsg::DETACH: {
          report("detach", 0.0, 0.0);
          std::string msg;
          if (!applyDetach(seg.step.object_id, false, msg)) {
            finish(false, "SCENE_UPDATE_FAILED", label + ": " + msg, i);
            return;
          }
          break;
        }
      case TaskStepMsg::WAIT: {
          const auto t0 = std::chrono::steady_clock::now();
          while (true) {
            const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
              .count();
            if (el >= seg.step.duration_sec) {break;}
            if (task_cancel_.load() || !rclcpp::ok()) {
              finish(false, "CANCELED", label + ": " + cancel_reason(), i);
              return;
            }
            report("waiting", seg.step.duration_sec > 0 ? el / seg.step.duration_sec : 1.0, el);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }
          break;
        }
      default:
        break;   // SET_SPEED: planning only
    }
    report("done", 1.0, seg.duration);
  }
  finish(true, "SUCCESS", "executed " + std::to_string(total) + " steps", -1);
}

/* ------------------------------------------------------------------ action glue */

template<class ActionT>
typename rclcpp_action::Server<ActionT>::SharedPtr m2Iface::makeTaskActionServer(
  const std::string & name,
  std::function<void(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)> on_accept)
{
  return rclcpp_action::create_server<ActionT>(this, name,
           [this, name](const rclcpp_action::GoalUUID &,
           std::shared_ptr<const typename ActionT::Goal>) {
             if (robotState != JOINT_TRAJ_CTL && robotState != CART_TRAJ_CTL) {
               RCLCPP_ERROR_STREAM(this->get_logger(), name <<
                 ": rejected, set JOINT_TRAJ_CTL or CART_TRAJ_CTL first");
               return rclcpp_action::GoalResponse::REJECT;
             }
             if (motionBusy()) {
               RCLCPP_ERROR_STREAM(this->get_logger(), name << ": rejected, robot is busy");
               return rclcpp_action::GoalResponse::REJECT;
             }
             return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
           },
           [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> gh) {
             // running: stop it (the done callback reports canceled); queued: run() sees it
             if (task_active_.load() && task_owner_.load() == gh.get()) {
               cancelTask("action goal canceled");
             }
             return rclcpp_action::CancelResponse::ACCEPT;
           },
           [this, on_accept](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> gh) {
             task_jobs_.push_back(TaskJob{
      [on_accept, gh]() {on_accept(gh);},
      [this, gh](const std::string & why) {
        if (!gh->is_active()) {return;}
        auto r = std::make_shared<typename ActionT::Result>();
        r->success = false;
        r->error_code = "ABORTED";
        r->message = why;
        gh->abort(r);
        RCLCPP_WARN_STREAM(this->get_logger(), "Aborted a queued task goal: " << why);
      }});
           });
}

void m2Iface::processTaskJobs()
{
  auto jobs = std::move(task_jobs_);
  task_jobs_.clear();
  for (auto & j : jobs) {
    if (motionBusy()) {
      j.abort("robot is busy (another motion or task is running)");
      continue;
    }
    j.run();
  }
}

template<class ActionT>
void m2Iface::runStoredPlanGoal(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> gh, const std::string & label,
  const std::string & plan_id)
{
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;
  geometry_msgs::msg::PoseStamped no_pose;
  no_pose.header.frame_id = PLANNING_FRAME;
  auto finish_goal = [this, gh, label, no_pose](bool ok, const std::string & code,
    const std::string & msg, int failed, const std::string & id) {
      publishActionStatus(label, ok, code, msg, no_pose, 0.0);
      if (!gh->is_active()) {return;}
      auto r = std::make_shared<Result>();
      r->success = ok;
      r->error_code = code;
      r->message = msg;
      if constexpr (HasFailedStep<Result>::value) {r->failed_step = failed;}
      if constexpr (HasPlanId<Result>::value) {r->plan_id = id;}
      (void)failed;
      (void)id;
      if (ok) {
        gh->succeed(r);
      } else if (gh->is_canceling()) {
        gh->canceled(r);
      } else {
        gh->abort(r);
      }
    };
  if (gh->is_canceling()) {
    finish_goal(false, "CANCELED", "canceled before start", -1, plan_id);
    return;
  }
  std::string why;
  auto plan = plan_store_.get(plan_id, why);
  if (!plan) {finish_goal(false, "UNKNOWN_PLAN", why, -1, plan_id); return;}
  auto fb = [gh](const TaskProgress & p) {
      if (!gh->is_active()) {return;}
      auto f = std::make_shared<Feedback>();
      f->status = gh->is_canceling() ? "stopping" : p.status;
      f->progress = p.progress;
      if constexpr (HasCurrentStep<Feedback>::value) {
        f->current_step = p.step;
        f->total_steps = p.total;
        f->current_step_name = p.name;
        f->step_progress = p.step_progress;
      } else {
        f->elapsed_sec = p.step_elapsed;
        f->planned_duration_sec = p.step_planned;
        f->remaining_sec = std::max(0.0, p.step_planned - p.step_elapsed);
      }
      gh->publish_feedback(f);
    };
  auto done = [finish_goal, plan_id](bool ok, const std::string & code, const std::string & msg,
    int failed) {finish_goal(ok, code, msg, failed, plan_id);};
  if (planOnly) {
    finish_goal(true, "SUCCESS", "plan only mode; not executing (" + plan_id + ")", -1, plan_id);
    return;
  }
  if (!startTask(plan, gh.get(), fb, done, why)) {
    finish_goal(false, "EXECUTE_FAILED", "not started: " + why, -1, plan_id);
    return;
  }
  publishActionStatus(label, true, "SUCCESS", "Plan accepted; executing " + plan_id, no_pose,
    0.0, false);
}

template<class ActionT>
void m2Iface::runPlannedGoal(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> gh, const std::string & label,
  const std::vector<TaskStep> & steps)
{
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;
  geometry_msgs::msg::PoseStamped no_pose;
  no_pose.header.frame_id = PLANNING_FRAME;
  if (gh->is_canceling()) {
    auto r = std::make_shared<Result>();
    r->success = false;
    r->error_code = "CANCELED";
    r->message = "canceled before planning";
    gh->canceled(r);
    return;
  }
  motionBegin(label, "action");
  const int total = static_cast<int>(steps.size());
  auto res = planTask(steps, [gh, total](int i, const std::string & name) {
        auto f = std::make_shared<Feedback>();
        f->status = "planning";
        if constexpr (HasCurrentStep<Feedback>::value) {
          f->current_step = i;
          f->total_steps = total;
          f->current_step_name = name;
        }
        (void)i; (void)name; (void)total;
        gh->publish_feedback(f);
      });
  if (!res.ok) {
    publishActionStatus(label, false, res.code, res.message, no_pose, res.plan_time_sec);
    auto r = std::make_shared<Result>();
    r->success = false;
    r->error_code = res.code;
    r->message = res.message;
    if constexpr (HasFailedStep<Result>::value) {r->failed_step = res.failed_step;}
    gh->abort(r);
    return;
  }
  const std::string id = plan_store_.put(res.plan);
  runStoredPlanGoal<ActionT>(gh, label, id);
}

void m2Iface::plan_task_cb(
  const std::shared_ptr<arm_api2_msgs::srv::PlanTask::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::PlanTask::Response> res)
{
  res->success = false;
  res->failed_step = -1;
  if (motionBusy()) {
    res->error_code = "BUSY";
    res->message = "robot is moving; plan when it stands still (plans start at the current state)";
    return;
  }
  const auto r = planTask(req->steps);
  res->error_code = r.code;
  res->message = r.message;
  res->failed_step = r.failed_step;
  if (!r.ok) {return;}
  res->success = true;
  res->plan_id = plan_store_.put(r.plan);
  res->start_state = r.plan->start_state;
  res->planned_duration_sec = r.plan->duration;
  for (const auto & s : r.plan->segments) {
    arm_api2_msgs::msg::TaskSegment m;
    m.step_name = s.step.name;
    m.step_type = s.step.type;
    m.step_index = s.index;
    m.trajectory = s.trajectory;
    m.planner = s.planner;
    m.duration_sec = s.duration;
    res->segments.push_back(std::move(m));
  }
  res->message += " -> " + res->plan_id;
}

void m2Iface::init_task_interfaces()
{
  if (config["plans"]) {
    if (config["plans"]["ttl_sec"]) {plan_store_.setTtl(config["plans"]["ttl_sec"].as<double>());}
    if (config["plans"]["start_tolerance_rad"]) {
      plan_start_tolerance_rad_ = config["plans"]["start_tolerance_rad"].as<double>();
    }
  }
  plan_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  std::string ns = MOVE_GROUP_NS;
  while (!ns.empty() && ns.back() == '/') {ns.pop_back();}
  const std::string move_action = ns.empty() ? "move_action" : ns + "/move_action";
  move_group_plan_client_ = rclcpp_action::create_client<MoveGroupAction>(node_, move_action,
      plan_cb_group_);
  const std::string scene_srv = ns.empty() ? "get_planning_scene" : ns + "/get_planning_scene";
  get_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(scene_srv,
      rclcpp::ServicesQoS(), plan_cb_group_);

  const auto plan_name = interfaceName("srv", "plan", "arm/plan");
  plan_task_srv_ = this->create_service<arm_api2_msgs::srv::PlanTask>(plan_name,
      std::bind(&m2Iface::plan_task_cb, this, _1, _2));

  if (!enable_actions) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Task service '" << plan_name <<
      "' (actions disabled in simple mode)");
    return;
  }
  const auto task_name = interfaceName("action", "execute_task", "arm/execute_task");
  const auto exec_name = interfaceName("action", "execute_plan", "arm/execute_plan");
  const auto rel_name = interfaceName("action", "move_relative", "arm/move_relative");
  const auto named_name = interfaceName("action", "move_to_named", "arm/move_to_named");
  execute_task_as_ = makeTaskActionServer<ExecuteTask>(task_name,
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTask>> gh) {
        runPlannedGoal<ExecuteTask>(gh, "execute_task", gh->get_goal()->steps);
      });
  execute_plan_as_ = makeTaskActionServer<ExecutePlan>(exec_name,
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecutePlan>> gh) {
        motionBegin("execute_plan", "action");
        runStoredPlanGoal<ExecutePlan>(gh, "execute_plan", gh->get_goal()->plan_id);
      });
  move_relative_as_ = makeTaskActionServer<MoveRelative>(rel_name,
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveRelative>> gh) {
        TaskStep s;
        s.type = TaskStep::MOVE_RELATIVE;
        s.name = "move_relative";
        s.relative = gh->get_goal()->relative;
        runPlannedGoal<MoveRelative>(gh, "move_relative", {s});
      });
  move_to_named_as_ = makeTaskActionServer<MoveToNamed>(named_name,
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>> gh) {
        TaskStep s;
        s.type = TaskStep::MOVE_NAMED;
        s.name = gh->get_goal()->name;
        s.named_pose = gh->get_goal()->name;
        runPlannedGoal<MoveToNamed>(gh, "move_to_named", {s});
      });
  RCLCPP_INFO_STREAM(this->get_logger(), "Tasks: service '" << plan_name << "', actions '" <<
    task_name << "', '" << exec_name << "', '" << rel_name << "', '" << named_name <<
    "' (move_group '" << move_action << "', start tolerance " << plan_start_tolerance_rad_ <<
    " rad)");
}
