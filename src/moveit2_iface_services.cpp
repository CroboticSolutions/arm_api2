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

/*      Title       : moveit2_iface_services.cpp
 *      Project     : arm_api2
 *      Created     : 06/08/2025
 *      Author      : Filip Zoric
 *      Contributors: Edgar Welte
 *      Description : Parameter, collision, gripper and reachability/cartesian-check service callbacks.
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

namespace
{
Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  if (std::isfinite(q.norm()) && q.norm() > 1e-9) {
    q.normalize();
    out.linear() = q.toRotationMatrix();
  }
  out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return out;
}
}  // namespace

void m2Iface::set_vel_acc_cb(
  const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Response> res)
{
  if (!std::isfinite(req->max_vel) || !std::isfinite(req->max_acc) ||
    req->max_vel < 0 || req->max_acc < 0 || req->max_vel > 1 || req->max_acc > 1)
  {
    res->success = false;
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Velocity and acceleration must be in the range [0, 1]!");
    return;
  }
  max_vel_scaling_factor = float(req->max_vel);
  max_acc_scaling_factor = float(req->max_acc);
  res->success = true;
  RCLCPP_INFO_STREAM(this->get_logger(),
                           "Set velocity and acceleration to " << max_vel_scaling_factor << " " <<
    max_acc_scaling_factor);

}

void m2Iface::set_eelink_cb(
  const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res)
{
  if (!m_moveGroupPtr || !m_moveGroupPtr->getRobotModel() || req->value.empty() ||
    !m_moveGroupPtr->getRobotModel()->hasLinkModel(req->value))
  {
    res->success = false;
    RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Cannot set end effector link: '" << req->value <<
      "' is not in the robot model");
    return;
  }
  if (!m_moveGroupPtr->setEndEffectorLink(req->value)) {
    res->success = false;
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "MoveGroup rejected end effector link '" << req->value << "'");
    return;
  }
  EE_LINK_NAME = req->value;
  res->success = true;
  RCLCPP_INFO_STREAM(this->get_logger(), "Set end effector link to " << EE_LINK_NAME);
}

void m2Iface::set_planner_cb(
  const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res)
{
  std::string planner_string = req->value;

    // Parse the planner string (format: "planner_id_type", e.g., "pilz_LIN", "ompl_RRT")
  size_t underscore_pos = planner_string.find('_');
  if (underscore_pos == std::string::npos) {
    res->success = false;
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Invalid planner format. Expected 'planner_type' (e.g., 'pilz_LIN', 'ompl_RRT')");
    return;
  }

  std::string planner_prefix = planner_string.substr(0, underscore_pos);
  std::string planner_type = planner_string.substr(underscore_pos + 1);

    // Map short names to full planner IDs
  std::string planner_id;
  if (planner_prefix == "pilz") {
    planner_id = "pilz_industrial_motion_planner";
  } else if (planner_prefix == "ompl") {
    planner_id = "ompl";
  } else {
    res->success = false;
    RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Unknown planner: " << planner_prefix <<
      ". Supported: 'pilz', 'ompl'");
    return;
  }

    // Set the planner
  try {
    m_moveGroupPtr->setPlanningPipelineId(planner_id);
    m_moveGroupPtr->setPlannerId(planner_type);

    current_planner_id_ = planner_id;
    current_planner_type_ = planner_type;

    res->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(),
                                   "Successfully set planner to: " << planner_id << " / " <<
      planner_type);
  } catch (const std::exception & e) {
    res->success = false;
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set planner: " << e.what());
  }
}

void m2Iface::set_plan_only_cb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  planOnly = req->data;
  res->success = true;
  RCLCPP_INFO_STREAM(this->get_logger(), "Set plan only to " << req->data);
}

void m2Iface::add_collision_object_cb(
  const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Adding collision object: %s", req->id.c_str());

    // Validate primitive type
  if (req->primitive_type < 1 || req->primitive_type > 3) {
    res->success = false;
    res->message = "Invalid primitive type. Use 1 for BOX, 2 for SPHERE, 3 for CYLINDER";
    RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
    return;
  }

    // Validate dimensions based on primitive type
  if (req->primitive_type == shape_msgs::msg::SolidPrimitive::BOX && req->dimensions.size() != 3) {
    res->success = false;
    res->message = "BOX requires 3 dimensions [x, y, z]";
    RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
    return;
  }
  if (req->primitive_type == shape_msgs::msg::SolidPrimitive::SPHERE &&
    req->dimensions.size() != 1)
  {
    res->success = false;
    res->message = "SPHERE requires 1 dimension [radius]";
    RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
    return;
  }
  if (req->primitive_type == shape_msgs::msg::SolidPrimitive::CYLINDER &&
    req->dimensions.size() != 2)
  {
    res->success = false;
    res->message = "CYLINDER requires 2 dimensions [height, radius]";
    RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
    return;
  }

    // Create collision object
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = m_moveGroupPtr->getPlanningFrame();
  collision_object.id = req->id;

    // Define primitive shape
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = req->primitive_type;
    // Copy dimensions from vector to bounded vector
  primitive.dimensions.clear();
  for (const auto & dim : req->dimensions) {
    primitive.dimensions.push_back(dim);
  }

    // Define pose
  geometry_msgs::msg::Pose box_pose;
  box_pose.position = req->position;
  box_pose.orientation = req->orientation;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

    // Add to planning scene; synchronous (apply_planning_scene), so a plan requested right
    // after this call already sees the object, and a failure is reported.
  if (!m_planningSceneInterface->applyCollisionObject(collision_object)) {
    res->success = false;
    res->message = "move_group did not apply the collision object";
    RCLCPP_ERROR(this->get_logger(), "Collision object '%s' was not applied", req->id.c_str());
    return;
  }

  res->success = true;
  res->message = "Collision object added successfully";
  RCLCPP_INFO(this->get_logger(), "Collision object '%s' added to planning scene", req->id.c_str());
}

void m2Iface::open_gripper_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  if (gripper_none_) {
    res->success = false;
    res->message = "no gripper on this robot/tool (gripper: none)";
    return;
  }
  const bool success = sendGripperCmd(0.0);
  res->success = success;
  res->message = success ? "ok" : "failed";
  if (success && gripper_state_pub_) {
    std_msgs::msg::String state_msg;
    state_msg.data = "open";
    gripper_state_pub_->publish(state_msg);
  }
}

void m2Iface::close_gripper_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  if (gripper_none_) {
    res->success = false;
    res->message = "no gripper on this robot/tool (gripper: none)";
    return;
  }
  const bool success = sendGripperCmd(0.8);
  res->success = success;
  res->message = success ? "ok" : "failed";
  if (success && gripper_state_pub_) {
    std_msgs::msg::String state_msg;
    state_msg.data = "closed";
    gripper_state_pub_->publish(state_msg);
  }
}

rclcpp_action::GoalResponse m2Iface::gripper_control_goal_cb(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for gripper control!");
  (void)uuid;
  (void)goal;
  if (gripper_none_) {
    RCLCPP_ERROR(this->get_logger(), "Gripper goal rejected: no gripper (gripper: none)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::gripper_control_cancel_cb(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>>
  goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::gripper_control_accepted_cb(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
{
  m_gripperControlGoalHandle_ = goal_handle;
  received_gripper_cmd_ = true;
}

void m2Iface::check_reachability_cb(
  const std::shared_ptr<arm_api2_msgs::srv::CheckReachability::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::CheckReachability::Response> res)
{
  res->reachable = false;
  res->in_collision = false;
  res->ik_solutions_found = 0;
  res->reason = "";

  if (!moveGroupInit || !robotModelInit || !pSceneMonitorInit) {
    res->reason = "MoveIt not fully initialized";
    return;
  }

    /* Resolve the requested pose into PLANNING_FRAME via TF when needed.
     * Identical guard to pose_cmd_cb: empty frame_id is treated as PLANNING_FRAME. */
  geometry_msgs::msg::PoseStamped pose_in_planning_frame;
  const std::string source_frame = req->pose.header.frame_id;
  if (source_frame.empty() || source_frame == PLANNING_FRAME) {
    pose_in_planning_frame = req->pose;
    pose_in_planning_frame.header.frame_id = PLANNING_FRAME;
  } else {
    try {
      pose_in_planning_frame = tf_buffer_->transform(req->pose, PLANNING_FRAME);
    } catch (const tf2::TransformException & ex) {
      res->reason = std::string("TF: cannot transform '") + source_frame +
        "' -> '" + PLANNING_FRAME + "': " + ex.what();
      return;
    }
  }

    /* Snapshot the planning scene so we evaluate against the live world,
     * including any user-added collision objects. */
  if (!m_pSceneMonitorPtr || !m_pSceneMonitorPtr->getPlanningScene()) {
    res->reason = "Planning scene unavailable";
    return;
  }
  planning_scene_monitor::LockedPlanningSceneRO ls(m_pSceneMonitorPtr);

  moveit::core::RobotState seed_state(ls->getCurrentState());
  const moveit::core::JointModelGroup *jmg = seed_state.getJointModelGroup(PLANNING_GROUP);
  if (jmg == nullptr) {
    res->reason = "Joint model group '" + PLANNING_GROUP + "' not found";
    return;
  }

  const unsigned int attempts = (req->ik_attempts ==
    0) ? 4u : static_cast<unsigned int>(req->ik_attempts);
  const double per_attempt_timeout = (req->ik_timeout_sec > 0.0) ? req->ik_timeout_sec : 0.05;

  unsigned int ik_solutions = 0;
  bool any_in_collision = false;
  std::string last_collision_reason;

    /* Try N independent IK seeds; for each successful IK, run a state-validity
     * check (collision + bounds) against the locked planning scene. We accept
     * the first collision-free hit and short-circuit, but continue probing for
     * `ik_solutions_found` to give the GUI a confidence signal. */
  for (unsigned int i = 0; i < attempts; ++i) {
    moveit::core::RobotState attempt_state = seed_state;
    if (i > 0) {
      attempt_state.setToRandomPositions(jmg);
    }

    const bool ik_ok = attempt_state.setFromIK(
            jmg,
            pose_in_planning_frame.pose,
            EE_LINK_NAME,
            per_attempt_timeout);
    if (!ik_ok) {continue;}

    attempt_state.update();

    collision_detection::CollisionRequest col_req;
    col_req.contacts = false;
    col_req.distance = false;
    collision_detection::CollisionResult col_res;
    ls->checkCollision(col_req, col_res, attempt_state);
    if (col_res.collision) {
      any_in_collision = true;
      last_collision_reason = "Goal state collides with environment or self";
      continue;
    }

    if (!ls->isStateFeasible(attempt_state, false)) {
      any_in_collision = true;
      last_collision_reason = "Goal state violates planning-scene state validity";
      continue;
    }

    ++ik_solutions;
  }

  res->ik_solutions_found = static_cast<uint8_t>(std::min<unsigned int>(ik_solutions, 255u));
  if (ik_solutions > 0) {
    res->reachable = true;
    res->in_collision = false;
    res->reason = "OK";
    return;
  }

  res->reachable = false;
  res->in_collision = any_in_collision;
  if (any_in_collision) {
    res->reason = last_collision_reason;
  } else {
    res->reason = "No IK solution found";
  }
}

void m2Iface::check_cartesian_path_cb(
  const std::shared_ptr<arm_api2_msgs::srv::CheckCartesianPath::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::CheckCartesianPath::Response> res)
{
  res->full_path = false;
  res->fraction = 0.0;
  res->trajectory_points = 0;
  res->start_reachable = false;
  res->in_collision = false;
  res->start_ik_solutions_found = 0;
  res->start_configs_checked = 0;
  res->reason = "";

  if (!moveGroupInit || !robotModelInit || !pSceneMonitorInit) {
    res->reason = "MoveIt not fully initialized";
    return;
  }
  if (!m_pSceneMonitorPtr || !m_pSceneMonitorPtr->getPlanningScene()) {
    res->reason = "Planning scene unavailable";
    return;
  }

  auto resolve_pose = [&](const geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::PoseStamped & out,
    std::string & reason) -> bool {
      const std::string source_frame = pose.header.frame_id;
      if (source_frame.empty() || source_frame == PLANNING_FRAME) {
        out = pose;
        out.header.frame_id = PLANNING_FRAME;
        return true;
      }
      try {
        out = tf_buffer_->transform(pose, PLANNING_FRAME);
        return true;
      } catch (const tf2::TransformException & ex) {
        reason = std::string("TF: cannot transform '") + source_frame +
          "' -> '" + PLANNING_FRAME + "': " + ex.what();
        return false;
      }
    };

  geometry_msgs::msg::PoseStamped start_pose;
  geometry_msgs::msg::PoseStamped target_pose;
  std::string tf_reason;
  if (!resolve_pose(req->start_pose, start_pose, tf_reason) ||
    !resolve_pose(req->target_pose, target_pose, tf_reason))
  {
    res->reason = tf_reason;
    return;
  }

  planning_scene_monitor::LockedPlanningSceneRO ls(m_pSceneMonitorPtr);
  moveit::core::RobotState scene_state(ls->getCurrentState());
  const moveit::core::JointModelGroup *jmg = scene_state.getJointModelGroup(PLANNING_GROUP);
  if (jmg == nullptr) {
    res->reason = "Joint model group '" + PLANNING_GROUP + "' not found";
    return;
  }
  const moveit::core::LinkModel *link = scene_state.getLinkModel(EE_LINK_NAME);
  if (link == nullptr) {
    res->reason = "End-effector link '" + EE_LINK_NAME + "' not found";
    return;
  }

  auto state_valid = [&](moveit::core::RobotState *state,
    const moveit::core::JointModelGroup *group,
    const double *values) -> bool {
      if (group != nullptr && values != nullptr) {
        state->setJointGroupPositions(group, values);
      }
      state->update();
      if (req->ignore_collisions) {
        return ls->isStateFeasible(*state, false);
      }
      return ls->isStateValid(*state, PLANNING_GROUP, false);
    };

  const unsigned int attempts = (req->ik_attempts ==
    0) ? 4u : static_cast<unsigned int>(req->ik_attempts);
  const double per_attempt_timeout = (req->ik_timeout_sec > 0.0) ? req->ik_timeout_sec : 0.05;

    // Distinct valid start configurations. With check_all_start_configs the
    // Cartesian segment is interpolated from EVERY one of them (worst fraction
    // wins): a joint-space plan that moves the arm to start_pose first may
    // settle in any of these, so a single-seed pass is not a guarantee.
  std::vector<moveit::core::RobotState> start_states;
  unsigned int start_solutions = 0;
  bool any_in_collision = false;
  const std::vector<const moveit::core::JointModel *> & joint_models = jmg->getActiveJointModels();
  auto config_is_new = [&](const moveit::core::RobotState & candidate) -> bool {
      for (const auto & existing : start_states) {
        double max_delta = 0.0;
        for (const auto *jm : joint_models) {
          const double d = std::fabs(candidate.getVariablePosition(jm->getFirstVariableIndex()) -
                                                                                existing.
              getVariablePosition(
                                                                                        jm->
                getFirstVariableIndex()));
          max_delta = std::max(max_delta, d);
        }
        if (max_delta < 0.05) {
          return false;
        }
      }
      return true;
    };
  for (unsigned int i = 0; i < attempts; ++i) {
    moveit::core::RobotState attempt_state(scene_state);
    if (i > 0) {
      attempt_state.setToRandomPositions(jmg);
    }
    if (!attempt_state.setFromIK(jmg, start_pose.pose, EE_LINK_NAME, per_attempt_timeout)) {
      continue;
    }
    attempt_state.update();

    const bool valid = state_valid(&attempt_state, jmg, nullptr);
    if (!valid) {
      if (!req->ignore_collisions && ls->isStateColliding(attempt_state, PLANNING_GROUP, false)) {
        any_in_collision = true;
      }
      continue;
    }
    ++start_solutions;
    if (config_is_new(attempt_state)) {
      start_states.push_back(attempt_state);
      if (!req->check_all_start_configs) {
        break;
      }
    }
  }

  res->start_ik_solutions_found = static_cast<uint8_t>(std::min<unsigned int>(start_solutions,
    255u));
  if (start_states.empty()) {
    res->start_reachable = false;
    res->in_collision = any_in_collision;
    res->reason =
      any_in_collision ? "Start pose collides with environment or self" :
      "No IK solution found for start pose";
    return;
  }
  res->start_reachable = true;

  const double eef_step = (req->eef_step > 0.0) ? req->eef_step : eef_step_;
  moveit::core::MaxEEFStep max_step(eef_step);
  moveit::core::CartesianPrecision precision;

  double worst_fraction = 1.0;
  uint32_t worst_points = 0;
  for (auto & start : start_states) {
    std::vector<moveit::core::RobotStatePtr> trajectory;
    const auto percentage = moveit::core::CartesianInterpolator::computeCartesianPath(
            &start,
            jmg,
            trajectory,
            link,
            poseMsgToEigen(target_pose.pose),
            true,
            max_step,
            precision,
            state_valid,
            kinematics::KinematicsQueryOptions());
    if (percentage.value <= worst_fraction) {
      worst_fraction = percentage.value;
      worst_points = static_cast<uint32_t>(trajectory.size());
    }
  }

  res->start_configs_checked = static_cast<uint8_t>(std::min<size_t>(start_states.size(), 255u));
  res->fraction = worst_fraction;
  res->trajectory_points = worst_points;
  res->full_path = res->fraction >= 0.999;
  if (res->full_path) {
    res->reason = "OK";
  } else if (res->fraction > 0.0) {
    res->reason = "Cartesian path is partial (fraction=" + std::to_string(res->fraction) +
      ", worst of " + std::to_string(start_states.size()) + " start configs)";
  } else {
    res->reason = "Cartesian path could not be computed";
  }
}
