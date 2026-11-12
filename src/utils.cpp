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

/*      Title       : utils.cpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : Contains all functional implementations not directly related to MoveIt! or ROS 2
 */


#include "arm_api2/utils.hpp"

#include <algorithm>
#include <cmath>

// TODO: move to utils
std::vector<geometry_msgs::msg::Pose> utils::createCartesianWaypoints(
  geometry_msgs::msg::Pose p1,
  geometry_msgs::msg::Pose p2, int n)
{
  std::vector<geometry_msgs::msg::Pose> result;
  geometry_msgs::msg::Pose pose_;
  if (n <= 1) {
    result.push_back(p1);
    return result;
  }
  double dX = (p2.position.x - p1.position.x) / (n - 1);
  double dY = (p2.position.y - p1.position.y) / (n - 1);
  double dZ = (p2.position.z - p1.position.z) / (n - 1);
    // Set i == 1 because start point doesn't have to be included into waypoint list
    // https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/
  for (int i = 1; i < n; ++i) {
    pose_.position.x = p1.position.x + i * dX;
    pose_.position.y = p1.position.y + i * dY;
    pose_.position.z = p1.position.z + i * dZ;
    pose_.orientation.x = p1.orientation.x;
    pose_.orientation.y = p1.orientation.y;
    pose_.orientation.z = p1.orientation.z;
    pose_.orientation.w = p1.orientation.w;
    result.push_back(pose_);
  }
  return result;
}


// TODO: Move to utils
bool utils::comparePosition(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2)
{
    // Returns false if different positions
  bool x_cond = false;  bool y_cond = false;  bool z_cond = false;
  // 1 mm: only a real repeat of the same command (was 1 cm, which swallowed small nudges).
  double d = 0.001;

  if (std::abs(p1.pose.position.x - p2.pose.position.x) < d) {x_cond = true;}
  if (std::abs(p1.pose.position.y - p2.pose.position.y) < d) {y_cond = true;}
  if (std::abs(p1.pose.position.z - p2.pose.position.z) < d) {z_cond = true;}

  bool cond = x_cond && y_cond && z_cond;
  return cond;
}

// TODO: Move to utils
bool utils::compareOrientation(
  geometry_msgs::msg::PoseStamped p1,
  geometry_msgs::msg::PoseStamped p2)
{
  // True when the orientations differ by less than 0.1 deg (shortest-arc angle; the old
  // per-Euler-angle check used 0.57 deg and split at the +-pi wrap).
  tf2::Quaternion q1(p1.pose.orientation.x, p1.pose.orientation.y, p1.pose.orientation.z,
    p1.pose.orientation.w);
  tf2::Quaternion q2(p2.pose.orientation.x, p2.pose.orientation.y, p2.pose.orientation.z,
    p2.pose.orientation.w);
  if (q1.length2() < 1e-12 || q2.length2() < 1e-12) {
    return q1.length2() < 1e-12 && q2.length2() < 1e-12;
  }
  q1.normalize();
  q2.normalize();
  const double dot = std::min(1.0, std::abs(q1.dot(q2)));
  return 2.0 * std::acos(dot) < 0.1 * M_PI / 180.0;
}

// TODO: move to utils
bool utils::comparePose(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2)
{
  bool position, orientation;
  position = comparePosition(p1, p2);
  orientation = compareOrientation(p1, p2);
  return position && orientation;
}

//TODO: move to utils
geometry_msgs::msg::PoseStamped utils::normalizeOrientation(geometry_msgs::msg::PoseStamped p)
{
  geometry_msgs::msg::PoseStamped p_;
  tf2::Quaternion q(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z,
    p.pose.orientation.w);
  q.normalize();
  p_.pose.orientation.x = q.x();
  p_.pose.orientation.y = q.y();
  p_.pose.orientation.z = q.z();
  p_.pose.orientation.w = q.w();
  p_.pose.position = p.pose.position;
  return p_;
}

// There's also built-in method in tf2 however, didn't manage to get it to work
geometry_msgs::msg::PoseStamped utils::convertIsometryToMsg(Eigen::Isometry3d pose)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = pose.translation().x();
  p.pose.position.y = pose.translation().y();
  p.pose.position.z = pose.translation().z();
  Eigen::Quaterniond q(pose.rotation());
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  return p;
}

std_msgs::msg::String utils::stateToMsg(int state)
{
  const char * stateNames[4] = {"IDLE", "JOINT_TRAJ_CTL", "CART_TRAJ_CTL", "SERVO_CTL"};
  std_msgs::msg::String msg;

  if (state >= 0 && state < 4) {
    msg.data = stateNames[state];
  } else {
    msg.data = "UNKNOWN";
  }

  return msg;
}

std::string utils::moveItErrorCodeToString(const moveit::core::MoveItErrorCode & code)
{
  using moveit_msgs::msg::MoveItErrorCodes;
  switch (code.val) {
    case MoveItErrorCodes::SUCCESS:                                return "SUCCESS";
    case MoveItErrorCodes::FAILURE:                                return "FAILURE";
    case MoveItErrorCodes::PLANNING_FAILED:                        return "PLANNING_FAILED";
    case MoveItErrorCodes::INVALID_MOTION_PLAN:                    return "INVALID_MOTION_PLAN";
    case MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case MoveItErrorCodes::CONTROL_FAILED:                         return "CONTROL_FAILED";
    case MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:           return
        "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case MoveItErrorCodes::TIMED_OUT:                              return "TIMED_OUT";
    case MoveItErrorCodes::PREEMPTED:                              return "PREEMPTED";
    case MoveItErrorCodes::START_STATE_IN_COLLISION:               return "START_STATE_IN_COLLISION";
    case MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:  return
        "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case MoveItErrorCodes::START_STATE_INVALID:                    return "START_STATE_INVALID";
    case MoveItErrorCodes::GOAL_IN_COLLISION:                      return "GOAL_IN_COLLISION";
    case MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:         return
        "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:              return
        "GOAL_CONSTRAINTS_VIOLATED";
    case MoveItErrorCodes::GOAL_STATE_INVALID:                     return "GOAL_STATE_INVALID";
    case MoveItErrorCodes::UNRECOGNIZED_GOAL_TYPE:                 return "UNRECOGNIZED_GOAL_TYPE";
    case MoveItErrorCodes::INVALID_GROUP_NAME:                     return "INVALID_GROUP_NAME";
    case MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:               return "INVALID_GOAL_CONSTRAINTS";
    case MoveItErrorCodes::INVALID_ROBOT_STATE:                    return "INVALID_ROBOT_STATE";
    case MoveItErrorCodes::INVALID_LINK_NAME:                      return "INVALID_LINK_NAME";
    case MoveItErrorCodes::INVALID_OBJECT_NAME:                    return "INVALID_OBJECT_NAME";
    case MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:                return "FRAME_TRANSFORM_FAILURE";
    case MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:         return
        "COLLISION_CHECKING_UNAVAILABLE";
    case MoveItErrorCodes::ROBOT_STATE_STALE:                      return "ROBOT_STATE_STALE";
    case MoveItErrorCodes::SENSOR_INFO_STALE:                      return "SENSOR_INFO_STALE";
    case MoveItErrorCodes::COMMUNICATION_FAILURE:                  return "COMMUNICATION_FAILURE";
    case MoveItErrorCodes::CRASH:                                  return "CRASH";
    case MoveItErrorCodes::ABORT:                                  return "ABORT";
    case MoveItErrorCodes::NO_IK_SOLUTION:                         return "NO_IK_SOLUTION";
    default:
      return "MOVEIT_ERROR_" + std::to_string(static_cast<int>(code.val));
  }
}
