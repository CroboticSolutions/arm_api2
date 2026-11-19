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

/*      Title       : motion_logic.hpp
 *      Project     : arm_api2
 *      Description : Pure decision logic of moveit2_iface (no node, no MoveIt runtime):
 *                    joint error / path deviation / joint continuity checks, result
 *                    codes, arm state machine transitions and execution watchdog.
 *                    Unit tested in test/test_motion_logic.cpp.
 */

#ifndef ARM_API2__MOTION_LOGIC_HPP_
#define ARM_API2__MOTION_LOGIC_HPP_

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace arm_api2
{
namespace logic
{

using JointMap = std::unordered_map<std::string, double>;

/* Radians with 4 decimals, as used in status/result texts. */
std::string fmtRad(double v);

/* Largest |actual - target| over `names`. False if a joint is missing or the sizes differ. */
bool jointError(
  const JointMap & actual, const std::vector<std::string> & names,
  const std::vector<double> & target, double & err, std::string & joint);

/* Distance (max over joints) from `actual` to the closest point of the planned joint-space
 * polyline. Time independent, so controller lag is not counted as deviation. */
bool pathDeviation(
  const JointMap & actual, const trajectory_msgs::msg::JointTrajectory & traj,
  double & dev, std::string & joint);

/* False (with a reason) when a joint moves more than `max_step_rad` between consecutive
 * points (IK branch flip or 2*pi wrap). max_step_rad <= 0 disables the check. */
bool jointPathContinuous(
  const trajectory_msgs::msg::JointTrajectory & traj, double max_step_rad,
  std::string & reason);

/* Stable plan_status code for a finished execution (execution result text -> code). */
std::string execErrorCode(bool ok, const std::string & code);

/* ---- arm state machine ---- */
enum class ArmState { IDLE = 0, JOINT_TRAJ_CTL = 1, CART_TRAJ_CTL = 2, SERVO_CTL = 3 };

const char * stateName(ArmState s);
std::optional<ArmState> parseState(const std::string & name);

enum class StateChange
{
  SAME_STATE,       // nothing to do (queued commands stay)
  SWITCH,           // drop queued commands of the old state, switch
  STOP_AND_SWITCH,       // stop the running trajectory, drop queued commands, switch
  REJECT_MOVING       // refuse: trajectory running and policy is "reject"
};
StateChange decideStateChange(
  ArmState current, ArmState wanted, bool executing, bool stop_on_change);

/* ---- execution supervision ---- */
enum class Watchdog { OK, NO_GOAL_RESPONSE, RESULT_TIMEOUT };
constexpr double kGoalResponseTimeoutSec = 5.0;
/* move_group must answer within 5 s and finish within 2 * planned + 10 s. */
Watchdog execWatchdog(bool goal_accepted, double elapsed_sec, double planned_sec);

struct Progress
{
  double progress = 0.0;       // 0..1, time based
  double remaining = 0.0;       // s, >= 0
};
Progress execProgress(double elapsed_sec, double planned_sec);

enum class GoalCheck { PASS, WAIT, FAIL };
/* After move_group SUCCESS: PASS once within tolerance, WAIT until the settle deadline,
 * then FAIL. Not measured (no fresh joint_states) counts as out of tolerance. */
GoalCheck goalCheck(bool measured, double err, double tolerance, bool deadline_passed);

}  // namespace logic
}  // namespace arm_api2

#endif  // ARM_API2__MOTION_LOGIC_HPP_
