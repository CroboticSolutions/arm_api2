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

/*      Title       : motion_logic.cpp
 *      Project     : arm_api2
 *      Description : Pure decision logic of moveit2_iface, see motion_logic.hpp.
 */

#include "arm_api2/motion_logic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace arm_api2
{
namespace logic
{

std::string fmtRad(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.4f", v);
  return buf;
}

bool jointError(
  const JointMap & actual, const std::vector<std::string> & names,
  const std::vector<double> & target, double & err, std::string & joint)
{
  err = 0.0;
  joint.clear();
  if (names.size() != target.size() || names.empty()) {return false;}
  for (size_t i = 0; i < names.size(); ++i) {
    const auto it = actual.find(names[i]);
    if (it == actual.end()) {return false;}
    const double e = std::abs(it->second - target[i]);
    if (e > err || joint.empty()) {err = e; joint = names[i];}
  }
  return true;
}

bool pathDeviation(
  const JointMap & actual, const trajectory_msgs::msg::JointTrajectory & traj,
  double & dev, std::string & joint)
{
  const size_t nj = traj.joint_names.size();
  if (nj == 0 || traj.points.empty()) {return false;}
  std::vector<double> a(nj);
  for (size_t j = 0; j < nj; ++j) {
    const auto it = actual.find(traj.joint_names[j]);
    if (it == actual.end()) {return false;}
    a[j] = it->second;
  }
  dev = std::numeric_limits<double>::infinity();
        // closest point on the joint-space polyline (Euclidean projection, max-abs distance)
  for (size_t i = 0; i < traj.points.size(); ++i) {
    const auto & p0 = traj.points[i].positions;
    const auto & p1 = traj.points[std::min(i + 1, traj.points.size() - 1)].positions;
    if (p0.size() != nj || p1.size() != nj) {return false;}
    double dd = 0.0, ad = 0.0;
    for (size_t j = 0; j < nj; ++j) {
      const double d = p1[j] - p0[j];
      dd += d * d;
      ad += (a[j] - p0[j]) * d;
    }
    const double s = dd > 1e-12 ? std::clamp(ad / dd, 0.0, 1.0) : 0.0;
    double e = 0.0;
    size_t ej = 0;
    for (size_t j = 0; j < nj; ++j) {
      const double q = p0[j] + s * (p1[j] - p0[j]);
      const double ej_v = std::abs(a[j] - q);
      if (ej_v > e) {e = ej_v; ej = j;}
    }
    if (e < dev) {dev = e; joint = traj.joint_names[ej];}
  }
  return true;
}

bool jointPathContinuous(
  const trajectory_msgs::msg::JointTrajectory & jt, double max_step_rad, std::string & reason)
{
  if (max_step_rad <= 0.0) {return true;}
  for (size_t i = 1; i < jt.points.size(); ++i) {
    const auto & before = jt.points[i - 1].positions;
    const auto & after = jt.points[i].positions;
    for (size_t j = 0; j < std::min({before.size(), after.size(), jt.joint_names.size()}); ++j) {
      const double step = std::abs(after[j] - before[j]);
      if (step > max_step_rad) {
        const bool wrap = std::abs(std::remainder(step, 2.0 * M_PI)) <= max_step_rad;
        reason = "joint " + jt.joint_names[j] + " jumps " +
          std::to_string(static_cast<int>(std::lround(step * 180.0 / M_PI))) +
          " deg between path points " + std::to_string(i - 1) + " and " + std::to_string(i) +
          (wrap ? " (full-turn 2*pi wrap)" : " (IK branch flip)") + "; limit " +
          std::to_string(static_cast<int>(std::lround(max_step_rad * 180.0 / M_PI))) +
          " deg per point";
        return false;
      }
    }
  }
  return true;
}

std::string execErrorCode(bool ok, const std::string & code)
{
  if (ok) {return "SUCCESS";}
  if (code.rfind("GOAL_NOT_REACHED", 0) == 0) {return "GOAL_NOT_REACHED";}
  if (code.find("PATH_DEVIATION") != std::string::npos) {return "PATH_DEVIATION";}
  if (code.find("action goal canceled") != std::string::npos) {return "CANCELED";}
  return "EXECUTION_FAILED";
}

namespace
{
constexpr const char * kStateNames[] = {"IDLE", "JOINT_TRAJ_CTL", "CART_TRAJ_CTL", "SERVO_CTL"};
}  // namespace

const char * stateName(ArmState s)
{
  const int i = static_cast<int>(s);
  return (i >= 0 && i < 4) ? kStateNames[i] : "UNKNOWN";
}

std::optional<ArmState> parseState(const std::string & name)
{
  for (int i = 0; i < 4; ++i) {
    if (name == kStateNames[i]) {return static_cast<ArmState>(i);}
  }
  return std::nullopt;
}

StateChange decideStateChange(
  ArmState current, ArmState wanted, bool executing, bool stop_on_change)
{
  if (wanted == current) {return StateChange::SAME_STATE;}
  if (!executing) {return StateChange::SWITCH;}
  return stop_on_change ? StateChange::STOP_AND_SWITCH : StateChange::REJECT_MOVING;
}

Watchdog execWatchdog(bool goal_accepted, double elapsed_sec, double planned_sec)
{
  if (!goal_accepted) {
    return elapsed_sec > kGoalResponseTimeoutSec ? Watchdog::NO_GOAL_RESPONSE : Watchdog::OK;
  }
  return elapsed_sec > 2.0 * planned_sec + 10.0 ? Watchdog::RESULT_TIMEOUT : Watchdog::OK;
}

Progress execProgress(double elapsed_sec, double planned_sec)
{
  Progress p;
  p.remaining = std::max(0.0, planned_sec - elapsed_sec);
  p.progress = planned_sec > 0.0 ? std::clamp(elapsed_sec / planned_sec, 0.0, 1.0) : 0.0;
  return p;
}

GoalCheck goalCheck(bool measured, double err, double tolerance, bool deadline_passed)
{
  if (measured && err <= tolerance) {return GoalCheck::PASS;}
  return deadline_passed ? GoalCheck::FAIL : GoalCheck::WAIT;
}

}  // namespace logic
}  // namespace arm_api2
