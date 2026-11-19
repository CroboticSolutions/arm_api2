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

/*      Title       : motion_log.hpp
 *      Project     : arm_api2
 *      Description : Permanent per-motion log: one JSON line per motion command (planner,
 *                    planning time, Cartesian fraction, planned vs measured duration, final
 *                    joint error, max path deviation, result). Daily files, append only,
 *                    never pruned. No ROS dependency (unit tested).
 */

#ifndef ARM_API2__MOTION_LOG_HPP_
#define ARM_API2__MOTION_LOG_HPP_

#include <cmath>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace arm_api2
{

/* NaN / empty = unknown (written as JSON null). */
struct MotionRecord
{
  bool active = false;           // being filled (not part of the JSON)
  std::string time;              // ISO 8601 UTC, when the command was taken
  std::string robot;             // node namespace ("/" for single robot)
  std::string command;           // move_to_joint, move_to_pose, move_to_pose_path, topic_pose, ...
  std::string source;            // action | topic
  std::string mode;              // arm state (JOINT_TRAJ_CTL, ...)
  std::string frame;
  double vel_scale = NAN;
  double acc_scale = NAN;
  bool has_target_pose = false;
  double target_pose[7] = {0, 0, 0, 0, 0, 0, 1};       // x y z qx qy qz qw
  std::string planner;           // pipeline/planner_id or "cartesian"
  double plan_time_sec = NAN;
  double fraction = NAN;         // Cartesian paths only
  int planned_points = -1;
  double planned_duration_sec = NAN;
  std::vector<std::string> joint_names;
  std::vector<double> start_joints;
  std::vector<double> goal_joints;       // last planned point
  bool executed = false;
  double actual_duration_sec = NAN;
  double final_joint_error = NAN;
  std::string final_error_joint;
  double max_path_deviation = NAN;
  std::string max_deviation_joint;
  bool success = false;
  std::string code;              // plan_status error_code
  std::string reason;
};

std::string jsonEscape(const std::string & s);
std::string toJson(const MotionRecord & r);
/* ISO 8601 UTC with milliseconds for the current wall time. */
std::string isoTimeNow();

class MotionLog
{
public:
/* dir: created if missing. file_tag: goes into the file name (robot namespace). */
  bool open(const std::string & dir, const std::string & file_tag, std::string & error);
  bool enabled() const
  {
    return enabled_;
  }
/* Appends one line (thread safe); rolls over to a new file when the UTC day changes. */
  bool write(const MotionRecord & r);
  std::string currentPath();

private:
  std::string pathFor(const std::string & day) const;
  std::mutex mutex_;
  bool enabled_ = false;
  std::string dir_;
  std::string tag_;
  std::string day_;
  std::ofstream out_;
};

}  // namespace arm_api2

#endif  // ARM_API2__MOTION_LOG_HPP_
