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

/*      Title       : motion_log.cpp
 *      Project     : arm_api2
 *      Description : Permanent per-motion JSONL log, see motion_log.hpp.
 */

#include "arm_api2/motion_log.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <sstream>

namespace arm_api2
{

std::string jsonEscape(const std::string & s)
{
  std::string o;
  o.reserve(s.size() + 2);
  for (const unsigned char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          o += buf;
        } else {
          o += static_cast<char>(c);
        }
    }
  }
  return o;
}

namespace
{
std::string num(double v)
{
  if (!std::isfinite(v)) {return "null";}
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}
std::string str(const std::string & s)
{
  return s.empty() ? "null" : "\"" + jsonEscape(s) + "\"";
}
std::string nums(const std::vector<double> & v)
{
  if (v.empty()) {return "null";}
  std::string o = "[";
  for (size_t i = 0; i < v.size(); ++i) {
    o += (i ? "," : "") + num(v[i]);
  }
  return o + "]";
}
std::string strs(const std::vector<std::string> & v)
{
  if (v.empty()) {return "null";}
  std::string o = "[";
  for (size_t i = 0; i < v.size(); ++i) {
    o += (i ? ",\"" : "\"") + jsonEscape(v[i]) + "\"";
  }
  return o + "]";
}
std::string utcDay()
{
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[16];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d", &tm);
  return buf;
}
}  // namespace

std::string isoTimeNow()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  const int ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                now.time_since_epoch()).count() % 1000);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[40];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
  char out[48];
  std::snprintf(out, sizeof(out), "%s.%03dZ", buf, ms);
  return out;
}

std::string toJson(const MotionRecord & r)
{
  std::ostringstream o;
  o << "{\"time\":" << str(r.time) << ",\"robot\":" << str(r.robot) <<
    ",\"command\":" << str(r.command) << ",\"source\":" << str(r.source) <<
    ",\"mode\":" << str(r.mode) << ",\"success\":" << (r.success ? "true" : "false") <<
    ",\"code\":" << str(r.code) << ",\"reason\":" << str(r.reason) <<
    ",\"planner\":" << str(r.planner) << ",\"plan_time_sec\":" << num(r.plan_time_sec) <<
    ",\"fraction\":" << num(r.fraction) <<
    ",\"planned_points\":" << (r.planned_points >= 0 ? std::to_string(r.planned_points) : "null") <<
    ",\"executed\":" << (r.executed ? "true" : "false") <<
    ",\"planned_duration_sec\":" << num(r.planned_duration_sec) <<
    ",\"actual_duration_sec\":" << num(r.actual_duration_sec) <<
    ",\"final_joint_error_rad\":" << num(r.final_joint_error) <<
    ",\"final_error_joint\":" << str(r.final_error_joint) <<
    ",\"max_path_deviation_rad\":" << num(r.max_path_deviation) <<
    ",\"max_deviation_joint\":" << str(r.max_deviation_joint) <<
    ",\"vel_scale\":" << num(r.vel_scale) << ",\"acc_scale\":" << num(r.acc_scale) <<
    ",\"frame\":" << str(r.frame) << ",\"target_pose\":";
  if (r.has_target_pose) {
    o << "[";
    for (int i = 0; i < 7; ++i) {
      o << (i ? "," : "") << num(r.target_pose[i]);
    }
    o << "]";
  } else {
    o << "null";
  }
  o << ",\"joint_names\":" << strs(r.joint_names) << ",\"start_joints\":" <<
    nums(r.start_joints) << ",\"goal_joints\":" << nums(r.goal_joints) << "}";
  return o.str();
}

bool MotionLog::open(const std::string & dir, const std::string & file_tag, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = false;
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    error = "cannot create " + dir + ": " + ec.message();
    return false;
  }
  dir_ = dir;
  tag_ = file_tag;
  day_ = utcDay();
  out_.open(pathFor(day_), std::ios::app);
  if (!out_) {
    error = "cannot open " + pathFor(day_) + " for appending";
    return false;
  }
  enabled_ = true;
  return true;
}

std::string MotionLog::pathFor(const std::string & day) const
{
  return (std::filesystem::path(dir_) / ("motions_" + tag_ + "_" + day + ".jsonl")).string();
}

std::string MotionLog::currentPath()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return enabled_ ? pathFor(day_) : std::string();
}

bool MotionLog::write(const MotionRecord & r)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {return false;}
  const std::string day = utcDay();
  if (day != day_) {
    out_.close();
    day_ = day;
    out_.open(pathFor(day_), std::ios::app);
  }
  out_ << toJson(r) << '\n';
  out_.flush();
  return static_cast<bool>(out_);
}

}  // namespace arm_api2
