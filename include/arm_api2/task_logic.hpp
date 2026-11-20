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

/*      Title       : task_logic.hpp
 *      Project     : arm_api2
 *
 *      Description : ROS-free logic for tasks: relative TCP moves, named poses
 *                    (YAML), the plan store and start-state checks. Unit tested.
 */

#ifndef ARM_API2__TASK_LOGIC_HPP_
#define ARM_API2__TASK_LOGIC_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "arm_api2/motion_logic.hpp"

namespace arm_api2
{
namespace logic
{

/* ---- relative moves ---- */
enum class RelFrame { TCP, BASE };
/* "tcp" / "" -> TCP, "base" / "world" / "planning" -> BASE, anything else -> nullopt */
std::optional<RelFrame> parseRelFrame(const std::string & s);
/* R = Rz(yaw) * Ry(pitch) * Rx(roll) */
Eigen::Quaterniond rpyToQuat(double roll, double pitch, double yaw);
/* TCP: tcp * T(offset). BASE: translation in base axes, rotation about base axes
 * through the TCP point (the TCP position only moves by the translation). */
Eigen::Isometry3d relativeTarget(
  const Eigen::Isometry3d & tcp, RelFrame frame,
  const Eigen::Vector3d & translation, const Eigen::Vector3d & rpy);

/* ---- named poses ---- */
struct NamedPose
{
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  std::string source;  // saved | config | srdf
};
using NamedPoseMap = std::map<std::string, NamedPose>;

/* A named-pose name: [A-Za-z0-9_-], 1..64 chars. */
bool validPoseName(const std::string & name);

/* Parse a map  name: [v1, ..., vn]  (group joint order)  or  name: {joint: value, ...}.
 * Bad entries are skipped with a message in `errors`. */
NamedPoseMap parseNamedPoses(
  const YAML::Node & node, const std::vector<std::string> & group_joints,
  const std::string & source, std::vector<std::string> & errors);

/* name: {joint: value} for every pose (stable, readable file format). */
std::string dumpNamedPoses(const NamedPoseMap & poses);

/* Write atomically (tmp + rename); creates the directory. False + error on failure. */
bool writeFileAtomic(const std::string & path, const std::string & content, std::string & error);

/* Resolve a pose to full group joint values: joints missing in the pose keep `fallback`.
 * False if the pose names a joint that is not in the group. */
bool namedPoseToGroup(
  const NamedPose & pose, const std::vector<std::string> & group_joints,
  const std::vector<double> & fallback, std::vector<double> & out, std::string & error);

/* ---- plan store: id -> plan, bounded size, expiry ---- */
template<class T>
class PlanStore
{
public:
  using Clock = std::chrono::steady_clock;
  PlanStore(size_t capacity, double ttl_sec)
  : capacity_(capacity), ttl_(ttl_sec) {}

  void setTtl(double ttl_sec) {std::lock_guard<std::mutex> l(m_); ttl_ = ttl_sec;}

  std::string put(std::shared_ptr<T> plan, Clock::time_point now = Clock::now())
  {
    std::lock_guard<std::mutex> l(m_);
    evict(now);
    while (entries_.size() >= capacity_ && !entries_.empty()) {
      // drop the oldest
      auto oldest = entries_.begin();
      for (auto it = entries_.begin(); it != entries_.end(); ++it) {
        if (it->second.created < oldest->second.created) {oldest = it;}
      }
      entries_.erase(oldest);
    }
    const std::string id = "plan_" + std::to_string(++seq_);
    entries_[id] = Entry{std::move(plan), now};
    return id;
  }

  /* nullptr and a reason when unknown or expired. */
  std::shared_ptr<T> get(
    const std::string & id, std::string & why,
    Clock::time_point now = Clock::now())
  {
    std::lock_guard<std::mutex> l(m_);
    const auto it = entries_.find(id);
    if (it == entries_.end()) {why = "unknown plan_id '" + id + "'"; return nullptr;}
    const double age = std::chrono::duration<double>(now - it->second.created).count();
    if (ttl_ > 0.0 && age > ttl_) {
      entries_.erase(it);
      why = "plan '" + id + "' expired (" + std::to_string(static_cast<int>(age)) +
        " s old, plans.ttl_sec " + std::to_string(static_cast<int>(ttl_)) + ")";
      return nullptr;
    }
    return it->second.plan;
  }

  bool erase(const std::string & id)
  {
    std::lock_guard<std::mutex> l(m_);
    return entries_.erase(id) > 0;
  }

  size_t size() const {std::lock_guard<std::mutex> l(m_); return entries_.size();}

private:
  struct Entry
  {
    std::shared_ptr<T> plan;
    Clock::time_point created;
  };
  void evict(Clock::time_point now)
  {
    if (ttl_ <= 0.0) {return;}
    for (auto it = entries_.begin(); it != entries_.end(); ) {
      if (std::chrono::duration<double>(now - it->second.created).count() > ttl_) {
        it = entries_.erase(it);
      } else {
        ++it;
      }
    }
  }
  mutable std::mutex m_;
  std::map<std::string, Entry> entries_;
  size_t capacity_;
  double ttl_;
  uint64_t seq_{0};
};

/* ---- task progress (time weighted) ---- */
/* Overall progress when step `index` (of durations `d`) is `step_progress` done. */
double taskProgress(const std::vector<double> & durations, size_t index, double step_progress);

}  // namespace logic
}  // namespace arm_api2

#endif  // ARM_API2__TASK_LOGIC_HPP_
