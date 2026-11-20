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

#include "arm_api2/task_logic.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>

namespace arm_api2
{
namespace logic
{

std::optional<RelFrame> parseRelFrame(const std::string & s_in)
{
  std::string s = s_in;
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  if (s.empty() || s == "tcp" || s == "tool") {return RelFrame::TCP;}
  if (s == "base" || s == "world" || s == "planning") {return RelFrame::BASE;}
  return std::nullopt;
}

Eigen::Quaterniond rpyToQuat(double roll, double pitch, double yaw)
{
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

Eigen::Isometry3d relativeTarget(
  const Eigen::Isometry3d & tcp, RelFrame frame,
  const Eigen::Vector3d & translation, const Eigen::Vector3d & rpy)
{
  const Eigen::Quaterniond q = rpyToQuat(rpy.x(), rpy.y(), rpy.z());
  if (frame == RelFrame::TCP) {
    Eigen::Isometry3d off = Eigen::Isometry3d::Identity();
    off.translate(translation);
    off.rotate(q);
    return tcp * off;
  }
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = q.toRotationMatrix() * tcp.linear();
  out.translation() = tcp.translation() + translation;
  return out;
}

bool validPoseName(const std::string & name)
{
  if (name.empty() || name.size() > 64) {return false;}
  return std::all_of(name.begin(), name.end(), [](unsigned char c) {
             return std::isalnum(c) || c == '_' || c == '-';
           });
}

NamedPoseMap parseNamedPoses(
  const YAML::Node & node, const std::vector<std::string> & group_joints,
  const std::string & source, std::vector<std::string> & errors)
{
  NamedPoseMap out;
  if (!node || node.IsNull()) {return out;}
  if (!node.IsMap()) {
    errors.push_back("named poses: expected a map name -> joints");
    return out;
  }
  for (const auto & kv : node) {
    const std::string name = kv.first.as<std::string>();
    if (!validPoseName(name)) {
      errors.push_back("named pose '" + name + "': invalid name (use [A-Za-z0-9_-])");
      continue;
    }
    NamedPose p;
    p.source = source;
    try {
      if (kv.second.IsSequence()) {
        if (kv.second.size() != group_joints.size()) {
          errors.push_back("named pose '" + name + "': " + std::to_string(kv.second.size()) +
            " values, the group has " + std::to_string(group_joints.size()) + " joints");
          continue;
        }
        p.joint_names = group_joints;
        for (const auto & v : kv.second) {
          p.positions.push_back(v.as<double>());
        }
      } else if (kv.second.IsMap()) {
        for (const auto & jv : kv.second) {
          p.joint_names.push_back(jv.first.as<std::string>());
          p.positions.push_back(jv.second.as<double>());
        }
        if (p.joint_names.empty()) {
          errors.push_back("named pose '" + name + "': no joints");
          continue;
        }
      } else {
        errors.push_back("named pose '" + name + "': expected a list or a joint map");
        continue;
      }
    } catch (const YAML::Exception & e) {
      errors.push_back("named pose '" + name + "': " + e.what());
      continue;
    }
    out[name] = std::move(p);
  }
  return out;
}

std::string dumpNamedPoses(const NamedPoseMap & poses)
{
  YAML::Emitter em;
  em.SetDoublePrecision(6);
  em << YAML::BeginMap;
  for (const auto & kv : poses) {
    em << YAML::Key << kv.first << YAML::Value << YAML::BeginMap;
    for (size_t i = 0; i < kv.second.joint_names.size() && i < kv.second.positions.size(); ++i) {
      em << YAML::Key << kv.second.joint_names[i] << YAML::Value << kv.second.positions[i];
    }
    em << YAML::EndMap;
  }
  em << YAML::EndMap;
  return std::string(
        "# arm_api2 saved named poses (arm/save_named_pose); joint values in rad/m\n") +
         em.c_str() + "\n";
}

bool writeFileAtomic(const std::string & path, const std::string & content, std::string & error)
{
  try {
    const std::filesystem::path p(path);
    if (p.has_parent_path()) {std::filesystem::create_directories(p.parent_path());}
    const std::string tmp = path + ".tmp." + std::to_string(::getpid());
    {
      std::ofstream f(tmp, std::ios::trunc);
      if (!f) {error = "cannot write " + tmp; return false;}
      f << content;
      f.flush();
      if (!f) {error = "write failed: " + tmp; return false;}
    }
    std::filesystem::rename(tmp, p);
  } catch (const std::exception & e) {
    error = e.what();
    return false;
  }
  return true;
}

bool namedPoseToGroup(
  const NamedPose & pose, const std::vector<std::string> & group_joints,
  const std::vector<double> & fallback, std::vector<double> & out, std::string & error)
{
  if (fallback.size() != group_joints.size()) {
    error = "current joint state does not match the group";
    return false;
  }
  out = fallback;
  for (size_t i = 0; i < pose.joint_names.size() && i < pose.positions.size(); ++i) {
    const auto it = std::find(group_joints.begin(), group_joints.end(), pose.joint_names[i]);
    if (it == group_joints.end()) {
      error = "joint '" + pose.joint_names[i] + "' is not in the planning group";
      return false;
    }
    out[static_cast<size_t>(it - group_joints.begin())] = pose.positions[i];
  }
  return true;
}

double taskProgress(const std::vector<double> & durations, size_t index, double step_progress)
{
  if (durations.empty()) {return 0.0;}
  double total = 0.0;
  for (double d : durations) {
    total += std::max(0.0, d);
  }
  step_progress = std::clamp(step_progress, 0.0, 1.0);
  if (total <= 0.0) {
    return std::clamp((static_cast<double>(index) + step_progress) / durations.size(), 0.0, 1.0);
  }
  double done = 0.0;
  for (size_t i = 0; i < index && i < durations.size(); ++i) {
    done += std::max(0.0, durations[i]);
  }
  if (index < durations.size()) {done += std::max(0.0, durations[index]) * step_progress;}
  return std::clamp(done / total, 0.0, 1.0);
}

}  // namespace logic
}  // namespace arm_api2
