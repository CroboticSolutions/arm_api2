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

/*      Title       : moveit2_iface_config.cpp
 *      Project     : arm_api2
 *      Created     : 06/08/2025
 *      Author      : Filip Zoric
 *      Contributors: Edgar Welte
 *      Description : Robot YAML loading and validation, planning configuration.
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

YAML::Node m2Iface::init_config(std::string yaml_path)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Config yaml path is: " << yaml_path);
  try {
    return YAML::LoadFile(yaml_path);
  } catch (const YAML::BadFile &) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Config file not found or unreadable: " << yaml_path);
    throw std::runtime_error("arm_api2 config not found: " + yaml_path);
  } catch (const YAML::ParserException & e) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Config " << yaml_path << " is not valid YAML: " <<
      e.what());
    throw std::runtime_error("arm_api2 config is not valid YAML: " + yaml_path);
  }
}

namespace
{
bool yamlGet(const YAML::Node & root, const std::string & dotted, YAML::Node & out)
{
  YAML::Node cur;
  cur.reset(root);
  std::stringstream ss(dotted);
  std::string key;
  while (std::getline(ss, key, '.')) {
    if (!cur.IsMap()) {return false;}
    const YAML::Node & c = cur;
    YAML::Node next = c[key];
    if (!next) {return false;}
    cur.reset(next);
  }
  out.reset(cur);
  return true;
}

}  // namespace

void m2Iface::validateConfig()
{
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  enum Kind {STR, STR_OR_NULL, NUM, BOOL, SEQ, MAP};
  auto check = [&](const std::string & dotted, Kind kind, bool required) -> bool {
      YAML::Node n;
      if (!yamlGet(config, dotted, n)) {
        if (required) {errors.push_back(dotted + ": missing (required)");}
        return false;
      }
      try {
        switch (kind) {
          case STR:
            if (!n.IsScalar()) {throw std::runtime_error("expected a string");}
            (void)n.as<std::string>();
            break;
          case STR_OR_NULL:
            if (!n.IsNull()) {(void)n.as<std::string>();}
            break;
          case NUM: (void)n.as<double>(); break;
          case BOOL: (void)n.as<bool>(); break;
          case SEQ: if (!n.IsSequence()) {throw std::runtime_error("expected a list");} break;
          case MAP: if (!n.IsMap()) {throw std::runtime_error("expected a mapping");} break;
        }
      } catch (const std::exception & e) {
        std::string got = n.IsScalar() ? ("'" + n.Scalar() + "'") : "a non-scalar";
        static const char * names[] = {"string", "string", "number", "bool", "list", "mapping"};
        errors.push_back(dotted + ": expected " + names[kind] + ", got " + got);
        return false;
      }
      return true;
    };
  auto range = [&](const std::string & dotted, double lo, double hi, bool lo_open) {
      YAML::Node n;
      if (!yamlGet(config, dotted, n)) {return;}
      try {
        const double v = n.as<double>();
        if ((lo_open ? v <= lo : v < lo) || v > hi) {
          errors.push_back(dotted + ": " + std::to_string(v) + " out of range " +
            (lo_open ? "(" : "[") + std::to_string(lo) + ", " + std::to_string(hi) + "]");
        }
      } catch (...) {
      }
    };
  auto oneOf = [&](const std::string & dotted, const std::vector<std::string> & allowed,
    bool error) {
      YAML::Node n;
      if (!yamlGet(config, dotted, n) || !n.IsScalar()) {return;}
      const std::string v = n.as<std::string>();
      if (std::find(allowed.begin(), allowed.end(), v) == allowed.end()) {
        std::string list;
        for (const auto & a : allowed) {
          list += (list.empty() ? "" : "|") + a;
        }
        (error ? errors : warnings).push_back(dotted + ": '" + v + "' is not one of " + list);
      }
    };

  if (!config.IsMap()) {
    errors.push_back("top level: expected a mapping with robot/topic/srv/action sections");
  } else {
    check("robot", MAP, true);
    check("robot.arm_name", STR, true);
    check("robot.ee_link_name", STR, true);
    check("robot.robot_desc", STR, true);
    check("robot.planning_frame", STR, true);
    check("robot.move_group_ns", STR_OR_NULL, true);
    check("robot.joint_states", STR, true);
    check("robot.max_vel_scaling_factor", NUM, true);
    check("robot.max_acc_scaling_factor", NUM, true);
    range("robot.max_vel_scaling_factor", 0.0, 1.0, true);
    range("robot.max_acc_scaling_factor", 0.0, 1.0, true);
    check("topic.pub.current_pose.name", STR, true);
    check("topic.pub.current_robot_state.name", STR, true);
    check("topic.sub.joint_states.name", STR, true);
    for (const char * s : {"change_robot_state", "set_vel_acc", "set_planner", "set_eelink",
        "set_planonly"})
    {
      check(std::string("srv.") + s + ".name", STR, true);
    }
    if (enable_actions) {
      for (const char * a : {"move_to_pose", "move_to_joint", "move_to_pose_path",
          "gripper_control"})
      {
        check(std::string("action.") + a + ".name", STR, true);
      }
    }

    // optional, typed
    for (const char * k : {"pose_plan_time_sec", "cartesian_path_min_fraction",
        "max_joint_step_rad", "goal_joint_tolerance_rad", "goal_settle_sec",
        "path_deviation_max_rad"})
    {
      check(std::string("robot.") + k, NUM, false);
    }
    range("robot.cartesian_path_min_fraction", 0.0, 1.0, true);
    for (const char * k : {"goal_joint_tolerance_rad", "goal_settle_sec",
        "path_deviation_max_rad", "max_joint_step_rad"})
    {
      range(std::string("robot.") + k, 0.0, 1e6, false);
    }
    check("robot.async_actions", BOOL, false);  // optional, default true
    check("robot.servo_collision_checking", BOOL, false);
    if (check("robot.on_state_change_while_moving", STR, false)) {
      oneOf("robot.on_state_change_while_moving", {"stop", "reject"}, true);
    }
    if (check("robot.gripper_action_type", STR, false)) {
      oneOf("robot.gripper_action_type",
        {"none", "piper_joint", "gripper_command", "parallel_gripper_command", "parallel",
          "robotiq"}, false);
    }
    if (check("robot.servo_output", STR, false)) {
      oneOf("robot.servo_output", {"trajectory", "forward_position"}, false);
    }

    if (check("planning", MAP, false)) {
      check("planning.eef_step", NUM, false);
      range("planning.eef_step", 0.0, 1.0, true);
      for (const char * list : {"planning.pose_planners", "planning.topic_pose_planners"}) {
        YAML::Node n;
        if (!check(list, SEQ, false) || !yamlGet(config, list, n)) {continue;}
        if (n.size() == 0) {errors.push_back(std::string(list) + ": empty list");}
        for (size_t i = 0; i < n.size(); ++i) {
          const std::string where = std::string(list) + "[" + std::to_string(i) + "]";
          const YAML::Node & e = n[i];
          if (!e.IsMap() || !e["pipeline"] || !e["planner"]) {
            errors.push_back(where + ": needs pipeline and planner");
            continue;
          }
          try {
            if (e["time"] && e["time"].as<double>() < 0.0) {
              errors.push_back(where + ".time: must be >= 0");
            }
            if (e["attempts"] && e["attempts"].as<int>() < 1) {
              errors.push_back(where + ".attempts: must be >= 1");
            }
          } catch (const std::exception &) {
            errors.push_back(where + ": time must be a number, attempts an integer");
          }
        }
      }
    }

    // unknown keys are most likely typos (silently ignored before)
    static const std::vector<std::string> known_top = {
      "robot", "topic", "srv", "action", "tcp_state", "planning", "plans", "named_poses"};
    for (const auto & kv : config) {
      const auto k = kv.first.as<std::string>();
      if (std::find(known_top.begin(), known_top.end(), k) == known_top.end()) {
        warnings.push_back(k + ": unknown top-level section (ignored)");
      }
    }
    static const std::vector<std::string> known_robot = {
      "arm_name", "ee_link_name", "robot_desc", "planning_frame", "move_group_ns",
      "joint_states", "max_vel_scaling_factor", "max_acc_scaling_factor", "max_joint_step_rad",
      "pose_plan_time_sec", "cartesian_path_min_fraction", "async_actions",
      "on_state_change_while_moving", "goal_joint_tolerance_rad", "goal_settle_sec",
      "path_deviation_max_rad", "servo_trajectory_topic", "servo_output",
      "servo_collision_checking", "servo_forward_controller", "servo_default_controller",
      "gripper_action_type", "gripper_backend_action", "gripper_parallel_joint_name",
      "piper_joint_gripper_state_topic", "piper_joint_gripper_cmd_topic", "piper_gripper_open_m",
      "piper_gripper_close_m", "piper_gripper_command_mode", "piper_gripper_trajectory_action",
      "piper_gripper_trajectory_joint_name", "piper_gripper_trajectory_mirror_joint_name",
      "piper_gripper_trajectory_mirror_sign", "piper_gripper_mirror_trajectory_action",
      "piper_gripper_mirror_trajectory_joint_name", "piper_gripper_trajectory_time_sec",
      "piper_gripper_invert_normalized", "attach_touch_links", "named_poses_file"};
    static const std::vector<std::string> removed_robot = {
      "planning_scene", "with_planner", "num_cart_pts"};
    if (config["robot"] && config["robot"].IsMap()) {
      for (const auto & kv : config["robot"]) {
        const auto k = kv.first.as<std::string>();
        if (std::find(removed_robot.begin(), removed_robot.end(), k) != removed_robot.end()) {
          warnings.push_back("robot." + k + ": no longer used, remove it");
        } else if (std::find(known_robot.begin(), known_robot.end(), k) == known_robot.end()) {
          warnings.push_back("robot." + k + ": unknown key (typo?), ignored");
        }
      }
    }
  }

  for (const auto & w : warnings) {
    RCLCPP_WARN_STREAM(this->get_logger(), "config " << config_path << ": " << w);
  }
  if (!errors.empty()) {
    for (const auto & e : errors) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "config " << config_path << ": " << e);
    }
    throw std::runtime_error("invalid arm_api2 config " + config_path + " (" +
                                         std::to_string(errors.size()) +
      " error(s), see the log above)");
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Config " << config_path << " validated (" <<
    warnings.size() << " warning(s))");
}

void m2Iface::loadPlanningConfig()
{
  pose_planners_ = {
    {"pilz_industrial_motion_planner", "LIN", 0.0, 3},
    {"ompl", "EST", 0.0, 3},
    {"ompl", "PRM", 0.0, 3}};
  topic_pose_planners_ = {
    {"pilz_industrial_motion_planner", "LIN", 0.25, 1},
    {"ompl", "EST", 0.75, 1}};
  const YAML::Node p = config["planning"];
  if (p && p["eef_step"]) {eef_step_ = p["eef_step"].as<double>();}
  auto parse = [](const YAML::Node & list, std::vector<PlannerAttempt> & out) {
      if (!list) {return;}
      out.clear();
      for (const auto & e : list) {
        PlannerAttempt a;
        a.pipeline = e["pipeline"].as<std::string>();
        a.planner = e["planner"].as<std::string>();
        if (e["time"]) {a.time = e["time"].as<double>();}
        if (e["attempts"]) {a.attempts = e["attempts"].as<int>();}
        out.push_back(a);
      }
    };
  if (p) {
    parse(p["pose_planners"], pose_planners_);
    parse(p["topic_pose_planners"], topic_pose_planners_);
  }
  auto describe = [](const std::vector<PlannerAttempt> & v) {
      std::string s;
      for (const auto & a : v) {
        s += (s.empty() ? "" : " -> ") + a.pipeline + "/" + a.planner + " x" +
          std::to_string(a.attempts) + (a.time > 0.0 ? " " + fmtRad(a.time) + " s" : "");
      }
      return s;
    };
  RCLCPP_INFO_STREAM(this->get_logger(), "Planning: eef_step " << eef_step_ << ", pose: " <<
                           describe(pose_planners_) << ", topic pose: " <<
    describe(topic_pose_planners_));
}
