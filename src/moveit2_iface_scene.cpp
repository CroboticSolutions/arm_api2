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

/*      Title       : moveit2_iface_scene.cpp
 *      Project     : arm_api2
 *
 *      Description : Planning scene services under arm/ (collision objects, meshes,
 *                    attach/detach), path constraints and named poses.
 */

#include "arm_api2/moveit2_iface.hpp"
#include "moveit2_iface_internal.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <set>

#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

namespace
{
using Trigger = std_srvs::srv::Trigger;
namespace srv = arm_api2_msgs::srv;
}  // namespace

/* ------------------------------------------------------------------ helpers */

bool m2Iface::hasPathConstraints() const
{
  return !path_constraints_.joint_constraints.empty() ||
         !path_constraints_.orientation_constraints.empty() ||
         !path_constraints_.position_constraints.empty() ||
         !path_constraints_.visibility_constraints.empty();
}

std::vector<std::string> m2Iface::touchLinksFor(
  const std::string & link, const std::vector<std::string> & requested) const
{
  std::set<std::string> out;
  out.insert(link);
  if (!requested.empty()) {
    out.insert(requested.begin(), requested.end());
  } else if (!attach_touch_links_cfg_.empty()) {
    out.insert(attach_touch_links_cfg_.begin(), attach_touch_links_cfg_.end());
  } else if (kinematic_model) {
    // Default: the tool around the attach link = its parent link and every link below
    // that parent which is not part of the planning group (fingers, TCP, flange parts).
    const auto * lm = kinematic_model->getLinkModel(link);
    const auto * jmg = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    const auto * parent = lm ? lm->getParentLinkModel() : nullptr;
    if (parent && jmg) {
      out.insert(parent->getName());
      for (const auto * child_joint : parent->getChildJointModels()) {
        for (const auto * l : child_joint->getDescendantLinkModels()) {
          if (!jmg->hasLinkModel(l->getName())) {out.insert(l->getName());}
        }
      }
    }
  }
  return {out.begin(), out.end()};
}

bool m2Iface::applyAttach(
  const std::string & object_id, const std::string & link,
  const std::vector<std::string> & touch_links, std::string & message)
{
  const auto known = m_planningSceneInterface->getKnownObjectNames();
  if (std::find(known.begin(), known.end(), object_id) == known.end()) {
    const auto attached = m_planningSceneInterface->getAttachedObjects({object_id});
    message = attached.count(object_id) ? "'" + object_id + "' is already attached" :
      "'" + object_id + "' is not a collision object in the scene";
    return false;
  }
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = link;
  aco.object.id = object_id;
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;   // takes the world object
  aco.touch_links = touch_links;
  if (!m_planningSceneInterface->applyAttachedCollisionObject(aco)) {
    message = "move_group did not apply the attach of '" + object_id + "'";
    return false;
  }
  std::string tl;
  for (const auto & t : touch_links) {
    tl += (tl.empty() ? "" : ",") + t;
  }
  message = "attached '" + object_id + "' to " + link + " (touch links: " + tl + ")";
  RCLCPP_INFO_STREAM(this->get_logger(), message);
  return true;
}

bool m2Iface::applyDetach(const std::string & object_id, bool remove, std::string & message)
{
  const auto attached = object_id.empty() ? m_planningSceneInterface->getAttachedObjects() :
    m_planningSceneInterface->getAttachedObjects({object_id});
  if (attached.empty()) {
    message = object_id.empty() ? "nothing attached" : "'" + object_id + "' is not attached";
    return object_id.empty();
  }
  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.robot_state.is_diff = true;
  for (const auto & kv : attached) {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = kv.second.link_name;
    aco.object.id = kv.first;
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;   // back into the world
    ps.robot_state.attached_collision_objects.push_back(aco);
  }
  if (!m_planningSceneInterface->applyPlanningScene(ps)) {
    message = "move_group did not apply the detach";
    return false;
  }
  if (remove) {
    moveit_msgs::msg::PlanningScene rm;
    rm.is_diff = true;
    rm.robot_state.is_diff = true;
    for (const auto & kv : attached) {
      moveit_msgs::msg::CollisionObject co;
      co.id = kv.first;
      co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      rm.world.collision_objects.push_back(co);
    }
    if (!m_planningSceneInterface->applyPlanningScene(rm)) {
      message = "detached, but removing from the scene failed";
      return false;
    }
  }
  message = "detached " + std::to_string(attached.size()) + " object(s)" +
    (remove ? " and removed them from the scene" : "");
  RCLCPP_INFO_STREAM(this->get_logger(), message);
  return true;
}

/* ------------------------------------------------------------------ named poses */

void m2Iface::loadNamedPoses()
{
  std::vector<std::string> group_vars;
  if (kinematic_model) {
    if (const auto * jmg = kinematic_model->getJointModelGroup(PLANNING_GROUP)) {
      group_vars = jmg->getVariableNames();
    }
  }
  if (config["robot"] && config["robot"]["named_poses_file"]) {
    named_poses_file_ = config["robot"]["named_poses_file"].as<std::string>();
  }
  if (named_poses_file_.empty()) {
    const char * dir = std::getenv("ARM_API2_NAMED_POSES_DIR");
    const char * home = std::getenv("HOME");
    const std::string base = dir ? std::string(dir) :
      std::string(home ? home : "/tmp") + "/.ros/arm_api2/named_poses";
    const std::string model = kinematic_model ? kinematic_model->getName() : "robot";
    named_poses_file_ = base + "/" + model + "_" + PLANNING_GROUP + ".yaml";
  }
  if (!named_poses_file_.empty() && named_poses_file_[0] == '~') {
    const char * home = std::getenv("HOME");
    named_poses_file_ = std::string(home ? home : "") + named_poses_file_.substr(1);
  }
  std::vector<std::string> errors;
  auto cfg = arm_api2::logic::parseNamedPoses(config["named_poses"], group_vars, "config",
      errors);
  arm_api2::logic::NamedPoseMap saved;
  if (std::filesystem::exists(named_poses_file_)) {
    try {
      saved = arm_api2::logic::parseNamedPoses(YAML::LoadFile(named_poses_file_), group_vars,
          "saved", errors);
    } catch (const YAML::Exception & e) {
      errors.push_back(named_poses_file_ + ": " + e.what());
    }
  }
  for (const auto & e : errors) {
    RCLCPP_WARN_STREAM(this->get_logger(), "named poses: " << e);
  }
  std::lock_guard<std::mutex> lock(named_poses_mutex_);
  config_named_poses_ = std::move(cfg);
  saved_named_poses_ = std::move(saved);
  RCLCPP_INFO(this->get_logger(), "Named poses: %zu from config, %zu saved (%s)",
    config_named_poses_.size(), saved_named_poses_.size(), named_poses_file_.c_str());
}

bool m2Iface::lookupNamedPose(
  const std::string & name, arm_api2::logic::NamedPose & out, std::string & error)
{
  {
    std::lock_guard<std::mutex> lock(named_poses_mutex_);
    for (const auto * m : {&saved_named_poses_, &config_named_poses_}) {
      const auto it = m->find(name);
      if (it != m->end()) {out = it->second; return true;}
    }
  }
  if (m_moveGroupPtr) {
    const auto targets = m_moveGroupPtr->getNamedTargets();
    if (std::find(targets.begin(), targets.end(), name) != targets.end()) {
      out = arm_api2::logic::NamedPose();
      out.source = "srdf";
      for (const auto & kv : m_moveGroupPtr->getNamedTargetValues(name)) {
        out.joint_names.push_back(kv.first);
        out.positions.push_back(kv.second);
      }
      return true;
    }
  }
  error = "unknown named pose '" + name + "' (arm/list_named_poses)";
  return false;
}

/* ------------------------------------------------------------------ services */

void m2Iface::init_scene_interfaces()
{
  if (config["robot"] && config["robot"]["attach_touch_links"]) {
    attach_touch_links_cfg_ = config["robot"]["attach_touch_links"].as<std::vector<std::string>>();
  }
  auto name = [this](const std::string & key, const std::string & def) {
      return interfaceName("srv", key, def);
    };

  /* ---- named poses ---- */
  save_named_pose_srv_ = this->create_service<srv::SaveNamedPose>(
    name("save_named_pose", "arm/save_named_pose"),
    [this](const std::shared_ptr<srv::SaveNamedPose::Request> req,
    std::shared_ptr<srv::SaveNamedPose::Response> res) {
      res->success = false;
      if (!arm_api2::logic::validPoseName(req->name)) {
        res->message = "invalid name (1-64 chars of [A-Za-z0-9_-])";
        return;
      }
      const auto * jmg = kinematic_model ?
      kinematic_model->getJointModelGroup(PLANNING_GROUP) : nullptr;
      if (!jmg) {res->message = "robot model not ready"; return;}
      const auto vars = jmg->getVariableNames();
      arm_api2::logic::NamedPose pose;
      pose.source = "saved";
      pose.joint_names = vars;
      if (req->joint_state.position.empty()) {
        std::unordered_map<std::string, double> js;
        if (!latestJoints(js, 1.0)) {res->message = "no recent joint_states"; return;}
        for (const auto & v : vars) {
          const auto it = js.find(v);
          if (it == js.end()) {res->message = "joint_states miss " + v; return;}
          pose.positions.push_back(it->second);
        }
      } else {
        arm_api2::logic::NamedPose given;
        given.joint_names = req->joint_state.name.empty() ? vars : req->joint_state.name;
        given.positions = req->joint_state.position;
        if (given.joint_names.size() != given.positions.size()) {
          res->message = "joint_state names/positions do not match (empty names = group order)";
          return;
        }
        std::vector<double> full;
        std::string err;
        std::vector<double> zeros(vars.size(), 0.0);
        if (!arm_api2::logic::namedPoseToGroup(given, vars, zeros, full, err)) {
          res->message = err;
          return;
        }
        pose.joint_names = given.joint_names;
        pose.positions = given.positions;
      }
      arm_api2::logic::NamedPose existing;
      std::string err;
      if (!req->overwrite && lookupNamedPose(req->name, existing, err)) {
        res->message = "'" + req->name + "' exists (" + existing.source + "); set overwrite";
        return;
      }
      std::lock_guard<std::mutex> lock(named_poses_mutex_);
      auto updated = saved_named_poses_;
      updated[req->name] = pose;
      if (!arm_api2::logic::writeFileAtomic(named_poses_file_,
      arm_api2::logic::dumpNamedPoses(updated), err))
      {
        res->message = "cannot save " + named_poses_file_ + ": " + err;
        return;
      }
      saved_named_poses_ = std::move(updated);
      res->success = true;
      res->message = "saved '" + req->name + "' to " + named_poses_file_;
      RCLCPP_INFO_STREAM(this->get_logger(), res->message);
    });

  list_named_poses_srv_ = this->create_service<srv::ListNamedPoses>(
    name("list_named_poses", "arm/list_named_poses"),
    [this](const std::shared_ptr<srv::ListNamedPoses::Request>,
    std::shared_ptr<srv::ListNamedPoses::Response> res) {
      std::set<std::string> seen;
      auto add = [&](const std::string & n, const arm_api2::logic::NamedPose & p) {
        if (!seen.insert(n).second) {return;}
        res->names.push_back(n);
        res->sources.push_back(p.source);
        sensor_msgs::msg::JointState js;
        js.name = p.joint_names;
        js.position = p.positions;
        res->joint_states.push_back(js);
      };
      {
        std::lock_guard<std::mutex> lock(named_poses_mutex_);
        for (const auto & kv : saved_named_poses_) {
          add(kv.first, kv.second);
        }
        for (const auto & kv : config_named_poses_) {
          add(kv.first, kv.second);
        }
      }
      if (m_moveGroupPtr) {
        for (const auto & n : m_moveGroupPtr->getNamedTargets()) {
          arm_api2::logic::NamedPose p;
          std::string err;
          if (seen.count(n) == 0 && lookupNamedPose(n, p, err)) {add(n, p);}
        }
      }
    });

  delete_named_pose_srv_ = this->create_service<srv::NameRequest>(
    name("delete_named_pose", "arm/delete_named_pose"),
    [this](const std::shared_ptr<srv::NameRequest::Request> req,
    std::shared_ptr<srv::NameRequest::Response> res) {
      std::lock_guard<std::mutex> lock(named_poses_mutex_);
      res->success = false;
      if (saved_named_poses_.count(req->name) == 0) {
        res->message = config_named_poses_.count(req->name) ?
        "'" + req->name + "' comes from the robot YAML (named_poses); edit it there" :
        "'" + req->name + "' is not a saved pose";
        return;
      }
      auto updated = saved_named_poses_;
      updated.erase(req->name);
      std::string err;
      if (!arm_api2::logic::writeFileAtomic(named_poses_file_,
      arm_api2::logic::dumpNamedPoses(updated), err))
      {
        res->message = "cannot save " + named_poses_file_ + ": " + err;
        return;
      }
      saved_named_poses_ = std::move(updated);
      res->success = true;
      res->message = "deleted '" + req->name + "'";
    });

  /* ---- attach / detach ---- */
  attach_object_srv_ = this->create_service<srv::AttachObject>(
    name("attach_object", "arm/attach_object"),
    [this](const std::shared_ptr<srv::AttachObject::Request> req,
    std::shared_ptr<srv::AttachObject::Response> res) {
      res->success = false;
      std::string id = req->object_id;
      if (!req->object.id.empty()) {
        auto co = req->object;
        if (co.header.frame_id.empty()) {co.header.frame_id = PLANNING_FRAME;}
        co.operation = moveit_msgs::msg::CollisionObject::ADD;
        if (!m_planningSceneInterface->applyCollisionObject(co)) {
          res->message = "adding '" + co.id + "' to the scene failed";
          return;
        }
        if (id.empty()) {id = co.id;}
      }
      if (id.empty()) {res->message = "object_id (or object.id) is empty"; return;}
      const std::string link = req->link.empty() ? EE_LINK_NAME : req->link;
      if (!kinematic_model || !kinematic_model->hasLinkModel(link)) {
        res->message = "link '" + link + "' not in the robot model";
        return;
      }
      res->success = applyAttach(id, link, touchLinksFor(link, req->touch_links), res->message);
    });

  detach_object_srv_ = this->create_service<srv::DetachObject>(
    name("detach_object", "arm/detach_object"),
    [this](const std::shared_ptr<srv::DetachObject::Request> req,
    std::shared_ptr<srv::DetachObject::Response> res) {
      res->success = applyDetach(req->object_id, req->remove_from_scene, res->message);
    });

  /* ---- path constraints ---- */
  set_path_constraints_srv_ = this->create_service<srv::SetPathConstraints>(
    name("set_path_constraints", "arm/set_path_constraints"),
    [this](const std::shared_ptr<srv::SetPathConstraints::Request> req,
    std::shared_ptr<srv::SetPathConstraints::Response> res) {
      moveit_msgs::msg::Constraints c;
      c.name = "arm_api2_path_constraints";
      c.joint_constraints = req->joint_constraints;
      for (auto o : req->orientation_constraints) {
        if (o.link_name.empty()) {o.link_name = EE_LINK_NAME;}
        if (o.header.frame_id.empty()) {o.header.frame_id = PLANNING_FRAME;}
        if (o.weight <= 0.0) {o.weight = 1.0;}
        c.orientation_constraints.push_back(o);
      }
      for (auto p : req->position_constraints) {
        if (p.link_name.empty()) {p.link_name = EE_LINK_NAME;}
        if (p.header.frame_id.empty()) {p.header.frame_id = PLANNING_FRAME;}
        if (p.weight <= 0.0) {p.weight = 1.0;}
        if (p.constraint_region.primitives.empty() && p.constraint_region.meshes.empty()) {
          res->success = false;
          res->message = "position constraint on " + p.link_name + " has no constraint_region";
          return;
        }
        c.position_constraints.push_back(p);
      }
      for (auto & j : c.joint_constraints) {
        if (j.weight <= 0.0) {j.weight = 1.0;}
      }
      path_constraints_ = c;
      if (m_moveGroupPtr) {m_moveGroupPtr->setPathConstraints(c);}
      res->success = true;
      res->message = "path constraints set: " + std::to_string(c.joint_constraints.size()) +
      " joint, " + std::to_string(c.orientation_constraints.size()) + " orientation, " +
      std::to_string(c.position_constraints.size()) +
      " position (Pilz is skipped while they are set)";
      RCLCPP_INFO_STREAM(this->get_logger(), res->message);
    });

  clear_path_constraints_srv_ = this->create_service<srv::ClearPathConstraints>(
    name("clear_path_constraints", "arm/clear_path_constraints"),
    [this](const std::shared_ptr<srv::ClearPathConstraints::Request>,
    std::shared_ptr<srv::ClearPathConstraints::Response> res) {
      path_constraints_ = moveit_msgs::msg::Constraints();
      if (m_moveGroupPtr) {m_moveGroupPtr->clearPathConstraints();}
      res->success = true;
      res->message = "path constraints cleared";
    });

  /* ---- collision objects under arm/ ---- */
  arm_add_collision_object_srv_ = this->create_service<srv::AddCollisionObject>(
    name("arm_add_collision_object", "arm/add_collision_object"),
    std::bind(&m2Iface::add_collision_object_cb, this, _1, _2));

  add_collision_mesh_srv_ = this->create_service<srv::AddCollisionMesh>(
    name("add_collision_mesh", "arm/add_collision_mesh"),
    [this](const std::shared_ptr<srv::AddCollisionMesh::Request> req,
    std::shared_ptr<srv::AddCollisionMesh::Response> res) {
      res->success = false;
      if (req->id.empty()) {res->message = "id is empty"; return;}
      std::string uri = req->mesh_path;
      if (!uri.empty() && uri[0] == '/') {uri = "file://" + uri;}
      const double s = req->scale > 0.0 ? req->scale : 1.0;
      std::unique_ptr<shapes::Mesh> mesh(
        shapes::createMeshFromResource(uri, Eigen::Vector3d(s, s, s)));
      if (!mesh) {res->message = "cannot load mesh '" + req->mesh_path + "'"; return;}
      shapes::ShapeMsg shape_msg;
      if (!shapes::constructMsgFromShape(mesh.get(), shape_msg)) {
        res->message = "mesh conversion failed";
        return;
      }
      moveit_msgs::msg::CollisionObject co;
      co.id = req->id;
      co.header.frame_id = req->pose.header.frame_id.empty() ? PLANNING_FRAME :
      req->pose.header.frame_id;
      co.pose.orientation.w = 1.0;
      co.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
      co.mesh_poses.push_back(req->pose.pose);
      if (co.mesh_poses.back().orientation.x == 0.0 && co.mesh_poses.back().orientation.y == 0.0 &&
      co.mesh_poses.back().orientation.z == 0.0 && co.mesh_poses.back().orientation.w == 0.0)
      {
        co.mesh_poses.back().orientation.w = 1.0;
      }
      co.operation = moveit_msgs::msg::CollisionObject::ADD;
      if (!m_planningSceneInterface->applyCollisionObject(co)) {
        res->message = "move_group did not apply the mesh";
        return;
      }
      res->success = true;
      res->message = "mesh '" + req->id + "' added (" + std::to_string(mesh->vertex_count) +
      " vertices, scale " + std::to_string(s) + ")";
      RCLCPP_INFO_STREAM(this->get_logger(), res->message);
    });

  remove_collision_object_srv_ = this->create_service<srv::NameRequest>(
    name("remove_collision_object", "arm/remove_collision_object"),
    [this](const std::shared_ptr<srv::NameRequest::Request> req,
    std::shared_ptr<srv::NameRequest::Response> res) {
      res->success = false;
      if (!m_planningSceneInterface->getAttachedObjects({req->name}).empty()) {
        res->message = "'" + req->name + "' is attached; detach it first (arm/detach_object)";
        return;
      }
      const auto known = m_planningSceneInterface->getKnownObjectNames();
      if (std::find(known.begin(), known.end(), req->name) == known.end()) {
        res->message = "'" + req->name + "' is not in the scene";
        return;
      }
      moveit_msgs::msg::CollisionObject co;
      co.id = req->name;
      co.header.frame_id = PLANNING_FRAME;
      co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      res->success = m_planningSceneInterface->applyCollisionObject(co);
      res->message = res->success ? "removed '" + req->name + "'" : "remove failed";
    });

  list_collision_objects_srv_ = this->create_service<srv::ListCollisionObjects>(
    name("list_collision_objects", "arm/list_collision_objects"),
    [this](const std::shared_ptr<srv::ListCollisionObjects::Request>,
    std::shared_ptr<srv::ListCollisionObjects::Response> res) {
      res->world_ids = m_planningSceneInterface->getKnownObjectNames();
      for (const auto & kv : m_planningSceneInterface->getAttachedObjects()) {
        res->attached_ids.push_back(kv.first);
        res->attached_links.push_back(kv.second.link_name);
      }
    });

  clear_collision_objects_srv_ = this->create_service<Trigger>(
    name("clear_collision_objects", "arm/clear_collision_objects"),
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
      const auto ids = m_planningSceneInterface->getKnownObjectNames();
      moveit_msgs::msg::PlanningScene ps;
      ps.is_diff = true;
      ps.robot_state.is_diff = true;
      for (const auto & id : ids) {
        moveit_msgs::msg::CollisionObject co;
        co.id = id;
        co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        ps.world.collision_objects.push_back(co);
      }
      res->success = ids.empty() || m_planningSceneInterface->applyPlanningScene(ps);
      const auto attached = m_planningSceneInterface->getAttachedObjects();
      res->message = "removed " + std::to_string(ids.size()) + " world object(s)" +
      (attached.empty() ? "" : "; " + std::to_string(attached.size()) +
      " attached object(s) kept (arm/detach_object)");
    });

  RCLCPP_INFO(this->get_logger(),
    "Scene services: arm/{attach,detach}_object, arm/{set,clear}_path_constraints, "
    "arm/{add,remove,list,clear}_collision_object(s), arm/add_collision_mesh, "
    "arm/{save,list,delete}_named_pose(s)");
}
