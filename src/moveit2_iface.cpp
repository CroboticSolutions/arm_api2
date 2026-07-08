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

/*      Title       : moveit2_iface.cpp
 *      Project     : arm_api2
 *      Created     : 06/08/2025
 *      Author      : Filip Zoric
 *      Contributors: Edgar Welte
 *      Description : The core robot manipulator and MoveIt2! ROS 2 interfacing header class.
 */

#include "arm_api2/moveit2_iface.hpp"
#include <algorithm>
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

m2Iface::m2Iface(const rclcpp::NodeOptions & options)
: Node("moveit2_iface", options),
  node_(std::make_shared<rclcpp::Node>("moveit2_iface_node", options)),
  executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()), gripper_(node_)
{
  this->get_parameter("config_path", config_path);
  this->get_parameter("enable_servo", enable_servo);
    // mode sets the interface profile; explicit enable_* params still override it
  this->get_parameter("mode", mode_);
  if (mode_ == "simple") {
    enable_actions = false;
  } else if (mode_ != "advanced") {
    RCLCPP_WARN_STREAM(this->get_logger(),
      "Unknown mode '" << mode_ << "', expected 'simple' or 'advanced'. Using 'advanced'.");
    mode_ = "advanced";
  }
  this->get_parameter("enable_topics", enable_topics);
  this->get_parameter("enable_actions", enable_actions);
  this->get_parameter("dt", dt);

  RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config!");

  std::chrono::duration<double> SYSTEM_DT(dt);
  timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&m2Iface::run, this));

    // Load arm basically --> two important params
    // Manual param specification --> https://github.com/moveit/moveit2_tutorials/blob/8eaef05bfbabde3f35910ad054a819d79e70d3fc/doc/tutorials/quickstart_in_rviz/launch/demo.launch.py#L105
  config = init_config(config_path);
  PLANNING_GROUP = config["robot"]["arm_name"].as<std::string>();
  EE_LINK_NAME = config["robot"]["ee_link_name"].as<std::string>();
  ROBOT_DESC = config["robot"]["robot_desc"].as<std::string>();
  PLANNING_FRAME = config["robot"]["planning_frame"].as<std::string>();
  PLANNING_SCENE = config["robot"]["planning_scene"].as<std::string>();
  MOVE_GROUP_NS = config["robot"]["move_group_ns"].as<std::string>();
  JOINT_STATES = config["robot"]["joint_states"].as<std::string>();
  WITH_PLANNER = config["robot"]["with_planner"].as<bool>();
  INIT_VEL_SCALING = 0.05;
  INIT_ACC_SCALING = 0.05;
  eager_execution = true;
  max_vel_scaling_factor = config["robot"]["max_vel_scaling_factor"].as<float>();
  max_acc_scaling_factor = config["robot"]["max_acc_scaling_factor"].as<float>();
  if (config["robot"]["num_cart_pts"]) {
    num_cart_pts_ = config["robot"]["num_cart_pts"].as<int>();
  }

    /* Optional override for OMPL planning time. Default 1.5 s — see hpp comment. */
  if (config["robot"]["pose_plan_time_sec"]) {
    const double v = config["robot"]["pose_plan_time_sec"].as<double>();
    if (v > 0.0) {pose_plan_time_sec_ = v;}
  }

  SERVO_TRAJECTORY_TOPIC = "/joint_trajectory_controller/joint_trajectory";
  if (config["robot"]["servo_trajectory_topic"]) {
    SERVO_TRAJECTORY_TOPIC = config["robot"]["servo_trajectory_topic"].as<std::string>();
  }
  if (config["robot"]["servo_output"]) {
    servo_use_forward_position_ =
      config["robot"]["servo_output"].as<std::string>() == std::string("forward_position");
  }
  if (config["robot"]["servo_collision_checking"]) {
    servo_collision_checking_ = config["robot"]["servo_collision_checking"].as<bool>();
  }
  if (config["robot"]["servo_forward_controller"]) {
    servo_forward_controller_ = config["robot"]["servo_forward_controller"].as<std::string>();
  }
  if (config["robot"]["servo_default_controller"]) {
    servo_default_controller_ = config["robot"]["servo_default_controller"].as<std::string>();
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  {
    const YAML::Node & r = config["robot"];
    std::optional<std::string> action_type_val;
    if (r["gripper_action_type"]) {
      action_type_val = r["gripper_action_type"].as<std::string>();
    }

    if (action_type_val && *action_type_val == std::string("piper_joint")) {
      PiperJointGripperConfig pcfg;
      if (r["piper_joint_gripper_state_topic"]) {
        pcfg.state_topic = r["piper_joint_gripper_state_topic"].as<std::string>();
      }
      if (r["piper_joint_gripper_cmd_topic"]) {
        pcfg.cmd_topic = r["piper_joint_gripper_cmd_topic"].as<std::string>();
      }
      if (r["piper_gripper_open_m"]) {
        pcfg.open_stroke_m = r["piper_gripper_open_m"].as<double>();
      }
      if (r["piper_gripper_close_m"]) {
        pcfg.close_stroke_m = r["piper_gripper_close_m"].as<double>();
      }
      if (r["piper_gripper_command_mode"]) {
        std::string mode = r["piper_gripper_command_mode"].as<std::string>();
        std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
          });
        if (mode == std::string("joint_state")) {
          pcfg.command_mode = PiperGripperCommandMode::JointState;
        } else {
          pcfg.command_mode = PiperGripperCommandMode::FollowJointTrajectory;
        }
      }
      if (r["piper_gripper_trajectory_action"]) {
        pcfg.trajectory_action = r["piper_gripper_trajectory_action"].as<std::string>();
      }
      if (r["piper_gripper_trajectory_joint_name"]) {
        pcfg.trajectory_joint_name = r["piper_gripper_trajectory_joint_name"].as<std::string>();
      }
      if (r["piper_gripper_trajectory_mirror_joint_name"]) {
        pcfg.trajectory_mirror_joint_name =
          r["piper_gripper_trajectory_mirror_joint_name"].as<std::string>();
      }
      if (r["piper_gripper_trajectory_mirror_sign"]) {
        pcfg.trajectory_mirror_sign = r["piper_gripper_trajectory_mirror_sign"].as<double>();
      }
      if (r["piper_gripper_mirror_trajectory_action"]) {
        pcfg.mirror_trajectory_action =
          r["piper_gripper_mirror_trajectory_action"].as<std::string>();
      }
      if (r["piper_gripper_mirror_trajectory_joint_name"]) {
        pcfg.mirror_trajectory_joint_name =
          r["piper_gripper_mirror_trajectory_joint_name"].as<std::string>();
      }
      if (r["piper_gripper_trajectory_time_sec"]) {
        pcfg.trajectory_time_from_start_sec = r["piper_gripper_trajectory_time_sec"].as<double>();
      }
      if (r["piper_gripper_invert_normalized"]) {
        pcfg.invert_robotiq_normalized = r["piper_gripper_invert_normalized"].as<bool>();
      }
      piper_joint_gripper_ = std::make_unique<PiperJointGripper>(node_);
      piper_joint_gripper_->configure(pcfg);
    } else {
      RobotiqGripperConfig gcfg;
      if (r["gripper_backend_action"]) {
        gcfg.backend_action = r["gripper_backend_action"].as<std::string>();
      }
      if (action_type_val) {
        const std::string & t = *action_type_val;
        if (t == "parallel_gripper_command" || t == "parallel") {
          gcfg.backend = RobotiqGripperConfig::BackendKind::ParallelGripperCommand;
        }
      }
      if (r["gripper_parallel_joint_name"]) {
        gcfg.parallel_joint_name = r["gripper_parallel_joint_name"].as<std::string>();
      }
      gripper_.configure(gcfg);
    }
  }

    // Align MoveIt namespace defaults with node namespace when config is generic.
  ns_ = this->get_namespace();
  if (MOVE_GROUP_NS.empty() || MOVE_GROUP_NS == "null") {
    MOVE_GROUP_NS = (ns_ == "/") ? "" : ns_;
  }
  init_publishers();
  init_subscribers();
  init_services();
  init_moveit();
  if (enable_actions) {
    init_actionservers();
  }
  if (enable_servo) {servoPtr = init_servo();}

  RCLCPP_INFO_STREAM(this->get_logger(),
    "Initialized node! mode: " << mode_ << ", topics: " << enable_topics <<
    ", actions: " << enable_actions);

    // Seed the previous pose with an impossible value so the first command is always accepted.
  m_oldTopicPoseCmd.pose.position.x = 5.0;
  nodeInit = true;
}

m2Iface::~m2Iface()
{
  if (m_moveGroupPtr) {
    try {
      m_moveGroupPtr->stop();
    } catch (const std::exception & ex) {
      RCLCPP_WARN(this->get_logger(), "Failed to stop MoveGroup during shutdown: %s", ex.what());
    }
  }

  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  std::lock_guard<std::mutex> lock(execution_thread_mutex_);
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
}

YAML::Node m2Iface::init_config(std::string yaml_path)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Config yaml path is: " << yaml_path);
  return YAML::LoadFile(yaml_path);
}

void m2Iface::init_publishers()
{
  auto pose_state_name = config["topic"]["pub"]["current_pose"]["name"].as<std::string>();
  auto robot_state_name = config["topic"]["pub"]["current_robot_state"]["name"].as<std::string>();
  pose_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_state_name, 1);
  robot_state_pub_ = this->create_publisher<std_msgs::msg::String>(robot_state_name, 1);
  if (config["topic"] && config["topic"]["pub"] && config["topic"]["pub"]["gripper_state"] &&
    config["topic"]["pub"]["gripper_state"]["name"])
  {
    const auto gripper_state_name =
      config["topic"]["pub"]["gripper_state"]["name"].as<std::string>();
    gripper_state_pub_ = this->create_publisher<std_msgs::msg::String>(gripper_state_name, 1);
  }

    /* Plan-status: latched so a late-joining GUI client sees the most recent
     * attempt without waiting for the next click. */
  std::string plan_status_name = "arm/state/plan_status";
  if (config["topic"]["pub"]["plan_status"] && config["topic"]["pub"]["plan_status"]["name"]) {
    plan_status_name = config["topic"]["pub"]["plan_status"]["name"].as<std::string>();
  }
  rclcpp::QoS plan_status_qos(rclcpp::KeepLast(1));
  plan_status_qos.transient_local();
  plan_status_pub_ = this->create_publisher<arm_api2_msgs::msg::PlanStatus>(plan_status_name,
    plan_status_qos);

  if (servo_use_forward_position_) {
    servo_forward_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "forward_position_controller/commands", 10);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialized publishers!");
}

void m2Iface::init_subscribers()
{
  auto joint_states_name = config["topic"]["sub"]["joint_states"]["name"].as<std::string>();
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_states_name, 1,
    std::bind(&m2Iface::joint_state_cb, this, _1));
    // Servo twist subscriber
  servo_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "~/servo_twist_cmd", 10, std::bind(&m2Iface::servo_twist_cb, this, _1));
    // Servo trajectory output publisher
  servo_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        SERVO_TRAJECTORY_TOPIC, 10);
    // Servo status publisher
  servo_status_pub_ = this->create_publisher<moveit_msgs::msg::ServoStatus>(
        "~/status", 10);
  if (enable_topics && config["topic"] && config["topic"]["sub"] &&
    config["topic"]["sub"]["cmd_pose"] && config["topic"]["sub"]["cmd_pose"]["name"])
  {
    auto pose_cmd_name = config["topic"]["sub"]["cmd_pose"]["name"].as<std::string>();
    pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_cmd_name, 1, std::bind(&m2Iface::pose_cmd_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed to pose cmd topic: " << pose_cmd_name);
  }
  if (enable_topics && config["topic"] && config["topic"]["sub"] &&
    config["topic"]["sub"]["cmd_traj"] && config["topic"]["sub"]["cmd_traj"]["name"])
  {
    auto cart_traj_cmd_name = config["topic"]["sub"]["cmd_traj"]["name"].as<std::string>();
    ctraj_cmd_sub_ = this->create_subscription<arm_api2_msgs::msg::CartesianWaypoints>(
            cart_traj_cmd_name, 1, std::bind(&m2Iface::cart_poses_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(),
      "Subscribed to Cartesian waypoint topic: " << cart_traj_cmd_name);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!");
}

void m2Iface::init_services()
{
  auto change_state_name = config["srv"]["change_robot_state"]["name"].as<std::string>();
  auto set_vel_acc_name = config["srv"]["set_vel_acc"]["name"].as<std::string>();
  auto set_planner_name = config["srv"]["set_planner"]["name"].as<std::string>();
  auto set_eelink_name = config["srv"]["set_eelink"]["name"].as<std::string>();
  auto set_plan_only_name = config["srv"]["set_planonly"]["name"].as<std::string>();
  std::string open_gripper_name = "arm/open_gripper";
  std::string close_gripper_name = "arm/close_gripper";
  if (config["srv"]["open_gripper"] && config["srv"]["open_gripper"]["name"]) {
    open_gripper_name = config["srv"]["open_gripper"]["name"].as<std::string>();
  }
  if (config["srv"]["close_gripper"] && config["srv"]["close_gripper"]["name"]) {
    close_gripper_name = config["srv"]["close_gripper"]["name"].as<std::string>();
  }
  change_state_srv_ = this->create_service<arm_api2_msgs::srv::ChangeState>(change_state_name,
    std::bind(&m2Iface::change_state_cb, this, _1, _2));
  set_vel_acc_srv_ = this->create_service<arm_api2_msgs::srv::SetVelAcc>(set_vel_acc_name,
    std::bind(&m2Iface::set_vel_acc_cb, this, _1, _2));
  set_planner_srv_ = this->create_service<arm_api2_msgs::srv::SetStringParam>(set_planner_name,
    std::bind(&m2Iface::set_planner_cb, this, _1, _2));
  set_eelink_srv_ = this->create_service<arm_api2_msgs::srv::SetStringParam>(set_eelink_name,
    std::bind(&m2Iface::set_eelink_cb, this, _1, _2));
  set_plan_only_srv_ = this->create_service<std_srvs::srv::SetBool>(set_plan_only_name,
    std::bind(&m2Iface::set_plan_only_cb, this, _1, _2));
  add_collision_object_srv_ =
    this->create_service<arm_api2_msgs::srv::AddCollisionObject>("add_collision_object",
    std::bind(&m2Iface::add_collision_object_cb, this, _1, _2));
  open_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(
        open_gripper_name, std::bind(&m2Iface::open_gripper_cb, this, _1, _2));
  close_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(
        close_gripper_name, std::bind(&m2Iface::close_gripper_cb, this, _1, _2));

  std::string check_reachability_name = "arm/check_reachability";
  if (config["srv"]["check_reachability"] && config["srv"]["check_reachability"]["name"]) {
    check_reachability_name = config["srv"]["check_reachability"]["name"].as<std::string>();
  }
  check_reachability_srv_ = this->create_service<arm_api2_msgs::srv::CheckReachability>(
        check_reachability_name,
        std::bind(&m2Iface::check_reachability_cb, this, _1, _2));

  std::string check_cartesian_path_name = "arm/check_cartesian_path";
  if (config["srv"]["check_cartesian_path"] && config["srv"]["check_cartesian_path"]["name"]) {
    check_cartesian_path_name = config["srv"]["check_cartesian_path"]["name"].as<std::string>();
  }
  check_cartesian_path_srv_ = this->create_service<arm_api2_msgs::srv::CheckCartesianPath>(
        check_cartesian_path_name,
        std::bind(&m2Iface::check_cartesian_path_cb, this, _1, _2));

  if (servo_use_forward_position_) {
    configure_controller_client_ =
      node_->create_client<controller_manager_msgs::srv::ConfigureController>(
            "controller_manager/configure_controller");
    load_controller_client_ =
      node_->create_client<controller_manager_msgs::srv::LoadController>(
            "controller_manager/load_controller");
    switch_controller_client_ =
      node_->create_client<controller_manager_msgs::srv::SwitchController>(
            "controller_manager/switch_controller");
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialized services!");
}

void m2Iface::init_actionservers()
{
  auto move_to_pose_name = config["action"]["move_to_pose"]["name"].as<std::string>();
  auto move_to_joint_name = config["action"]["move_to_joint"]["name"].as<std::string>();
  auto move_to_pose_path_name = config["action"]["move_to_pose_path"]["name"].as<std::string>();
  auto gripper_control_name = config["action"]["gripper_control"]["name"].as<std::string>();
  move_to_pose_as_ = rclcpp_action::create_server<arm_api2_msgs::action::MoveCartesian>(this,
                                                                                        move_to_pose_name,
                                                                                        std::bind(
    &m2Iface::move_to_pose_goal_cb, this, _1, _2),
                                                                                        std::bind(
    &m2Iface::move_to_pose_cancel_cb, this, _1),
                                                                                        std::bind(
    &m2Iface::move_to_pose_accepted_cb, this, _1));
  move_to_joint_as_ = rclcpp_action::create_server<arm_api2_msgs::action::MoveJoint>(this,
                                                                                        move_to_joint_name,
                                                                                        std::bind(
    &m2Iface::move_to_joint_goal_cb, this, _1, _2),
                                                                                        std::bind(
    &m2Iface::move_to_joint_cancel_cb, this, _1),
                                                                                        std::bind(
    &m2Iface::move_to_joint_accepted_cb, this, _1));
  move_to_pose_path_as_ =
    rclcpp_action::create_server<arm_api2_msgs::action::MoveCartesianPath>(this,
                                                                                        move_to_pose_path_name,
                                                                                        std::bind(
    &m2Iface::move_to_pose_path_goal_cb, this, _1, _2),
                                                                                        std::bind(
    &m2Iface::move_to_pose_path_cancel_cb, this, _1),
                                                                                        std::bind(
    &m2Iface::move_to_pose_path_accepted_cb, this, _1));
  gripper_control_as_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(this,
                                                                                        gripper_control_name,
                                                                                        std::bind(
    &m2Iface::gripper_control_goal_cb, this, _1, _2),
                                                                                        std::bind(
    &m2Iface::gripper_control_cancel_cb, this, _1),
                                                                                        std::bind(
    &m2Iface::gripper_control_accepted_cb, this, _1));

  RCLCPP_INFO_STREAM(this->get_logger(), "Initialized action servers!");
}
void m2Iface::init_moveit()
{

  RCLCPP_INFO_STREAM(this->get_logger(), "robot_description: " << ROBOT_DESC);
  RCLCPP_INFO_STREAM(this->get_logger(), "planning_group: " << PLANNING_GROUP);
  RCLCPP_INFO_STREAM(this->get_logger(), "planning_frame: " << PLANNING_FRAME);
  RCLCPP_INFO_STREAM(this->get_logger(), "move_group_ns: " << MOVE_GROUP_NS);
    // MoveIt related things!
  moveGroupInit = setMoveGroup(node_, PLANNING_GROUP, MOVE_GROUP_NS);
  pSceneMonitorInit = setPlanningSceneMonitor(node_, ROBOT_DESC);
  robotModelInit = setRobotModel(node_);
    // Sanitize the namespace string. If "null", use empty string (root/default).
  std::string interface_ns = (MOVE_GROUP_NS == "null") ? "" : MOVE_GROUP_NS;
  m_planningSceneInterface =
    std::make_shared<moveit::planning_interface::PlanningSceneInterface>(interface_ns);
  RCLCPP_INFO(this->get_logger(), "PlanningSceneInterface initialized!");
}

std::unique_ptr<moveit_servo::Servo> m2Iface::init_servo()
{
    // New Jazzy API - use ParamListener instead of ServoParameters
  servo_param_listener_ = std::make_shared<servo::ParamListener>(node_, "moveit_servo");
  auto servo_params = servo_param_listener_->get_params();
  RCLCPP_INFO_STREAM(this->get_logger(), "Servo move_group_name: " << servo_params.move_group_name);

  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_param_listener_,
    m_pSceneMonitorPtr);
  RCLCPP_INFO(this->get_logger(), "Servo initialized!");
  return servo;
}

void m2Iface::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg || !robotModelInit || !kinematic_model || !m_robotStatePtr) {
    return;
  }

  std::vector<std::string> names;
  std::vector<double> pos;
  names.reserve(msg->name.size());
  pos.reserve(msg->position.size());

  bool gripper_seen = false;
  double gripper_m = 0.0;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (i >= msg->position.size()) {
      break;
    }
    const std::string & joint_name = msg->name[i];
    if (piper_joint_gripper_ && joint_name == std::string("gripper")) {
      gripper_seen = true;
      gripper_m = msg->position[i];
      continue;
    }
    if (!kinematic_model->hasJointModel(joint_name)) {
      continue;
    }
    names.push_back(joint_name);
    pos.push_back(msg->position[i]);
  }

  const bool piper_map_gripper =
    static_cast<bool>(piper_joint_gripper_) && gripper_seen &&
    kinematic_model->hasJointModel("joint7") && kinematic_model->hasJointModel("joint8");

  if (piper_map_gripper) {
    std::vector<std::string> n2;
    std::vector<double> p2;
    n2.reserve(names.size() + 2);
    p2.reserve(pos.size() + 2);
    for (size_t k = 0; k < names.size(); ++k) {
      if (names[k] == "joint7" || names[k] == "joint8") {
        continue;
      }
      n2.push_back(names[k]);
      p2.push_back(pos[k]);
    }
    const double half = gripper_m * 0.5;
    n2.push_back("joint7");
    p2.push_back(half);
    n2.push_back("joint8");
    p2.push_back(-half);
    names.swap(n2);
    pos.swap(p2);
  }

  if (names.empty()) {
    return;
  }

  try {
    m_robotStatePtr->setVariablePositions(names, pos);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "joint_state_cb: setVariablePositions failed (%s)", e.what());
  }
}

void m2Iface::pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (robotState != CART_TRAJ_CTL && robotState != JOINT_TRAJ_CTL) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Ignoring cmd_pose: set robot to JOINT_TRAJ_CTL or CART_TRAJ_CTL first.");
    return;
  }
  geometry_msgs::msg::PoseStamped cmd_plan;
  const std::string source_frame = msg->header.frame_id;
  if (source_frame.empty() || source_frame == PLANNING_FRAME) {
    cmd_plan = *msg;
    cmd_plan.header.frame_id = PLANNING_FRAME;
  } else {
    try {
      cmd_plan = tf_buffer_->transform(*msg, PLANNING_FRAME);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
                this->get_logger(),
                "cmd_pose: cannot transform '%s' -> '%s': %s",
                source_frame.c_str(), PLANNING_FRAME.c_str(), ex.what());
      return;
    }
  }
  std::lock_guard<std::mutex> lock(pose_topic_mutex_);
  m_topicPoseCmd = cmd_plan;
  m_topicPoseCmd.header.frame_id = PLANNING_FRAME;
    /* Ignore a repeat of the last successfully started pose — the arm is
     * already going there; a *failed* pose is not stored, so a re-click retries. */
  if (!utils::comparePose(m_topicPoseCmd, m_oldTopicPoseCmd)) {
    received_topic_pose_cmd_ = true;
  }
}

void m2Iface::cart_poses_cb(const arm_api2_msgs::msg::CartesianWaypoints::SharedPtr msg)
{
  if (robotState != CART_TRAJ_CTL) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Ignoring cmd_traj: set robot to Cartesian control (CART_TRAJ_CTL / GUI CARTESIAN_CONTROL) first.");
    return;
  }
  if (!msg || msg->poses.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Ignoring cmd_traj: at least two waypoints are required.");
    return;
  }
  m_cartesianWaypoints = msg->poses;
  received_topic_traj_cmd_ = true;
}

void m2Iface::servo_twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_twist_cmd_ = *msg;
  new_twist_cmd_ = true;
}

void m2Iface::processServoCommand()
{
  if (!servoPtr || !new_twist_cmd_ || !m_moveGroupPtr) {return;}

    // Wait 0.5s after entering servo mode to avoid processing old buffered commands
  auto time_since_servo_entered = (this->now() - servo_entered_time_).seconds();
  if (time_since_servo_entered < 0.5) {
    new_twist_cmd_ = false;
    return;
  }

  new_twist_cmd_ = false;

    // Ignore commands with all zero velocities to prevent old buffered commands
  bool all_zero = (std::abs(latest_twist_cmd_.twist.linear.x) < 1e-6 &&
    std::abs(latest_twist_cmd_.twist.linear.y) < 1e-6 &&
    std::abs(latest_twist_cmd_.twist.linear.z) < 1e-6 &&
    std::abs(latest_twist_cmd_.twist.angular.x) < 1e-6 &&
    std::abs(latest_twist_cmd_.twist.angular.y) < 1e-6 &&
    std::abs(latest_twist_cmd_.twist.angular.z) < 1e-6);
  if (all_zero) {
    return;
  }

  try {
        // Create TwistCommand for servo
    moveit_servo::TwistCommand twist_cmd;
    twist_cmd.frame_id =
      latest_twist_cmd_.header.frame_id.empty() ? EE_LINK_NAME : latest_twist_cmd_.header.frame_id;
    twist_cmd.velocities[0] = latest_twist_cmd_.twist.linear.x;
    twist_cmd.velocities[1] = latest_twist_cmd_.twist.linear.y;
    twist_cmd.velocities[2] = latest_twist_cmd_.twist.linear.z;
    twist_cmd.velocities[3] = latest_twist_cmd_.twist.angular.x;
    twist_cmd.velocities[4] = latest_twist_cmd_.twist.angular.y;
    twist_cmd.velocities[5] = latest_twist_cmd_.twist.angular.z;

        // Get current robot state
    auto current_state = m_moveGroupPtr->getCurrentState(1.0);
    if (!current_state) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Could not get current robot state");
      return;
    }

        // Set command type
    servoPtr->setCommandType(moveit_servo::CommandType::TWIST);

        // Get next joint state from servo
    moveit_servo::KinematicState next_state = servoPtr->getNextJointState(current_state, twist_cmd);

        // Check servo status and publish it
    auto status = servoPtr->getStatus();
    auto status_msg_str = servoPtr->getStatusMessage();

        // Publish ServoStatus
    moveit_msgs::msg::ServoStatus status_msg;
    status_msg.code = static_cast<int8_t>(status);
    status_msg.message = status_msg_str;
    servo_status_pub_->publish(status_msg);

    if (status == moveit_servo::StatusCode::INVALID ||
      status == moveit_servo::StatusCode::HALT_FOR_SINGULARITY ||
      status == moveit_servo::StatusCode::HALT_FOR_COLLISION)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Servo status: %s", status_msg_str.c_str());
      return;
    }

        // Check if we got valid output
    if (next_state.joint_names.empty() || next_state.positions.size() == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Servo returned empty state");
      return;
    }

    if (servo_use_forward_position_ && servo_forward_position_pub_) {
            // Forward position controller consumes raw joint positions
      std_msgs::msg::Float64MultiArray command;
      for (Eigen::Index i = 0; i < next_state.positions.size(); ++i) {
        command.data.push_back(next_state.positions[i]);
      }
      servo_forward_position_pub_->publish(command);
    } else {
            // Create and publish trajectory
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.header.stamp = this->now();
      traj_msg.header.frame_id = PLANNING_FRAME;
      traj_msg.joint_names = next_state.joint_names;

      trajectory_msgs::msg::JointTrajectoryPoint point;
      for (Eigen::Index i = 0; i < next_state.positions.size(); ++i) {
        point.positions.push_back(next_state.positions[i]);
        if (i < next_state.velocities.size()) {
          point.velocities.push_back(next_state.velocities[i]);
        }
      }
      point.time_from_start = rclcpp::Duration::from_seconds(0.1);
      traj_msg.points.push_back(point);

      servo_trajectory_pub_->publish(traj_msg);
    }
    last_servo_state_ = next_state;

  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Servo command processing failed: %s", e.what());
  }
}


void m2Iface::set_vel_acc_cb(
  const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Response> res)
{
  if(req->max_vel < 0 || req->max_acc < 0 || req->max_vel > 1 || req->max_acc > 1) {
    res->success = false;
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Velocity and acceleration must be in the range [0, 1]!");
    return;
  }
  max_vel_scaling_factor = float(req->max_vel);
  max_acc_scaling_factor = float(req->max_acc);
  res->success = true;
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Set velocity and acceleration to " << max_vel_scaling_factor << " " << max_acc_scaling_factor);

}

void m2Iface::set_eelink_cb(
  const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res)
{
  EE_LINK_NAME = req->value;
  m_moveGroupPtr->setEndEffectorLink(EE_LINK_NAME);
  res->success = true;
  RCLCPP_INFO_STREAM(this->get_logger(), "Set end effector link to " << req->value);
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
      "Unknown planner: " << planner_prefix << ". Supported: 'pilz', 'ompl'");
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
      "Successfully set planner to: " << planner_id << " / " << planner_type);
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

    // Add to planning scene
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  m_planningSceneInterface->addCollisionObjects(collision_objects);

  res->success = true;
  res->message = "Collision object added successfully";
  RCLCPP_INFO(this->get_logger(), "Collision object '%s' added to planning scene", req->id.c_str());
}

void m2Iface::open_gripper_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
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
  const bool success = sendGripperCmd(0.8);
  res->success = success;
  res->message = success ? "ok" : "failed";
  if (success && gripper_state_pub_) {
    std_msgs::msg::String state_msg;
    state_msg.data = "closed";
    gripper_state_pub_->publish(state_msg);
  }
}

rclcpp_action::GoalResponse m2Iface::move_to_joint_goal_cb(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const arm_api2_msgs::action::MoveJoint::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Received goal request for joint control! Number of joints: " <<
    goal->joint_state.position.size());
  if(robotState != JOINT_TRAJ_CTL) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in joint control mode!");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if(goal->joint_state.position.size() != m_currJointPosition.size()) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Number of joint positions does not match the number of joints!");
    return rclcpp_action::GoalResponse::REJECT;
  }
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_joint_cancel_cb(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>>
  goal_handle)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel joint control!");
  (void)goal_handle;
  m_moveGroupPtr->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_joint_accepted_cb(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>> goal_handle)
{
  m_moveToJointGoalHandle_ = goal_handle;
  received_cmd_ = true;
}

rclcpp_action::GoalResponse m2Iface::move_to_pose_goal_cb(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const arm_api2_msgs::action::MoveCartesian::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for Cartesian control!");
  if(robotState != CART_TRAJ_CTL) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in Cartesian control mode!");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if(goal->goal.header.frame_id != PLANNING_FRAME) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Pose frame_id is not planning frame! PLANNING_FRAME: " << PLANNING_FRAME);
    return rclcpp_action::GoalResponse::REJECT;
  }
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_pose_cancel_cb(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>>
  goal_handle)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel Cartesian control!");
  (void)goal_handle;
  m_moveGroupPtr->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_pose_accepted_cb(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>> goal_handle)
{
  m_moveToPoseGoalHandle_ = goal_handle;
  received_cmd_ = true;
}

rclcpp_action::GoalResponse m2Iface::move_to_pose_path_goal_cb(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const arm_api2_msgs::action::MoveCartesianPath::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for Cartesian path control!");
  if(robotState != CART_TRAJ_CTL) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in Cartesian control mode!");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if(goal->poses.size() < 2) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Number of poses in path is less than 2!");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if(goal->poses[0].header.frame_id != PLANNING_FRAME) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Path frame_id is not planning frame! PLANNING_FRAME: " << PLANNING_FRAME);
    return rclcpp_action::GoalResponse::REJECT;
  }
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_pose_path_cancel_cb(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>>
  goal_handle)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel Cartesian path control!");
  (void)goal_handle;
  m_moveGroupPtr->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_pose_path_accepted_cb(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>>
  goal_handle)
{
  m_moveToPosePathGoalHandle_ = goal_handle;
  received_traj_ = true;
}

rclcpp_action::GoalResponse m2Iface::gripper_control_goal_cb(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for gripper control!");
  (void)uuid;
  (void)goal;
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

void m2Iface::change_state_cb(
  const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res)
{
  auto itr = std::find(std::begin(stateNames), std::end(stateNames), req->state);

  if (itr != std::end(stateNames)) {
    int wantedIndex_ = std::distance(stateNames, itr);
    robotState = (state)wantedIndex_;
    RCLCPP_INFO_STREAM(this->get_logger(), "Switching state!");
    res->success = true;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Failed switching to state " << req->state);
    res->success = false;
  }
}

bool m2Iface::setMoveGroup(
  rclcpp::Node::SharedPtr nodePtr, std::string groupName,
  std::string moveNs)
{
    // check if moveNs is empty
  if (moveNs == "null") {moveNs = "";}

    //https://github.com/moveit/moveit2/issues/496
  m_moveGroupPtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(nodePtr,
        moveit::planning_interface::MoveGroupInterface::Options(
            groupName,
            "robot_description",
            moveNs));

  double POS_TOL = 0.001;
    // set move group stuff
  m_moveGroupPtr->setEndEffectorLink(EE_LINK_NAME);
  m_moveGroupPtr->setPoseReferenceFrame(PLANNING_FRAME);
  m_moveGroupPtr->setGoalPositionTolerance(POS_TOL);
  m_moveGroupPtr->startStateMonitor();

    // velocity scaling
  m_moveGroupPtr->setMaxVelocityScalingFactor(INIT_VEL_SCALING);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(INIT_ACC_SCALING);
    // executor
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() {executor_->spin();});
  RCLCPP_INFO_STREAM(this->get_logger(), "Move group interface set up!");
  return true;
}

bool m2Iface::setRobotModel(rclcpp::Node::SharedPtr nodePtr)
{
  robot_model_loader::RobotModelLoader robot_model_loader(nodePtr);
  kinematic_model = robot_model_loader.getModel();
    // Find nicer way to do this
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  m_robotStatePtr = kinematic_state;
  m_robotStatePtr->setToDefaultValues();
  RCLCPP_INFO_STREAM(this->get_logger(), "Robot model loaded!");
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Robot model frame is: " << kinematic_model->getModelFrame().c_str());
  return true;
}

bool m2Iface::setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name)
{
    // https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
    // https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/planning_scene/src/planning_scene_tutorial.cpp
  m_pSceneMonitorPtr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(nodePtr,
    name);
  m_pSceneMonitorPtr->startSceneMonitor(PLANNING_SCENE);
  if (m_pSceneMonitorPtr->getPlanningScene()) {
    m_pSceneMonitorPtr->startStateMonitor(JOINT_STATES);
    m_pSceneMonitorPtr->setPlanningScenePublishingFrequency(25);
    m_pSceneMonitorPtr->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
    m_pSceneMonitorPtr->startSceneMonitor();
    m_pSceneMonitorPtr->providePlanningSceneService();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning scene not configured!");
    return false;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Created planning scene monitor!");
  return true;
}

void m2Iface::planAndExecJoint()
{
  const auto feedback = std::make_shared<arm_api2_msgs::action::MoveJoint::Feedback>();
  const auto result = std::make_shared<arm_api2_msgs::action::MoveJoint::Result>();
  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(this->get_logger(), "Trajectory still executing; aborting joint goal.");
    result->success = false;
    m_moveToJointGoalHandle_->abort(result);
    return;
  }
  feedback->set__status("planning");
  m_moveToJointGoalHandle_->publish_feedback(feedback);

  const auto goalJointState = m_moveToJointGoalHandle_->get_goal()->joint_state;
  m_moveGroupPtr->setJointValueTarget(goalJointState);
  m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = planWithPlanner(plan, eager_execution);
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Planning to joint space goal: " << (success ? "SUCCEEDED" : "FAILED"));

  if (success && planOnly) {
    RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
    RCLCPP_INFO_STREAM(this->get_logger(), "                   Plan only mode!");
    RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
    result->success = true;
    m_moveToJointGoalHandle_->succeed(result);
  } else if (success) {
        //addTimestampsToTrajectory(plan.trajectory);
        //printTimestamps(plan.trajectory);

    feedback->set__status("executing");
    m_moveToJointGoalHandle_->publish_feedback(feedback);
    const bool execOk = execPlan_with_plan(plan, false);
    if(execOk) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Execution succeeded!");
      result->success = true;
      m_moveToJointGoalHandle_->succeed(result);
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Execution failed with error code");
      result->success = false;
      m_moveToJointGoalHandle_->abort(result);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    result->success = false;
    m_moveToJointGoalHandle_->abort(result);
  }
}

void m2Iface::planAndExecPose()
{
  const auto feedback = std::make_shared<arm_api2_msgs::action::MoveCartesian::Feedback>();
  const auto result = std::make_shared<arm_api2_msgs::action::MoveCartesian::Result>();
  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(this->get_logger(), "Trajectory still executing; aborting pose goal.");
    result->success = false;
    m_moveToPoseGoalHandle_->abort(result);
    return;
  }
  feedback->set__status("planning");
  m_moveToPoseGoalHandle_->publish_feedback(feedback);

  auto goalPose = m_moveToPoseGoalHandle_->get_goal()->goal;
  RCLCPP_INFO_STREAM(this->get_logger(), "Planning to Cartesian Pose!");
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Current pose is: " << m_currPoseState.pose.position.x << " " <<
    m_currPoseState.pose.position.y << " " << m_currPoseState.pose.position.z);
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Target pose is: " << goalPose.pose.position.x << " " << goalPose.pose.position.y << " " <<
    goalPose.pose.position.z);

  m_moveGroupPtr->setPoseTarget(goalPose);
  m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = planWithPlanner(plan, eager_execution);
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Planning to pose goal: " << (success ? "SUCCEEDED" : "FAILED"));

  if (success && planOnly) {
    RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
    RCLCPP_INFO_STREAM(this->get_logger(), "                   Plan only mode!");
    RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
    result->success = true;
    m_moveToPoseGoalHandle_->succeed(result);
  } else if(success) {
        //addTimestampsToTrajectory(plan.trajectory);
        //printTimestamps(plan.trajectory);

    feedback->set__status("executing");
    m_moveToPoseGoalHandle_->publish_feedback(feedback);
    const bool execOk = execPlan_with_plan(plan, false);
    if(execOk) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Execution succeeded!");
      result->success = true;
      m_moveToPoseGoalHandle_->succeed(result);
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Execution failed with error code, time stamps:");
      printTimestamps(plan.trajectory);
      result->success = false;
      m_moveToPoseGoalHandle_->abort(result);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    result->success = false;
    m_moveToPoseGoalHandle_->abort(result);
  }
}

void m2Iface::planAndExecTopicPose()
{
  geometry_msgs::msg::PoseStamped goalPose;
  {
    std::lock_guard<std::mutex> lock(pose_topic_mutex_);
    goalPose = m_topicPoseCmd;
  }

  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory still executing; ignoring new pose command until it finishes.");
    publishPlanStatus(false, "BUSY_TRAJECTORY_EXECUTING",
                          "Trajectory still executing; ignored new pose command",
                          goalPose, 0.0, "CART_TRAJ_CTL");
    return;
  }

  stopBeforeAsyncExecute();

  RCLCPP_INFO_STREAM(this->get_logger(), "Planning to Cartesian Pose from cmd_pose topic!");
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Current pose is: " << m_currPoseState.pose.position.x << " " <<
    m_currPoseState.pose.position.y << " " << m_currPoseState.pose.position.z);
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Target pose is: " << goalPose.pose.position.x << " " << goalPose.pose.position.y << " " <<
    goalPose.pose.position.z);

  m_moveGroupPtr->setPoseTarget(goalPose);
  m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

  const auto plan_t0 = std::chrono::steady_clock::now();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success = planTopicPoseFast(plan);
  const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() -
    plan_t0).count();
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Planning to topic pose goal: " << (success ? "SUCCEEDED" : "FAILED"));

  if (!success) {
    publishPlanStatus(false, "PLAN_NOT_FOUND",
                          "MoveIt planner did not return SUCCESS for this pose",
                          goalPose, plan_dt, "CART_TRAJ_CTL");
    return;
  }
  if (planOnly) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Plan only mode!");
    publishPlanStatus(true, "SUCCESS", "Plan only mode; not executing", goalPose, plan_dt,
      "CART_TRAJ_CTL");
    return;
  }

  const bool ok = execPlan_with_plan(plan, async);
  if (ok) {
    std::lock_guard<std::mutex> lock(pose_topic_mutex_);
    m_oldTopicPoseCmd = goalPose;
    publishPlanStatus(true, "SUCCESS", "Plan accepted; executing", goalPose, plan_dt,
      "CART_TRAJ_CTL");
  } else {
    publishPlanStatus(false, "EXECUTE_FAILED",
                          "Plan succeeded but execute() did not start",
                          goalPose, plan_dt, "CART_TRAJ_CTL");
  }
}

void m2Iface::planAndExecTopicPoseJoint()
{
  geometry_msgs::msg::PoseStamped pose_snap;
  {
    std::lock_guard<std::mutex> lock(pose_topic_mutex_);
    pose_snap = m_topicPoseCmd;
  }

  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory still executing; ignoring new pose command until it finishes.");
    publishPlanStatus(false, "BUSY_TRAJECTORY_EXECUTING",
                          "Trajectory still executing; ignored new pose command",
                          pose_snap, 0.0, "JOINT_TRAJ_CTL");
    return;
  }

  stopBeforeAsyncExecute();
  geometry_msgs::msg::PoseStamped cmdPose_ = utils::normalizeOrientation(pose_snap);
  m_moveGroupPtr->clearPoseTargets();
  m_moveGroupPtr->setPoseTarget(cmdPose_.pose, EE_LINK_NAME);
  m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
  if (pose_plan_time_sec_ > 0.0) {
    m_moveGroupPtr->setPlanningTime(pose_plan_time_sec_);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "poseTarget is: "    << cmdPose_.pose.position.x << " "
                                                              << cmdPose_.pose.position.y << " "
                                                              << cmdPose_.pose.position.z);
  const auto plan_t0 = std::chrono::steady_clock::now();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_err = m_moveGroupPtr->plan(plan);
  const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() -
    plan_t0).count();

  if (plan_err != moveit::core::MoveItErrorCode::SUCCESS) {
    const std::string code = utils::moveItErrorCodeToString(plan_err);
    RCLCPP_ERROR_STREAM(this->get_logger(),
            "Planning failed (" << code << ") in " << plan_dt << " s; not executing.");
    publishPlanStatus(false, code,
                          "MoveIt planner did not return SUCCESS for this pose",
                          cmdPose_, plan_dt, "JOINT_TRAJ_CTL");
    return;
  }

  if (planOnly) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Plan only mode!");
    publishPlanStatus(true, "SUCCESS", "Plan only mode; not executing", cmdPose_, plan_dt,
      "JOINT_TRAJ_CTL");
    return;
  }

  const bool ok = execPlan_with_plan(plan, async);
  if (ok) {
    std::lock_guard<std::mutex> lock(pose_topic_mutex_);
    m_oldTopicPoseCmd = cmdPose_;
    RCLCPP_INFO_STREAM(this->get_logger(), "Plan succeeded; executing commanded path.");
    publishPlanStatus(true, "SUCCESS", "Plan accepted; executing", cmdPose_, plan_dt,
      "JOINT_TRAJ_CTL");
  } else {
    publishPlanStatus(false, "EXECUTE_FAILED",
                          "Plan succeeded but execute() did not start",
                          cmdPose_, plan_dt, "JOINT_TRAJ_CTL");
  }
}

bool m2Iface::planTopicPoseFast(moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  const double original_planning_time = m_moveGroupPtr->getPlanningTime();
  const auto restore_planning_time = [this, original_planning_time]() {
      m_moveGroupPtr->setPlanningTime(original_planning_time);
    };

  struct PlannerAttempt
  {
    const char *pipeline;
    const char *planner;
    double planning_time;
  };
  const std::vector<PlannerAttempt> attempts = {
    {"pilz_industrial_motion_planner", "LIN", 0.25},
    {"ompl", "EST", 0.75},
  };

  for (size_t i = 0; i < attempts.size(); ++i) {
    const auto & attempt = attempts[i];
    m_moveGroupPtr->setPlanningPipelineId(attempt.pipeline);
    m_moveGroupPtr->setPlannerId(attempt.planner);
    m_moveGroupPtr->setPlanningTime(attempt.planning_time);

    moveit::planning_interface::MoveGroupInterface::Plan candidate;
    const bool success = static_cast<bool>(m_moveGroupPtr->plan(candidate));
    if (success) {
      RCLCPP_INFO(
                this->get_logger(),
                "%s found GUI topic pose plan %zu with %d points",
                attempt.planner,
                i,
                int(candidate.trajectory.joint_trajectory.points.size()));
      plan = candidate;
      restore_planning_time();
      return true;
    }
    RCLCPP_INFO(
            this->get_logger(),
            "%s failed GUI topic pose plan %zu",
            attempt.planner,
            i);
  }

  restore_planning_time();
  RCLCPP_WARN(this->get_logger(), "GUI topic pose planning failed fast; skipping queued nudge.");
  return false;
}

void m2Iface::planAndExecPosePath()
{
  const auto feedback = std::make_shared<arm_api2_msgs::action::MoveCartesianPath::Feedback>();
  const auto result = std::make_shared<arm_api2_msgs::action::MoveCartesianPath::Result>();
  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(this->get_logger(), "Trajectory still executing; aborting pose path goal.");
    result->success = false;
    m_moveToPosePathGoalHandle_->abort(result);
    return;
  }
  feedback->set__status("planning");
  m_moveToPosePathGoalHandle_->publish_feedback(feedback);

  auto goalPoseStampeds = m_moveToPosePathGoalHandle_->get_goal()->poses;
  RCLCPP_INFO_STREAM(this->get_logger(), "Planning Cartesian path!");
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Current pose is: " << m_currPoseState.pose.position.x << " " <<
    m_currPoseState.pose.position.y << " " << m_currPoseState.pose.position.z);
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Target pose is: " << goalPoseStampeds[goalPoseStampeds.size() - 1].pose.position.x << " " <<
    goalPoseStampeds[goalPoseStampeds.size() - 1].pose.position.y << " " <<
    goalPoseStampeds[goalPoseStampeds.size() - 1].pose.position.z);
  RCLCPP_INFO_STREAM(this->get_logger(), "Creating Cartesian waypoints!");
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Number of waypoints in the path: " << goalPoseStampeds.size());

    // extract all poses from the PoseStamped messages
  std::vector<geometry_msgs::msg::Pose> goalPoses;
  for (auto pose : goalPoseStampeds) {
    goalPoses.push_back(pose.pose);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
    // TODO: Set as params that can be configured in YAML!
  double eefStep = 0.02;
  bool success = (m_moveGroupPtr->computeCartesianPath(goalPoses, eefStep, trajectory,
    true) == 1.0);

  if (success && planOnly) {
    RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
    RCLCPP_INFO_STREAM(this->get_logger(), "                   Plan only mode!");
    RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
    result->success = true;
    m_moveToPosePathGoalHandle_->succeed(result);
  } else if(success) {
    addTimestampsToTrajectory(trajectory);

    feedback->set__status("executing");
    m_moveToPosePathGoalHandle_->publish_feedback(feedback);
    const bool execOk = execTrajectory(trajectory, false);
    if (execOk) {
      result->success = true;
      m_moveToPosePathGoalHandle_->succeed(result);
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Cartesian path execution failed.");
      result->success = false;
      m_moveToPosePathGoalHandle_->abort(result);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    result->success = false;
    m_moveToPosePathGoalHandle_->abort(result);
  }


}

void m2Iface::planAndExecTopicPosePath()
{
  if (m_cartesianWaypoints.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Topic Cartesian path needs at least two waypoints.");
    return;
  }

  RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Planning Cartesian path from cmd_traj topic with " << m_cartesianWaypoints.size() <<
      " waypoints.");

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = PLANNING_FRAME;
  target_pose.pose = m_cartesianWaypoints.back();

  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory still executing; ignoring Cartesian waypoint trajectory.");
    publishPlanStatus(false, "BUSY_TRAJECTORY_EXECUTING",
                          "Trajectory still executing; ignored Cartesian waypoint trajectory",
                          target_pose, 0.0, "CART_TRAJ_CTL");
    return;
  }

  stopBeforeAsyncExecute();
  m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
  m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

  const auto plan_t0 = std::chrono::steady_clock::now();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eefStep = 0.02;
  const double fraction =
    m_moveGroupPtr->computeCartesianPath(m_cartesianWaypoints, eefStep, trajectory, true);
  const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() -
    plan_t0).count();

  if (fraction <= 0.0 || trajectory.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(
            this->get_logger(),
            "Planning Cartesian path from topic failed (fraction=%f)",
            fraction);
    publishPlanStatus(false, "PLAN_NOT_FOUND",
                          "Cartesian path could not be computed (fraction=" +
      std::to_string(fraction) + ")",
                          target_pose, plan_dt, "CART_TRAJ_CTL");
    return;
  }

  if (planOnly) {
    RCLCPP_INFO(
            this->get_logger(),
            "Plan-only mode: topic Cartesian path planned (fraction=%f), not executing.",
            fraction);
    publishPlanStatus(true, "SUCCESS", "Plan only mode; not executing", target_pose, plan_dt,
      "CART_TRAJ_CTL");
    return;
  }

  addTimestampsToTrajectory(trajectory);
  const bool ok = execTrajectory(std::move(trajectory), async);
  if (ok) {
    const std::string reason = (fraction >= 0.999) ?
      "Cartesian plan executing (full path)" :
      "Cartesian plan executing (partial fraction=" + std::to_string(fraction) + ")";
    publishPlanStatus(true, "SUCCESS", reason, target_pose, plan_dt, "CART_TRAJ_CTL");
  } else {
    publishPlanStatus(false, "EXECUTE_FAILED",
                          "Cartesian plan ready but execute() did not start",
                          target_pose, plan_dt, "CART_TRAJ_CTL");
  }
}

void m2Iface::printTimestamps(const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  trajectory_msgs::msg::JointTrajectory jointTrajectory = trajectory.joint_trajectory;
  std::vector<std::string> joint_names = jointTrajectory.joint_names;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points = jointTrajectory.points;
  for (long unsigned int i = 0; i < points.size(); i++) {
    trajectory_msgs::msg::JointTrajectoryPoint point = points[i];
    rclcpp::Duration duration = point.time_from_start;
    RCLCPP_INFO_STREAM(this->get_logger(),
      "Point " << i << " - time_from_start [s]: " << duration.seconds());
  }
}

void m2Iface::addTimestampsToTrajectory(moveit_msgs::msg::RobotTrajectory & trajectory)
{
    // The trajectory created with computeCartesianPath() needs to be modified so it will include velocities as well.
    // reference: https://groups.google.com/g/moveit-users/c/MOoFxy2exT4
    // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(m_moveGroupPtr->getCurrentState()->getRobotModel(),
    PLANNING_GROUP);
    // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*m_moveGroupPtr->getCurrentState(), trajectory);
    // Thrid create a TimeOptimalTrajectoryGeneration object
  trajectory_processing::TimeOptimalTrajectoryGeneration iptp;
    // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt, max_vel_scaling_factor, max_acc_scaling_factor);
  RCLCPP_INFO_STREAM(this->get_logger(),
    "Computed time stamp " << (success ? "SUCCEEDED" : "FAILED"));
    // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory);
}

bool m2Iface::planWithPlanner(
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  bool eagerExecution)
{
    // Planning priority:
    // 1. LIN planner from pilz_industrial_motion_planner
    // 2. EST planner from ompl
    // 3. PRM planner from ompl
    // for EST and PRM create three plans and choose the best one
    //-----------------------------------------------------------------------------------------------
    // TODO: Add cuMotion planner to the list of planners

  std::vector<std::pair<std::string, std::string>> planners = {
    {"pilz_industrial_motion_planner", "LIN"},
    {"ompl", "EST"},
    {"ompl", "PRM"}
  };
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> all_plans;

  bool success = false;
  int tries_per_planner = 3;
  for(int i = 0; i < int(tries_per_planner * planners.size()); i++) {

    int planner_index = i / tries_per_planner;
    int planner_try = i % tries_per_planner;

    m_moveGroupPtr->setPlanningPipelineId(planners[planner_index].first);
    m_moveGroupPtr->setPlannerId(planners[planner_index].second);

    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    success = static_cast<bool>(m_moveGroupPtr->plan(plan_));

    if(success) {
      RCLCPP_INFO(this->get_logger(), "%s found plan %d with %d points",
                planners[planner_index].second.c_str(), i,
        int(plan_.trajectory.joint_trajectory.points.size()));
      if (eagerExecution) {     // if eager execution is true, return after first successful plan
                // Set local planned path as the plan to be executed
        plan = plan_;
        return true;
      }
      all_plans.push_back(plan_);
    } else {
      RCLCPP_INFO(this->get_logger(), "%s failed to find plan %d",
                planners[planner_index].second.c_str(), i);
    }

    if(planner_try == tries_per_planner - 1 && all_plans.size() >= 3) {
      RCLCPP_INFO(this->get_logger(), "Found %d plans, stopping planning", int(all_plans.size()));
    }
  }

  if(all_plans.size() == 0) {
    RCLCPP_INFO_STREAM(this->get_logger(), "All planners failed!");
    return false;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Found " << all_plans.size() << " plans!");
  }

    // find the best plan from the list of plans
  auto best_plan = std::min_element(all_plans.begin(), all_plans.end(),
      [](auto const & a, auto const & b){
        return a.trajectory.joint_trajectory.points.size() <
               b.trajectory.joint_trajectory.points.size();
    });

  plan = *best_plan;
  RCLCPP_INFO(this->get_logger(), "Best plan selected with %d points.",
    int(best_plan->trajectory.joint_trajectory.points.size()));
  return true;
}

void m2Iface::getArmState()
{
  if (!m_robotStatePtr || !m_moveGroupPtr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MoveIt state is not ready");
    return;
  }
  const moveit::core::JointModelGroup * joint_model_group =
    m_robotStatePtr->getJointModelGroup(PLANNING_GROUP);
  if (!joint_model_group) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Planning group '%s' not found",
            PLANNING_GROUP.c_str());
    return;
  }
  m_robotStatePtr->copyJointGroupPositions(joint_model_group, m_currJointPosition);
  m_robotStatePtr = m_moveGroupPtr->getCurrentState(0.1);
  if (!m_robotStatePtr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to get current state");
    return;
  }
  m_robotStatePtr->update();

  Eigen::Isometry3d currentPose_ =
    m_moveGroupPtr->getCurrentState()->getFrameTransform(EE_LINK_NAME);
  m_currPoseState = utils::convertIsometryToMsg(currentPose_);
  auto frame_id = m_moveGroupPtr->getPlanningFrame().c_str();
  m_currPoseState.header.frame_id = frame_id;
  m_currPoseState.header.stamp = this->now();
}

void m2Iface::stopBeforeAsyncExecute()
{
  if (!m_moveGroupPtr) {
    return;
  }
  m_moveGroupPtr->stop();
    /* Allow the trajectory execution action to settle; short sleeps alone were not always enough. */
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
}

bool m2Iface::execPlan_with_plan(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  bool async_flag)
{
  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory already executing; skipping execute.");
    return false;
  }

  if (!async_flag) {
    trajectory_executing_.store(true, std::memory_order_release);
    const auto exec_err = m_moveGroupPtr->execute(plan);
    trajectory_executing_.store(false, std::memory_order_release);
    if (exec_err != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "execute(plan) did not report SUCCESS");
    }
    return exec_err == moveit::core::MoveItErrorCode::SUCCESS;
  }

    /* Async: non-blocking for the timer / callback thread — blocking execute runs on a worker thread.
     * A single-flight guard (trajectory_executing_) prevents overlapping MoveGroup executions (SIGSEGV). */
  trajectory_executing_.store(true, std::memory_order_release);
  try {
    std::lock_guard<std::mutex> lock(execution_thread_mutex_);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread([this, plan]() {
          try {
            const auto exec_err = m_moveGroupPtr->execute(plan);
            if (exec_err != moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_WARN(this->get_logger(),
            "execute(plan) in async worker did not report SUCCESS");
            }
          } catch (const std::exception & ex) {
            RCLCPP_ERROR(this->get_logger(), "execute(plan) async worker exception: %s", ex.what());
          } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "execute(plan) async worker unknown exception");
          }
          trajectory_executing_.store(false, std::memory_order_release);
        });
  } catch (const std::system_error & e) {
    trajectory_executing_.store(false, std::memory_order_release);
    RCLCPP_ERROR(this->get_logger(), "Failed to spawn async execute thread: %s", e.what());
    return false;
  }

  return true;
}

bool m2Iface::execTrajectory(moveit_msgs::msg::RobotTrajectory trajectory, bool async_flag)
{
  if (trajectory.joint_trajectory.points.empty()) {
    RCLCPP_WARN(this->get_logger(), "execTrajectory: empty trajectory, skipping execute");
    return false;
  }

  if (trajectory_executing_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory already executing; skipping duplicate execTrajectory.");
    return false;
  }

  if (!async_flag) {
    trajectory_executing_.store(true, std::memory_order_release);
    const auto exec_err = m_moveGroupPtr->execute(trajectory);
    trajectory_executing_.store(false, std::memory_order_release);
    if (exec_err != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "execute(trajectory) did not report SUCCESS");
    }
    return exec_err == moveit::core::MoveItErrorCode::SUCCESS;
  }

  trajectory_executing_.store(true, std::memory_order_release);
  try {
    std::lock_guard<std::mutex> lock(execution_thread_mutex_);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread([this, trajectory]() {
          try {
            const auto exec_err = m_moveGroupPtr->execute(trajectory);
            if (exec_err != moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_WARN(this->get_logger(),
            "execute(trajectory) in async worker did not report SUCCESS");
            }
          } catch (const std::exception & ex) {
            RCLCPP_ERROR(this->get_logger(), "execute(trajectory) async worker exception: %s",
          ex.what());
          } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "execute(trajectory) async worker unknown exception");
          }
          trajectory_executing_.store(false, std::memory_order_release);
        });
  } catch (const std::system_error & e) {
    trajectory_executing_.store(false, std::memory_order_release);
    RCLCPP_ERROR(this->get_logger(), "Failed to spawn async execTrajectory thread: %s", e.what());
    return false;
  }

  return true;
}

void m2Iface::publishPlanStatus(
  bool success,
  const std::string & error_code,
  const std::string & reason,
  const geometry_msgs::msg::PoseStamped & requested_pose,
  double plan_time_sec,
  const std::string & mode)
{
  if (!plan_status_pub_) {return;}
  arm_api2_msgs::msg::PlanStatus msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = PLANNING_FRAME;
  msg.success = success;
  msg.error_code = error_code;
  msg.reason = reason;
  msg.requested_pose = requested_pose;
  msg.plan_time_sec = plan_time_sec;
  msg.mode = mode;
  plan_status_pub_->publish(msg);
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
                                           existing.getVariablePosition(
          jm->getFirstVariableIndex()));
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

  const double eef_step = (req->eef_step > 0.0) ? req->eef_step : 0.02;
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

bool m2Iface::loadController(const std::string & controller_name)
{
  if (!load_controller_client_) {
    RCLCPP_ERROR(this->get_logger(), "load_controller client is not initialized");
    return false;
  }
  if (!load_controller_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "controller_manager/load_controller service not available");
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  request->name = controller_name;
  auto future = load_controller_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "load_controller('%s') timed out", controller_name.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    RCLCPP_WARN(
            this->get_logger(),
            "load_controller('%s') returned false; continuing in case it is already loaded",
            controller_name.c_str());
  }
  return true;
}

bool m2Iface::configureController(const std::string & controller_name)
{
  if (!configure_controller_client_) {
    RCLCPP_ERROR(this->get_logger(), "configure_controller client is not initialized");
    return false;
  }
  if (!configure_controller_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(),
      "controller_manager/configure_controller service not available");
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
  request->name = controller_name;
  auto future = configure_controller_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "configure_controller('%s') timed out",
      controller_name.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    RCLCPP_WARN(
            this->get_logger(),
            "configure_controller('%s') returned false; continuing in case it is already configured",
            controller_name.c_str());
  }
  return true;
}

bool m2Iface::switchControllers(
  const std::vector<std::string> & activate,
  const std::vector<std::string> & deactivate)
{
  if (!switch_controller_client_) {
    RCLCPP_ERROR(this->get_logger(), "switch_controller client is not initialized");
    return false;
  }
  if (!switch_controller_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "controller_manager/switch_controller service not available");
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  request->activate_asap = true;
  request->timeout.sec = 2;
  request->timeout.nanosec = 0;

  auto future = switch_controller_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "switch_controller call timed out");
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    RCLCPP_ERROR(
            this->get_logger(),
            "switch_controller failed (activate=%zu, deactivate=%zu)",
            activate.size(),
            deactivate.size());
    return false;
  }
  return true;
}

bool m2Iface::enterServoControllerMode()
{
  if (!servo_use_forward_position_) {
    return true;
  }
  if (forward_position_controller_active_) {
    return true;
  }
  (void)loadController(servo_forward_controller_);
  (void)configureController(servo_forward_controller_);
  const bool ok = switchControllers({servo_forward_controller_}, {servo_default_controller_});
  if (ok) {
    forward_position_controller_active_ = true;
    RCLCPP_INFO(this->get_logger(), "%s active for SERVO_CTL", servo_forward_controller_.c_str());
  }
  return ok;
}

bool m2Iface::leaveServoControllerMode()
{
  if (!servo_use_forward_position_) {
    return true;
  }
  if (!forward_position_controller_active_) {
    return true;
  }
  (void)loadController(servo_default_controller_);
  (void)configureController(servo_default_controller_);
  const bool ok = switchControllers({servo_default_controller_}, {servo_forward_controller_});
  if (ok) {
    forward_position_controller_active_ = false;
    RCLCPP_INFO(this->get_logger(), "%s restored", servo_default_controller_.c_str());
  }
  return ok;
}

bool m2Iface::run()
{
  if(!nodeInit) {RCLCPP_ERROR(this->get_logger(), "Node not fully initialized!"); return false;}
  if(!moveGroupInit) {
    RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized!"); return false;
  }

  getArmState();
  pose_state_pub_->publish(m_currPoseState);
  std_msgs::msg::String stateMsg;
  stateMsg.data = stateNames[robotState];
  robot_state_pub_->publish(stateMsg);

  rclcpp::Clock steady_clock;
  int LOG_STATE_TIMEOUT = 10000;

    // STATE MACHINE
  if (robotState == IDLE) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT,
      "arm_api2 is in IDLE mode.");
  } else {
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT,
      "arm_api2 is in " << stateNames[robotState] << " mode.");
  }

    // Check if servo active, to deactivate before sending to another pose
  if (robotState != SERVO_CTL && servoEntered) {
    servoPtr->setCollisionChecking(false);
    servoEntered = false;
    new_twist_cmd_ = false;
    (void)leaveServoControllerMode();
  }                                                                                                           // New API: no setPaused

    /* Single-shot semantics: a topic command is consumed exactly once. If planning
     * fails, the failure is surfaced via plan_status; we do NOT silently retry the
     * same goal on every tick. */
  if (robotState == JOINT_TRAJ_CTL) {
    if (received_cmd_) {
      planAndExecJoint();
      received_cmd_ = false;
    }

    if (received_topic_pose_cmd_) {
      received_topic_pose_cmd_ = false;
      planAndExecTopicPoseJoint();
    }
  }

  if (robotState == CART_TRAJ_CTL) {
        // TODO: Beware if both are true at the same time, shouldn't occur,
    if (received_cmd_) {
      planAndExecPose();
      received_cmd_ = false;
    }

    if (received_topic_pose_cmd_) {
      received_topic_pose_cmd_ = false;
      planAndExecTopicPose();
    }

    if (received_traj_) {
      planAndExecPosePath();
      received_traj_ = false;
    }

    if (received_topic_traj_cmd_) {
      received_topic_traj_cmd_ = false;
      planAndExecTopicPosePath();
    }
  }

  if (robotState == SERVO_CTL && servoPtr) {
    if (!servoEntered) {
            // Clear any old twist commands
      latest_twist_cmd_ = geometry_msgs::msg::TwistStamped();
      latest_twist_cmd_.header.frame_id = PLANNING_FRAME;
      latest_twist_cmd_.twist.linear.x = 0.0;
      latest_twist_cmd_.twist.linear.y = 0.0;
      latest_twist_cmd_.twist.linear.z = 0.0;
      latest_twist_cmd_.twist.angular.x = 0.0;
      latest_twist_cmd_.twist.angular.y = 0.0;
      latest_twist_cmd_.twist.angular.z = 0.0;
      new_twist_cmd_ = false;

            // Moveit servo status codes: https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/include/moveit_servo/utils/datatypes.hpp
      try {
                // Send a zero twist command to servo to clear internal state
        moveit_servo::TwistCommand zero_twist;
        zero_twist.frame_id = PLANNING_FRAME;
        zero_twist.velocities.fill(0.0);
        auto current_state = m_moveGroupPtr->getCurrentState(1.0);
        if (current_state) {
          servoPtr->setCommandType(moveit_servo::CommandType::TWIST);
          servoPtr->getNextJointState(current_state, zero_twist);
        }
                /* A sim planning scene with fixtures/gripper geometry close to the
                 * robot can report near-collision continuously; allow disabling. */
        servoPtr->setCollisionChecking(servo_collision_checking_);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Servo initialization failed: %s", e.what());
      }
      if (!enterServoControllerMode()) {
        RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Cannot enter SERVO_CTL: failed to activate %s",
                    servo_forward_controller_.c_str());
        return false;
      }
      RCLCPP_INFO(this->get_logger(),
        "Servo mode activated! Send twist commands to ~/servo_twist_cmd");
      servo_entered_time_ = this->now();
      servoEntered = true;
    }
        // Process servo commands every cycle
    processServoCommand();

  } else if (robotState == SERVO_CTL && !servoPtr) {
    RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "SERVO_CTL requested but servo is not enabled.");
  }

  if(received_gripper_cmd_) {

    auto goal = m_gripperControlGoalHandle_->get_goal();
    float position = goal->command.position;
    float effort = goal->command.max_effort;

    auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
    bool success = sendGripperCmd(position, effort);

    if(success) {
            // TODO: Wrap this in methods
      RCLCPP_INFO_STREAM(this->get_logger(), "Gripper command succeeded!");
      result->position = gripperMeasuredPositionNormalized();
      result->effort = gripperMeasuredEffort();
      result->stalled = gripperMeasuredStalled();
      result->reached_goal = gripperMeasuredReachedGoal();
      m_gripperControlGoalHandle_->succeed(result);
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Gripper command failed!");
      result->reached_goal = gripperMeasuredReachedGoal();
      m_gripperControlGoalHandle_->abort(result);
    }
    received_gripper_cmd_ = false;
  }

  return true;
}

bool m2Iface::sendGripperCmd(double normalized_position_robotiq, double max_effort)
{
  if (piper_joint_gripper_) {
    return piper_joint_gripper_->send_gripper_command(normalized_position_robotiq, max_effort);
  }
  return gripper_.send_gripper_command(normalized_position_robotiq, max_effort);
}

float m2Iface::gripperMeasuredPositionNormalized()
{
  if (piper_joint_gripper_) {
    return piper_joint_gripper_->get_position();
  }
  return gripper_.get_position();
}

float m2Iface::gripperMeasuredEffort()
{
  if (piper_joint_gripper_) {
    return piper_joint_gripper_->get_effort();
  }
  return gripper_.get_effort();
}

bool m2Iface::gripperMeasuredStalled()
{
  if (piper_joint_gripper_) {
    return piper_joint_gripper_->is_stalled();
  }
  return gripper_.is_stalled();
}

bool m2Iface::gripperMeasuredReachedGoal()
{
  if (piper_joint_gripper_) {
    return piper_joint_gripper_->reached_goal();
  }
  return gripper_.reached_goal();
}
