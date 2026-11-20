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
 *      Description : Node lifecycle: construction, publishers/subscribers/services wiring, MoveIt setup, arm state machine (run loop, change_state) and gripper glue.
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
                                   "Unknown mode '" << mode_ <<
      "', expected 'simple' or 'advanced'. Using 'advanced'.");
    mode_ = "advanced";
  }
  this->get_parameter("enable_topics", enable_topics);
  this->get_parameter("enable_actions", enable_actions);
  this->get_parameter("dt", dt);
  this->get_parameter("gripper_override", gripper_override_);

  RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config!");

  std::chrono::duration<double> SYSTEM_DT(dt);
  timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&m2Iface::run, this));

    // Load arm basically --> two important params
    // Manual param specification --> https://github.com/moveit/moveit2_tutorials/blob/8eaef05bfbabde3f35910ad054a819d79e70d3fc/doc/tutorials/quickstart_in_rviz/launch/demo.launch.py#L105
  config = init_config(config_path);
  validateConfig();
  PLANNING_GROUP = config["robot"]["arm_name"].as<std::string>();
  EE_LINK_NAME = config["robot"]["ee_link_name"].as<std::string>();
  ROBOT_DESC = config["robot"]["robot_desc"].as<std::string>();
  PLANNING_FRAME = config["robot"]["planning_frame"].as<std::string>();
  MOVE_GROUP_NS = config["robot"]["move_group_ns"].IsNull() ? std::string() :
    config["robot"]["move_group_ns"].as<std::string>();
  JOINT_STATES = config["robot"]["joint_states"].as<std::string>();
  INIT_VEL_SCALING = 0.05;
  INIT_ACC_SCALING = 0.05;
  eager_execution = true;
  max_vel_scaling_factor = config["robot"]["max_vel_scaling_factor"].as<float>();
  max_acc_scaling_factor = config["robot"]["max_acc_scaling_factor"].as<float>();
  if (config["robot"]["max_joint_step_rad"]) {
    max_joint_step_rad_ = std::max(0.0, config["robot"]["max_joint_step_rad"].as<double>());
  }

    /* Optional override for OMPL planning time. Default 1.5 s — see hpp comment. */
  if (config["robot"]["pose_plan_time_sec"]) {
    const double v = config["robot"]["pose_plan_time_sec"].as<double>();
    if (v > 0.0) {pose_plan_time_sec_ = v;}
  }

  if (config["robot"]["cartesian_path_min_fraction"]) {
    const double v = config["robot"]["cartesian_path_min_fraction"].as<double>();
    if (v > 0.0 && v <= 1.0) {cartesian_path_min_fraction_ = v;}
  }

  if (config["robot"]["goal_joint_tolerance_rad"]) {
    goal_joint_tolerance_rad_ = config["robot"]["goal_joint_tolerance_rad"].as<double>();
  }
  if (config["robot"]["goal_settle_sec"]) {
    goal_settle_sec_ = config["robot"]["goal_settle_sec"].as<double>();
  }
  if (config["robot"]["path_deviation_max_rad"]) {
    path_deviation_max_rad_ = config["robot"]["path_deviation_max_rad"].as<double>();
  }
  loadPlanningConfig();
  if (config["robot"]["async_actions"]) {
    async_actions_ = config["robot"]["async_actions"].as<bool>();
  }
  if (config["robot"]["on_state_change_while_moving"]) {
    const auto v = config["robot"]["on_state_change_while_moving"].as<std::string>();
    if (v == "reject") {
      stop_on_state_change_ = false;
    } else if (v != "stop") {
      RCLCPP_WARN_STREAM(this->get_logger(), "robot.on_state_change_while_moving '" << v <<
        "' unknown (stop|reject); using stop");
    }
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
    if (!gripper_override_.empty()) {
      // launch arg gripper:=none (e.g. welding gun stacks sharing the gripper profile)
      action_type_val = gripper_override_;
    }
    gripper_none_ = action_type_val && *action_type_val == std::string("none");

    if (gripper_none_) {
      RCLCPP_INFO(this->get_logger(), "No gripper on this robot/tool (gripper: none)");
    } else if (action_type_val && *action_type_val == std::string("piper_joint")) {
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
  init_execution();
  init_motion_log();
  init_moveit();  // starts executor_thread_
  try {
    init_diagnostics();
    loadNamedPoses();
    init_scene_interfaces();
    init_task_interfaces();
    if (enable_actions) {
      init_actionservers();
    }
    if (enable_servo) {servoPtr = init_servo();}
  } catch (...) {
    // a joinable thread must not be destroyed during unwinding (std::terminate)
    if (executor_) {executor_->cancel();}
    if (executor_thread_.joinable()) {executor_thread_.join();}
    throw;
  }

  RCLCPP_INFO_STREAM(this->get_logger(),
    "Initialized node! mode: " << mode_ << ", topics: " << enable_topics <<
    ", actions: " << enable_actions);

    // Seed the previous pose with an impossible value so the first command is always accepted.
  m_oldTopicPoseCmd.pose.position.x = 5.0;
  // Started last: a constructor exception after this point would destroy a joinable thread
  // (std::terminate instead of the clean "Startup failed" exit).
  {
      // Heartbeat for /arm/state/ctl_state while run() blocks in a synchronous execute.
      // Publishing from another thread is thread-safe in rclcpp.
    state_heartbeat_gap_ns_ = static_cast<int64_t>(std::max(2.0 * dt, 0.2) * 1e9);
    state_heartbeat_thread_ = std::thread([this]() {
          while (!state_heartbeat_stop_.load() && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            publishStateHeartbeat();
          }
                });
  }
  nodeInit = true;
}

m2Iface::~m2Iface()
{
  state_heartbeat_stop_.store(true);
  if (state_heartbeat_thread_.joinable()) {
    state_heartbeat_thread_.join();
  }
  cancelTask("shutdown");
  cancelExecution("shutdown");
  if (task_thread_.joinable()) {
    task_thread_.join();
  }
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
                                                                                        std::bind(
    &m2Iface::joint_state_cb, this, _1));
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
                                   "Subscribed to Cartesian waypoint topic: " <<
      cart_traj_cmd_name);
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
                                                                                        std::bind(
    &m2Iface::change_state_cb, this, _1, _2));
  set_vel_acc_srv_ = this->create_service<arm_api2_msgs::srv::SetVelAcc>(set_vel_acc_name,
                                                                                        std::bind(
    &m2Iface::set_vel_acc_cb, this, _1, _2));
  set_planner_srv_ = this->create_service<arm_api2_msgs::srv::SetStringParam>(set_planner_name,
                                                                                        std::bind(
    &m2Iface::set_planner_cb, this, _1, _2));
  set_eelink_srv_ = this->create_service<arm_api2_msgs::srv::SetStringParam>(set_eelink_name,
                                                                                        std::bind(
    &m2Iface::set_eelink_cb, this, _1, _2));
  set_plan_only_srv_ = this->create_service<std_srvs::srv::SetBool>(set_plan_only_name,
                                                                                        std::bind(
    &m2Iface::set_plan_only_cb, this, _1, _2));
  std::string add_collision_object_name = "add_collision_object";
  if (config["srv"]["add_collision_object"] && config["srv"]["add_collision_object"]["name"]) {
    add_collision_object_name = config["srv"]["add_collision_object"]["name"].as<std::string>();
  }
  add_collision_object_srv_ =
    this->create_service<arm_api2_msgs::srv::AddCollisionObject>(add_collision_object_name,
                                                                                        std::bind(
    &m2Iface::add_collision_object_cb, this, _1, _2));
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
                                                                                              std::
      bind(
                                                                                                      &
    m2Iface::move_to_pose_goal_cb, this, _1, _2),
                                                                                              std::
      bind(
                                                                                                      &
    m2Iface::move_to_pose_cancel_cb, this, _1),
                                                                                              std::
      bind(
                                                                                                      &
    m2Iface::move_to_pose_accepted_cb, this, _1));
  move_to_joint_as_ = rclcpp_action::create_server<arm_api2_msgs::action::MoveJoint>(this,
                                                                                        move_to_joint_name,
                                                                                           std::bind
      (
                                                                                                   &
    m2Iface::move_to_joint_goal_cb, this, _1, _2),
                                                                                           std::bind
      (
                                                                                                   &
    m2Iface::move_to_joint_cancel_cb, this, _1),
                                                                                           std::bind
      (
                                                                                                   &
    m2Iface::move_to_joint_accepted_cb, this, _1));
  move_to_pose_path_as_ =
    rclcpp_action::create_server<arm_api2_msgs::action::MoveCartesianPath>(this,
                                                                                        move_to_pose_path_name,
                                                                                        std::bind(
                                                                                               &
    m2Iface::move_to_pose_path_goal_cb, this, _1, _2),
                                                                                        std::bind(
                                                                                               &
    m2Iface::move_to_pose_path_cancel_cb, this, _1),
                                                                                        std::bind(
                                                                                               &
    m2Iface::move_to_pose_path_accepted_cb, this, _1));
  gripper_control_as_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(this,
                                                                                        gripper_control_name,
                                                                                                 std
      ::bind(
                                                                                                         &
    m2Iface::gripper_control_goal_cb, this, _1, _2),
                                                                                                 std
      ::bind(
                                                                                                         &
    m2Iface::gripper_control_cancel_cb, this, _1),
                                                                                                 std
      ::bind(
                                                                                                         &
    m2Iface::gripper_control_accepted_cb, this, _1));

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
  init_tcp_state();
    // Sanitize the namespace string. If "null", use empty string (root/default).
  std::string interface_ns = (MOVE_GROUP_NS == "null") ? "" : MOVE_GROUP_NS;
  m_planningSceneInterface =
    std::make_shared<moveit::planning_interface::PlanningSceneInterface>(interface_ns);
  RCLCPP_INFO(this->get_logger(), "PlanningSceneInterface initialized!");
}

void m2Iface::init_tcp_state()
{
  arm_api2::TcpStateConfig cfg;
  cfg.joint_states_topic = JOINT_STATES;
  cfg.group = PLANNING_GROUP;
  cfg.tip_link = EE_LINK_NAME;
  const YAML::Node t = config["tcp_state"];
  if (t) {
    if (t["enabled"]) {cfg.enabled = t["enabled"].as<bool>();}
    if (t["pose_topic"]) {cfg.pose_topic = t["pose_topic"].as<std::string>();}
    if (t["twist_topic"]) {cfg.twist_topic = t["twist_topic"].as<std::string>();}
    if (t["filter_tau_s"]) {cfg.filter_tau_s = t["filter_tau_s"].as<double>();}
    if (t["max_dt_s"]) {cfg.max_dt_s = t["max_dt_s"].as<double>();}
    if (t["velocity_source"]) {
      const auto src = t["velocity_source"].as<std::string>();
      if (src == "joint_velocity") {
        cfg.velocity_source = arm_api2::TcpVelocitySource::JointVelocity;
      } else if (src == "finite_difference") {
        cfg.velocity_source = arm_api2::TcpVelocitySource::FiniteDifference;
      } else if (src != "auto") {
        RCLCPP_WARN(this->get_logger(), "Unknown tcp_state.velocity_source '%s', using auto",
          src.c_str());
      }
    }
  }
  if (!cfg.enabled || !kinematic_model) {
    return;
  }
  tcp_state_pub_ = std::make_unique<arm_api2::TcpStatePublisher>(node_, *this, kinematic_model,
    cfg);
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

void m2Iface::change_state_cb(
  const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req,
  const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res)
{
  namespace logic = arm_api2::logic;
  const auto wanted = logic::parseState(req->state);
  if (!wanted) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Failed switching to state " << req->state);
    res->success = false;
    return;
  }
  switch (logic::decideStateChange(static_cast<logic::ArmState>(robotState), *wanted,
                                         motionBusy(), stop_on_state_change_))
  {
    case logic::StateChange::REJECT_MOVING:
      RCLCPP_WARN_STREAM(this->get_logger(), "Refusing state change to " << req->state <<
            " while a trajectory is executing (robot.on_state_change_while_moving: reject)");
      res->success = false;
      return;
    case logic::StateChange::STOP_AND_SWITCH:
      cancelTask("arm state changed to " + req->state);
      cancelExecution("arm state changed to " + req->state);
      [[fallthrough]];
    case logic::StateChange::SWITCH:
      // A command queued for the old state must not run when that state comes back.
      dropPendingCommands("arm state changed to " + req->state);
      break;
    case logic::StateChange::SAME_STATE:
      break;
  }
  robotState = static_cast<state>(*wanted);
  published_state_.store(static_cast<int>(robotState));
  RCLCPP_INFO_STREAM(this->get_logger(), "Switching state!");
  res->success = true;
}

void m2Iface::dropPendingCommands(const std::string & why)
{
  auto abort_pending = [&](auto & handle, auto result) {
      if (handle && handle->is_active()) {
        result->success = false;
        handle->abort(result);
        RCLCPP_WARN_STREAM(this->get_logger(), "Aborted a queued action goal: " << why);
      }
      handle.reset();
    };
  if (received_cmd_) {
    abort_pending(m_moveToJointGoalHandle_,
      std::make_shared<arm_api2_msgs::action::MoveJoint::Result>());
    abort_pending(m_moveToPoseGoalHandle_,
      std::make_shared<arm_api2_msgs::action::MoveCartesian::Result>());
  }
  if (received_traj_) {
    abort_pending(m_moveToPosePathGoalHandle_,
      std::make_shared<arm_api2_msgs::action::MoveCartesianPath::Result>());
  }
  if (received_topic_pose_cmd_ || received_topic_traj_cmd_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Dropped a queued topic command: " << why);
  }
  for (auto & job : task_jobs_) {
    job.abort(why);
  }
  task_jobs_.clear();
  received_cmd_ = false;
  received_traj_ = false;
  received_topic_pose_cmd_ = false;
  received_topic_traj_cmd_ = false;
}

bool m2Iface::setMoveGroup(
  rclcpp::Node::SharedPtr nodePtr, std::string groupName,
  std::string moveNs)
{
    // check if moveNs is empty
  if (moveNs == "null") {moveNs = "";}

    //https://github.com/moveit/moveit2/issues/496
  m_moveGroupPtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(nodePtr,
                                                                                          moveit::
      planning_interface::MoveGroupInterface::Options(
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
  executor_thread_ = std::thread([this]() {
        executor_->spin();
        });
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
  // Single robot (root namespace): the historic absolute names. Namespaced robots
  // (robot_namespaces:=ur1;ur2) get their own scene topic and services.
  std::string mg_ns = MOVE_GROUP_NS;
  while (!mg_ns.empty() && mg_ns.back() == '/') {mg_ns.pop_back();}
  if (!mg_ns.empty() && mg_ns.front() != '/') {mg_ns = "/" + mg_ns;}
  std::string node_ns = (ns_ == "/") ? "" : ns_;
  const std::string scene_topic = mg_ns + "/monitored_planning_scene";
  const std::string scene_srv = node_ns + "/arm_api2/get_planning_scene";
  const std::string servo_scene_topic = node_ns + "/moveit_servo/publish_planning_scene";
  RCLCPP_INFO_STREAM(this->get_logger(), "Planning scene: monitoring " << scene_topic <<
    ", service " << scene_srv << ", servo scene " << servo_scene_topic);
  m_pSceneMonitorPtr->startSceneMonitor(scene_topic);
  if (m_pSceneMonitorPtr->getPlanningScene()) {
    m_pSceneMonitorPtr->startStateMonitor(JOINT_STATES);
    m_pSceneMonitorPtr->setPlanningScenePublishingFrequency(25);
    m_pSceneMonitorPtr->startPublishingPlanningScene(
                        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
      servo_scene_topic);
    m_pSceneMonitorPtr->providePlanningSceneService(scene_srv);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning scene not configured!");
    return false;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Created planning scene monitor!");
  return true;
}

void m2Iface::getArmState()
{
  if (!m_moveGroupPtr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MoveIt state is not ready");
    return;
  }

  // Single fetch — never call getCurrentState() twice (second call can nullptr
  // under joint_states/DDS stalls and used to segfault on ->getFrameTransform).
  auto current_state = m_moveGroupPtr->getCurrentState(0.1);
  if (!current_state) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to get current state");
    return;
  }
  m_robotStatePtr = current_state;

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
  m_robotStatePtr->update();

  Eigen::Isometry3d currentPose_ = m_robotStatePtr->getFrameTransform(EE_LINK_NAME);
  m_currPoseState = utils::convertIsometryToMsg(currentPose_);
  auto frame_id = m_moveGroupPtr->getPlanningFrame().c_str();
  m_currPoseState.header.frame_id = frame_id;
  m_currPoseState.header.stamp = this->now();
}

void m2Iface::publishStateHeartbeat()
{
    // Only while run() is busy (it has published before and not for a while): the
    // normal per-tick publication is unchanged, the topic just never goes silent.
  const int64_t last = last_state_pub_ns_.load();
  if (last == 0 || !robot_state_pub_) {return;}
  const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  if (now - last < state_heartbeat_gap_ns_) {return;}
  const int idx = published_state_.load();
  if (idx < 0 || idx > 3) {return;}
  std_msgs::msg::String msg;
  msg.data = stateNames[idx];
  robot_state_pub_->publish(msg);
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
  published_state_.store(static_cast<int>(robotState));
  last_state_pub_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());

  rclcpp::Clock steady_clock;
  int LOG_STATE_TIMEOUT = 10000;

    // STATE MACHINE
  if (robotState == IDLE) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT,
      "arm_api2 is in IDLE mode.");
  } else {
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT,
                                            "arm_api2 is in " << stateNames[robotState] <<
      " mode.");
  }

    // gripper: none -> tell the GUI (it hides Open/Close); 1 Hz so late subscribers see it
  if (gripper_none_ && gripper_state_pub_ &&
    std::chrono::steady_clock::now() - last_gripper_none_pub_ > std::chrono::seconds(1))
  {
    std_msgs::msg::String none_msg;
    none_msg.data = "none";
    gripper_state_pub_->publish(none_msg);
    last_gripper_none_pub_ = std::chrono::steady_clock::now();
  }

    // arm/stop (node_ thread) asked to drop everything queued and halt servo motion
  if (stop_requested_.exchange(false)) {
    dropPendingCommands("arm/stop");
    latest_twist_cmd_ = geometry_msgs::msg::TwistStamped();
    latest_twist_cmd_.header.frame_id = PLANNING_FRAME;
    new_twist_cmd_ = false;
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

  // queued task goals (execute_task, execute_plan, move_relative, move_to_named)
  if ((robotState == JOINT_TRAJ_CTL || robotState == CART_TRAJ_CTL) && !task_jobs_.empty()) {
    processTaskJobs();
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
  if (gripper_none_) {return false;}
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
