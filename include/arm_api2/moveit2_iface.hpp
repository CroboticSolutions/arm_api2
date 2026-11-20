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

/*******************************************************************************
*
 * Copyright (c) 2024, Crobotic Solutions d.o.o. (www.crobotics.tech)
 * All rights reserved.
*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
*
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
*
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
*
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
*
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title       : arm2_action_iface.hpp
 *      Project     : arm_api2
 *      Created     : 08/06/2025
 *      Author      : Filip Zoric
 *
 *      Description : The core robot manipulator and MoveIt2! ROS 2 interfacing header class.
 */

#ifndef MOVEIT2_IFACE_HPP
#define MOVEIT2_IFACE_HPP

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <functional>
#include <string>

//* yaml params
#include <yaml-cpp/yaml.h>

//* ros
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <future>
#include <unordered_map>
#include <vector>

//* moveit
#include <moveit_servo/servo.hpp>
#include <moveit_servo/moveit_servo_lib_parameters.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <pluginlib/class_loader.hpp>

//* msgs
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arm_api2_msgs/msg/cartesian_waypoints.hpp"
#include "arm_api2_msgs/msg/plan_status.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/servo_status.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

//* srvs
#include "arm_api2_msgs/srv/change_state.hpp"
#include "arm_api2_msgs/srv/add_collision_object.hpp"
#include "arm_api2_msgs/srv/set_vel_acc.hpp"
#include "arm_api2_msgs/srv/set_string_param.hpp"
#include "arm_api2_msgs/srv/check_cartesian_path.hpp"
#include "arm_api2_msgs/srv/check_reachability.hpp"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "arm_api2_msgs/action/move_cartesian.hpp"
#include "arm_api2_msgs/action/move_joint.hpp"
#include "arm_api2_msgs/action/move_cartesian_path.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "arm_api2_msgs/msg/task_step.hpp"
#include "arm_api2_msgs/msg/task_segment.hpp"
#include "arm_api2_msgs/srv/plan_task.hpp"
#include "arm_api2_msgs/srv/save_named_pose.hpp"
#include "arm_api2_msgs/srv/list_named_poses.hpp"
#include "arm_api2_msgs/srv/name_request.hpp"
#include "arm_api2_msgs/srv/attach_object.hpp"
#include "arm_api2_msgs/srv/detach_object.hpp"
#include "arm_api2_msgs/srv/add_collision_mesh.hpp"
#include "arm_api2_msgs/srv/list_collision_objects.hpp"
#include "arm_api2_msgs/srv/set_path_constraints.hpp"
#include "arm_api2_msgs/srv/clear_path_constraints.hpp"
#include "arm_api2_msgs/action/execute_task.hpp"
#include "arm_api2_msgs/action/execute_plan.hpp"
#include "arm_api2_msgs/action/move_relative.hpp"
#include "arm_api2_msgs/action/move_to_named.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_msgs/msg/constraints.hpp"

// utils
#include "arm_api2/utils.hpp"
#include "arm_api2/motion_logic.hpp"
#include "arm_api2/motion_log.hpp"
#include "arm_api2/task_logic.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "controller_manager_msgs/srv/list_controllers.hpp"

using arm_api2::MotionLog;
using arm_api2::MotionRecord;

// For starters just include robotiq_gripper
// TODO: Think of a way to include different gripper based on the gripper type
#include "arm_api2/grippers/gripper.hpp"
#include "arm_api2/grippers/piper_joint_gripper.hpp"
#include "arm_api2/grippers/robotiq_gripper.hpp"
#include "arm_api2/tcp_state_publisher.hpp"

#define stringify(name) #name

// *std placeholders
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class m2Iface : public rclcpp::Node
{

public:
  m2Iface(const rclcpp::NodeOptions & options);
  ~m2Iface() override;

        /* namespace param, maybe redundant */
  std::string ns_;

private:
        /* node related stuff */
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;

/** Set while a trajectory is being executed (from send to result). */
  std::atomic_bool trajectory_executing_{false};

        /* gripper: Robotiq (action) OR Piper JointState shim */
  RobotiqGripper gripper_;
  std::unique_ptr<PiperJointGripper> piper_joint_gripper_;

        /* arm_definition */
  std::string PLANNING_GROUP;
  std::string EE_LINK_NAME;
  std::string ROBOT_DESC;
  std::string PLANNING_FRAME;
  std::string MOVE_GROUP_NS;
  std::string JOINT_STATES;
  std::string SERVO_TRAJECTORY_TOPIC;
  bool ENABLE_SERVO;
  float INIT_VEL_SCALING;
  float INIT_ACC_SCALING;

        /* timers */
  rclcpp::TimerBase::SharedPtr                                        timer_;
        /* ctl_state heartbeat: run() executes plans synchronously, so the state topic
         * would go silent for the whole motion (launch_server health probes it and
         * tears the stack down after a few seconds). A plain thread (not an executor
         * callback, so the node keeps its single-threaded spin and callback order)
         * republishes the last state while run() is busy. */
  std::thread                                                         state_heartbeat_thread_;
  std::atomic<bool> state_heartbeat_stop_{false};
  std::atomic<int> published_state_{0};
  std::atomic<int64_t> last_state_pub_ns_{0};
  int64_t                                                             state_heartbeat_gap_ns_{0};
  void publishStateHeartbeat();

        /* parameters */
  std::string                                                         config_path;
        /* "simple" enables just the topic interface; "advanced" adds action servers. */
  std::string                                                         mode_{"advanced"};
  bool                                                                enable_servo;
  bool                                                                enable_topics{true};
  bool                                                                enable_actions{true};
  bool                                                                eager_execution;
  float                                                               dt;
  float                                                               max_vel_scaling_factor;
  float                                                               max_acc_scaling_factor;
  /* Absolute per-point joint change allowed in Cartesian paths (rad, 0 = off).
     Consecutive points are <= eefStep apart, so more is an IK branch flip or a
     +-2pi wrap (full J4/J6 turn on CRX); computeCartesianPath has no jump check. */
  double                                                              max_joint_step_rad_{0.5};

        /* planning budget for OMPL pose targets (seconds). 5 s is the MoveIt
         * default, but for an interactive GUI 1.5 s is plenty: a healthy goal
         * solves in << 100 ms, and an infeasible goal doesn't get more honest
         * with more time. Reduces user-visible latency on bad clicks 3×. */
  double pose_plan_time_sec_ = 1.5;

        /* servo output: publish JointTrajectory (default) or switch to
         * forward_position_controller and publish Float64MultiArray. */
  bool servo_use_forward_position_{false};
  bool servo_collision_checking_{true};
  bool forward_position_controller_active_ = false;
  std::string servo_forward_controller_{"forward_position_controller"};
  std::string servo_default_controller_{"scaled_joint_trajectory_controller"};

        /* planner info */
  std::string current_planner_id_ = "pilz_industrial_motion_planner";
  std::string current_planner_type_ = "LIN";

        /* config_file */
  YAML::Node config;
        /*config*/
  YAML::Node init_config(std::string yaml_path);

        /* init methods */
  void init_subscribers();
  void init_publishers();
  void init_services();
  void init_actionservers();
  void init_moveit();
  std::unique_ptr<moveit_servo::Servo> init_servo();

        /* subs */
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr           joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr       pose_cmd_sub_;
  rclcpp::Subscription<arm_api2_msgs::msg::CartesianWaypoints>::SharedPtr ctraj_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr       servo_twist_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr     servo_trajectory_pub_;
  rclcpp::Publisher<moveit_msgs::msg::ServoStatus>::SharedPtr            servo_status_pub_;

        /* pubs */
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       pose_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 robot_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 gripper_state_pub_;
  rclcpp::Publisher<arm_api2_msgs::msg::PlanStatus>::SharedPtr        plan_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      servo_forward_position_pub_;

        /* srvs */
  rclcpp::Service<arm_api2_msgs::srv::ChangeState>::SharedPtr              change_state_srv_;
  rclcpp::Service<arm_api2_msgs::srv::SetVelAcc>::SharedPtr                set_vel_acc_srv_;
  rclcpp::Service<arm_api2_msgs::srv::SetStringParam>::SharedPtr           set_planner_srv_;
  rclcpp::Service<arm_api2_msgs::srv::SetStringParam>::SharedPtr           set_eelink_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                      set_plan_only_srv_;
  rclcpp::Service<arm_api2_msgs::srv::AddCollisionObject>::SharedPtr      add_collision_object_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                       open_gripper_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                       close_gripper_srv_;
  rclcpp::Service<arm_api2_msgs::srv::CheckReachability>::SharedPtr        check_reachability_srv_;
  rclcpp::Service<arm_api2_msgs::srv::CheckCartesianPath>::SharedPtr       check_cartesian_path_srv_;
  rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr
    configure_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr   load_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;

        /* actions */
  rclcpp_action::Server<arm_api2_msgs::action::MoveJoint>::SharedPtr              move_to_joint_as_;
  rclcpp_action::Server<arm_api2_msgs::action::MoveCartesian>::SharedPtr          move_to_pose_as_;
  rclcpp_action::Server<arm_api2_msgs::action::MoveCartesianPath>::SharedPtr
    move_to_pose_path_as_;
  rclcpp_action::Server<control_msgs::action::GripperCommand>::SharedPtr
    gripper_control_as_;

        /* topic callbacks */
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cart_poses_cb(const arm_api2_msgs::msg::CartesianWaypoints::SharedPtr msg);
  void servo_twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void processServoCommand();

        /* srv callbacks*/
  void change_state_cb(
    const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req,
    const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res);
  void set_vel_acc_cb(
    const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Request> req,
    const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Response> res);
  void set_planner_cb(
    const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req,
    const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res);
  void set_eelink_cb(
    const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req,
    const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res);
  void set_plan_only_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void add_collision_object_cb(
    const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Request> req,
    const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Response> res);
  void check_reachability_cb(
    const std::shared_ptr<arm_api2_msgs::srv::CheckReachability::Request> req,
    const std::shared_ptr<arm_api2_msgs::srv::CheckReachability::Response> res);
  void check_cartesian_path_cb(
    const std::shared_ptr<arm_api2_msgs::srv::CheckCartesianPath::Request> req,
    const std::shared_ptr<arm_api2_msgs::srv::CheckCartesianPath::Response> res);
  void open_gripper_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void close_gripper_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

        /* action callbacks */
  rclcpp_action::GoalResponse move_to_joint_goal_cb(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const arm_api2_msgs::action::MoveJoint::Goal> goal);
  rclcpp_action::CancelResponse move_to_joint_cancel_cb(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>>
    goal_handle);
  void move_to_joint_accepted_cb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>> goal_handle);

  rclcpp_action::GoalResponse move_to_pose_goal_cb(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const arm_api2_msgs::action::MoveCartesian::Goal> goal);
  rclcpp_action::CancelResponse move_to_pose_cancel_cb(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>>
    goal_handle);
  void move_to_pose_accepted_cb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>>
    goal_handle);

  rclcpp_action::GoalResponse move_to_pose_path_goal_cb(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const arm_api2_msgs::action::MoveCartesianPath::Goal> goal);
  rclcpp_action::CancelResponse move_to_pose_path_cancel_cb(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>>
    goal_handle);
  void move_to_pose_path_accepted_cb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>>
    goal_handle);

  rclcpp_action::GoalResponse gripper_control_goal_cb(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal);
  rclcpp_action::CancelResponse gripper_control_cancel_cb(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>>
    goal_handle);
  void gripper_control_accepted_cb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>>
    goal_handle);

  bool sendGripperCmd(double normalized_position_robotiq, double max_effort = 140.0);
  float gripperMeasuredPositionNormalized();
  float gripperMeasuredEffort();
  bool gripperMeasuredStalled();
  bool gripperMeasuredReachedGoal();

  bool run();

        /* setters */
  bool setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs);
  bool setRobotModel(rclcpp::Node::SharedPtr nodePtr);
  bool setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name);

        /* getters */
  void getArmState();

        /* controller switching for servo forward_position output */
  bool loadController(const std::string & controller_name);
  bool configureController(const std::string & controller_name);
  bool switchControllers(
    const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate);
  bool enterServoControllerMode();
  bool leaveServoControllerMode();

        /* Publishes a PlanStatus snapshot for the most recent plan attempt. */
  void publishPlanStatus(
    bool success,
    const std::string & error_code,
    const std::string & reason,
    const geometry_msgs::msg::PoseStamped & requested_pose,
    double plan_time_sec,
    const std::string & mode,
    bool final_status = true);     // false: "plan accepted; executing" (motion not finished yet)

        /* funcs */
  void planAndExecJoint();
  void planAndExecPose();
  void planAndExecTopicPose();
  void planAndExecTopicPoseJoint();
  void planAndExecPosePath();
  void planAndExecTopicPosePath();
        /** Execute an already-planned trajectory (joint-space Plan). Single-flight
         *  guarded; async runs the blocking execute on a worker thread. */
  /** Called once an async execute finishes: (succeeded, MoveIt error code string). */
  using ExecDone = std::function<void (bool, const std::string &)>;
  bool execPlan_with_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    bool async_flag, ExecDone on_done = nullptr);
  bool execTrajectory(
    moveit_msgs::msg::RobotTrajectory trajectory, bool async_flag,
    ExecDone on_done = nullptr);
/** Progress of a running execution: (elapsed s, planned duration s). */
  using ExecProgress = std::function<void (double, double)>;
  using ExecTraj = moveit_msgs::action::ExecuteTrajectory;

/* Trajectory execution goes to move_group's ExecuteTrajectory action through a
 * client on the internal node_ (spun by the MultiThreadedExecutor), never through
 * MoveGroupInterface::execute(). Result, progress feedback, cancel and arm/stop
 * therefore work while the public node is busy in run(). One execution at a time. */
  struct ActiveExec
  {
    uint64_t id{0};
    const void * owner{nullptr};            // action goal handle (nullptr: topic command)
    rclcpp_action::ClientGoalHandle<ExecTraj>::SharedPtr handle;
    bool cancel_requested{false};
    bool cancel_sent{false};
    std::string cancel_reason;
    std::chrono::steady_clock::time_point start;
    double planned_duration{0.0};
    ExecDone on_done;
    ExecProgress on_progress;
    trajectory_msgs::msg::JointTrajectory traj;     // planned, for deviation / goal checks
    double max_deviation{0.0};
    std::string max_deviation_joint;
    bool verifying{false};                       // result SUCCEEDED, waiting for goal tolerance
    std::chrono::steady_clock::time_point verify_deadline;
    std::string result_code;
    MotionRecord record;                         // motion log line, written by finishExecution
    double final_error{NAN};
    std::string final_error_joint;
  };
/* ---- observability: permanent motion log (JSONL) ---- */
  MotionLog motion_log_;
  std::mutex motion_mutex_;
  MotionRecord motion_;               // motion being planned (owned by motion_owner_ thread)
  std::thread::id motion_owner_;
  std::string last_planner_;          // set by planWithPlanner / planTopicPoseFast
  void init_motion_log();
  void motionBegin(const std::string & command, const std::string & source);
  void motionPlanned(
    const std::string & planner, double plan_time_sec, double fraction,
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const geometry_msgs::msg::PoseStamped * target);
/* A final plan_status on the planning thread closes a motion that never started executing. */
  void motionStatus(
    bool success, const std::string & code, const std::string & reason,
    const geometry_msgs::msg::PoseStamped & pose, double plan_time_sec);
/* startExecution: the planned motion record moves into the execution. */
  MotionRecord takeMotionForExecution();
  void writeMotion(MotionRecord & r);

/* ---- observability: /diagnostics ---- */
  void init_diagnostics();
  void diagMoveGroup(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagSceneMonitor(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagJointStates(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagExecution(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagGripper(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagControllers(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagArmState(diagnostic_updater::DiagnosticStatusWrapper & stat);
  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
  rclcpp::CallbackGroup::SharedPtr diag_cb_group_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  std::mutex diag_mutex_;
  std::vector<controller_manager_msgs::msg::ControllerState> diag_controllers_;
  bool diag_controllers_valid_{false};
  bool diag_controllers_pending_{false};
  std::chrono::steady_clock::time_point diag_controllers_time_;
  struct ExecStats
  {
    uint64_t succeeded{0};
    uint64_t failed{0};
    bool has_last{false};
    bool last_ok{false};
    std::string last_code;
    std::string last_reason;
    std::chrono::steady_clock::time_point last_time;
  } exec_stats_;                      // guarded by exec_mutex_

  rclcpp_action::Client<ExecTraj>::SharedPtr exec_client_;
  rclcpp::CallbackGroup::SharedPtr exec_cb_group_;
  rclcpp::CallbackGroup::SharedPtr stop_cb_group_;
  rclcpp::TimerBase::SharedPtr exec_progress_timer_;
  std::mutex exec_mutex_;
  ActiveExec active_exec_;
  uint64_t exec_seq_{0};
  void init_execution();
/** Send a timed trajectory; false if it could not be sent (busy, empty, no server).
 *  on_done is called exactly once when it was sent. */
  bool startExecution(
    const moveit_msgs::msg::RobotTrajectory & trajectory, ExecDone on_done,
    ExecProgress on_progress = nullptr, const void * owner = nullptr);
/** Same, waiting for the result (the result arrives on node_'s executor). */
  bool runExecutionBlocking(
    const moveit_msgs::msg::RobotTrajectory & trajectory, ExecProgress on_progress,
    const void * owner, std::string & code);
/** Cancel the running execution (only if it belongs to owner, when owner is set). */
  bool cancelExecution(const std::string & why, const void * owner = nullptr);
  void finishExecution(uint64_t id, bool ok, std::string code);
/** move_group reported SUCCEEDED: check the final joint error (goal tolerance). */
  void onExecutionSucceeded(uint64_t id, const std::string & moveit_code);

/* Execution checks against the robot's own joint_states (subscription on node_).
 * robot.goal_joint_tolerance_rad: after SUCCEEDED the joints must be within this of
 *   the last trajectory point (within robot.goal_settle_sec), else GOAL_NOT_REACHED.
 * robot.path_deviation_max_rad: during execution the joints must stay within this of
 *   the planned path (geometric, time independent), else the motion is stopped
 *   (PATH_DEVIATION). 0 = off; the measured values are always logged and reported. */
  double goal_joint_tolerance_rad_{0.0};
  double goal_settle_sec_{0.5};
  double path_deviation_max_rad_{0.0};
  rclcpp::CallbackGroup::SharedPtr exec_js_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr exec_js_sub_;
  std::mutex exec_js_mutex_;
  std::unordered_map<std::string, double> exec_js_;
  std::chrono::steady_clock::time_point exec_js_time_;
/** Copy of the latest joint positions; false if none or older than max_age_s. */
  bool latestJoints(std::unordered_map<std::string, double> & out, double max_age_s);

/* Configuration: validated as a whole before anything is created; all problems are
 * reported at once and the node exits with a clear message. */
  void validateConfig();
/* YAML planning: (defaults = the previously hard-coded behaviour) */
  struct PlannerAttempt
  {
    std::string pipeline;
    std::string planner;
    double time{0.0};     // planning time per attempt, 0 = leave MoveGroup's value
    int attempts{1};
  };
  std::vector<PlannerAttempt> pose_planners_;        // move_to_pose / move_to_joint
  std::vector<PlannerAttempt> topic_pose_planners_;  // arm/cmd/pose in CART_TRAJ_CTL
  double eef_step_{0.02};
  void loadPlanningConfig();

/* robot.gripper_action_type: none (or launch arg gripper:=none): no gripper backend;
 * open/close fail at once with a clear message, gripper_state publishes "none". */
  bool gripper_none_{false};
  std::string gripper_override_;
  std::chrono::steady_clock::time_point last_gripper_none_pub_;
  void execProgressTick();
/* Action goal helpers: feedback with progress, and the final succeed/canceled/abort. */
  template<class ActionT>
  ExecProgress actionProgress(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle);
  template<class ActionT>
  ExecDone actionDone(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle,
    const std::string & label, const geometry_msgs::msg::PoseStamped & pose,
    double plan_time_sec, const std::string & mode);
  template<class ActionT>
  void executeForGoal(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle,
    const moveit_msgs::msg::RobotTrajectory & trajectory, const std::string & label,
    const geometry_msgs::msg::PoseStamped & pose, double plan_time_sec);
/** plan_status for an action goal (same codes as the topic paths, reason "<label>: ..."). */
  void publishActionStatus(
    const std::string & label, bool success, const std::string & code,
    const std::string & reason, const geometry_msgs::msg::PoseStamped & pose,
    double plan_time_sec, bool final_status = true);
  template<class ActionT>
  bool finishIfCanceling(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle);

/* arm/stop: on node_ (own callback group), so it answers during planning and
 * synchronous executions. Cancels arm_api2's execution and sends MoveIt "stop"
 * (stops any trajectory move_group executes, also ones started by other clients). */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  void stop_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
/* set by stop_cb (node_ thread), consumed in run(): drop queued goals/commands, zero servo */
  std::atomic<bool> stop_requested_{false};

/* YAML robot.async_actions (default true): action goals execute without blocking
 * run(), so action cancel works and state/pose keep publishing.
 * false = previous blocking behaviour (arm/stop still works). */
  bool async_actions_{true};
/* YAML robot.on_state_change_while_moving: "stop" (default) cancels arm_api2's
 * running execution on a state change, "reject" refuses the change. */
  bool stop_on_state_change_{true};

        /** Final plan_status of an async topic execution; a failed one also forgets the
         *  last commanded pose so the same pose can be sent again. */
  ExecDone topicExecDone(
    const geometry_msgs::msg::PoseStamped & pose, double plan_time_sec,
    const std::string & mode, bool forget_pose);
        /** Abort accepted action goals and drop queued topic commands (state change). */
  void dropPendingCommands(const std::string & why);
        /** Stop active trajectory before planning a new one. Overlap is also prevented by trajectory_executing_. */
  void stopBeforeAsyncExecute();
  void printTimestamps(const moveit_msgs::msg::RobotTrajectory & trajectory);
  bool addTimestampsToTrajectory(moveit_msgs::msg::RobotTrajectory & trajectory);
  /* false (and reason) when two consecutive points differ by more than max_joint_step_rad_ */
  bool jointPathContinuous(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    std::string & reason) const;
  bool planWithPlanner(
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    bool eagerExecution = true);
  bool planTopicPoseFast(moveit::planning_interface::MoveGroupInterface::Plan & plan);

        // Simple state machine
  enum state
  {
    IDLE = 0,
    JOINT_TRAJ_CTL = 1,
    CART_TRAJ_CTL = 2,
    SERVO_CTL = 3
  };

        // stateNames
  const char * stateNames[4] =
  {
    stringify (IDLE),
    stringify (JOINT_TRAJ_CTL),
    stringify (CART_TRAJ_CTL),
    stringify (SERVO_CTL)
  };

        // robot state
  enum state robotState = IDLE;

        /* flags*/
  bool moveGroupInit = false;
  bool robotModelInit = false;
  bool pSceneMonitorInit = false;
  bool nodeInit = false;
  bool received_cmd_ = false;
  bool received_topic_pose_cmd_ = false;
  bool received_traj_ = false;
  bool received_topic_traj_cmd_ = false;
  bool received_gripper_cmd_ = false;
  bool servoEntered = false;
  rclcpp::Time servo_entered_time_;
  geometry_msgs::msg::TwistStamped latest_twist_cmd_;
  std::atomic<bool> new_twist_cmd_{false};
  moveit_servo::KinematicState last_servo_state_;
        /* topic-commanded executions run async (worker thread) by default */
  bool async = true;
  bool planOnly = false;
        /* topic Cartesian path (arm/cmd/traj): below this fraction nothing is executed
         * (YAML robot.cartesian_path_min_fraction; same 0.98 as the grasp descent gate) */
  double cartesian_path_min_fraction_ = 0.98;

        /* goal handles */
  std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>>
  m_moveToJointGoalHandle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>>
  m_moveToPoseGoalHandle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>>
  m_moveToPosePathGoalHandle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>>
  m_gripperControlGoalHandle_;

        /* ros vars */
  std::vector<double> m_currJointPosition;
  geometry_msgs::msg::PoseStamped m_currPoseState;
  std::vector<geometry_msgs::msg::Pose> m_cartesianWaypoints;
  geometry_msgs::msg::PoseStamped m_topicPoseCmd;
        /* last successfully started topic pose; used to ignore duplicate commands */
  geometry_msgs::msg::PoseStamped m_oldTopicPoseCmd;
  std::mutex pose_topic_mutex_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  moveit::planning_interface::MoveGroupInterfacePtr m_moveGroupPtr;
  moveit::core::RobotStatePtr m_robotStatePtr;
  moveit::core::RobotModelPtr kinematic_model;

        /* TCP pose + twist on every joint_states message (runs on the internal node executor) */
  std::unique_ptr<arm_api2::TcpStatePublisher> tcp_state_pub_;
  void init_tcp_state();
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> m_pSceneMonitorPtr;
  std::shared_ptr<servo::ParamListener> servo_param_listener_;
  std::unique_ptr<moveit_servo::Servo> servoPtr;
  moveit::planning_interface::PlanningSceneInterfacePtr m_planningSceneInterface;

/* ===================== tasks: plan -> preview -> execute =====================
 * moveit2_iface_task.cpp. A task is a list of TaskStep; every motion step is planned
 * from the end state of the previous one in a virtual copy of the planning scene
 * (attach/detach steps change that copy), so nothing moves unless the whole task
 * planned. Plans are stored by id (arm/plan) and executed later (arm/execute_plan),
 * or planned and executed in one goal (arm/execute_task, arm/move_relative,
 * arm/move_to_named). Execution runs on a worker thread, segment by segment, through
 * the same startExecution path (arm/stop, cancel, goal checks, motion log). */
  using TaskStep = arm_api2_msgs::msg::TaskStep;
  struct PlannedSegment
  {
    TaskStep step;
    int index{0};
    moveit_msgs::msg::RobotTrajectory trajectory;  // timed; empty for non-motion steps
    std::string planner;
    double duration{0.0};
  };
  struct StoredPlan
  {
    std::vector<PlannedSegment> segments;
    moveit_msgs::msg::RobotState start_state;
    double duration{0.0};
  };
  struct TaskPlanResult
  {
    bool ok{false};
    int failed_step{-1};
    std::string code;
    std::string message;
    std::shared_ptr<StoredPlan> plan;
    double plan_time_sec{0.0};
  };
  struct TaskProgress
  {
    std::string status;
    int step{0};
    int total{0};
    std::string name;
    double step_progress{0.0};
    double progress{0.0};
    double step_elapsed{0.0};
    double step_planned{0.0};
  };
  using TaskFeedbackFn = std::function<void (const TaskProgress &)>;
  using TaskDoneFn = std::function<void (bool ok, const std::string & code,
      const std::string & message, int failed_step)>;

  arm_api2::logic::PlanStore<StoredPlan> plan_store_{32, 600.0};
  double plan_start_tolerance_rad_{0.02};   // YAML plans.start_tolerance_rad
  std::vector<std::string> attach_touch_links_cfg_;   // YAML robot.attach_touch_links

  TaskPlanResult planTask(
    const std::vector<TaskStep> & steps,
    const std::function<void(int, const std::string &)> & on_step = nullptr);
  /* One motion step in the virtual scene; fills seg.trajectory/planner/duration. */
  bool planPoseOrJoint(
    planning_scene::PlanningScenePtr vscene, const moveit::core::RobotState & start,
    const std::vector<double> * joint_target, const geometry_msgs::msg::PoseStamped * pose_target,
    const std::string & planner_spec, double vel, double acc, PlannedSegment & seg,
    std::string & code, std::string & error);
  bool planCartesian(
    planning_scene::PlanningScenePtr vscene, const moveit::core::RobotState & start,
    const EigenSTL::vector_Isometry3d & waypoints, double vel, double acc,
    PlannedSegment & seg, std::string & code, std::string & error);
  bool resolvePoseInModel(
    const geometry_msgs::msg::PoseStamped & in, const moveit::core::RobotState & state,
    Eigen::Isometry3d & out, std::string & error);
  std::string stepLabel(const TaskStep & step, int index) const;

  std::atomic<bool> task_active_{false};
  std::atomic<bool> task_cancel_{false};
  std::atomic<const void *> task_owner_{nullptr};   // goal handle of the running task
  std::mutex task_mutex_;
  std::string task_cancel_reason_;
  std::thread task_thread_;
  bool motionBusy() const {return trajectory_executing_.load() || task_active_.load();}
  bool startTask(
    std::shared_ptr<StoredPlan> plan, const void * owner, TaskFeedbackFn fb, TaskDoneFn done,
    std::string & why);
  void taskWorker(
    std::shared_ptr<StoredPlan> plan, const void * owner, TaskFeedbackFn fb, TaskDoneFn done);
  void cancelTask(const std::string & why);

  /* Action goals are planned in run() (like the other actions); queued here. */
  struct TaskJob
  {
    std::function<void()> run;
    std::function<void(const std::string &)> abort;
  };
  std::vector<TaskJob> task_jobs_;    // public node thread only
  void processTaskJobs();
  template<class ActionT>
  void runPlannedGoal(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> gh, const std::string & label,
    const std::vector<TaskStep> & steps);
  template<class ActionT>
  void runStoredPlanGoal(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> gh, const std::string & label,
    const std::string & plan_id);

  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_plan_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;
  rclcpp::CallbackGroup::SharedPtr plan_cb_group_;
  rclcpp::Service<arm_api2_msgs::srv::PlanTask>::SharedPtr plan_task_srv_;
  rclcpp_action::Server<arm_api2_msgs::action::ExecuteTask>::SharedPtr execute_task_as_;
  rclcpp_action::Server<arm_api2_msgs::action::ExecutePlan>::SharedPtr execute_plan_as_;
  rclcpp_action::Server<arm_api2_msgs::action::MoveRelative>::SharedPtr move_relative_as_;
  rclcpp_action::Server<arm_api2_msgs::action::MoveToNamed>::SharedPtr move_to_named_as_;
  void init_task_interfaces();
  template<class ActionT>
  typename rclcpp_action::Server<ActionT>::SharedPtr makeTaskActionServer(
    const std::string & name,
    std::function<void(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)> on_accept);
  std::string interfaceName(
    const char * kind, const std::string & key, const std::string & fallback) const;
  void plan_task_cb(
    const std::shared_ptr<arm_api2_msgs::srv::PlanTask::Request> req,
    const std::shared_ptr<arm_api2_msgs::srv::PlanTask::Response> res);

/* ===================== scene, constraints, named poses =====================
 * moveit2_iface_scene.cpp */
  moveit_msgs::msg::Constraints path_constraints_;   // applied to every plan while set
  bool hasPathConstraints() const;
  std::vector<std::string> touchLinksFor(
    const std::string & link, const std::vector<std::string> & requested) const;
  bool applyAttach(
    const std::string & object_id, const std::string & link,
    const std::vector<std::string> & touch_links, std::string & message);
  bool applyDetach(const std::string & object_id, bool remove, std::string & message);

  std::string named_poses_file_;
  arm_api2::logic::NamedPoseMap config_named_poses_;
  arm_api2::logic::NamedPoseMap saved_named_poses_;
  std::mutex named_poses_mutex_;
  void loadNamedPoses();
  /* saved > config > srdf. False with a reason when unknown. */
  bool lookupNamedPose(
    const std::string & name, arm_api2::logic::NamedPose & out,
    std::string & error);

  rclcpp::Service<arm_api2_msgs::srv::SaveNamedPose>::SharedPtr save_named_pose_srv_;
  rclcpp::Service<arm_api2_msgs::srv::ListNamedPoses>::SharedPtr list_named_poses_srv_;
  rclcpp::Service<arm_api2_msgs::srv::NameRequest>::SharedPtr delete_named_pose_srv_;
  rclcpp::Service<arm_api2_msgs::srv::AttachObject>::SharedPtr attach_object_srv_;
  rclcpp::Service<arm_api2_msgs::srv::DetachObject>::SharedPtr detach_object_srv_;
  rclcpp::Service<arm_api2_msgs::srv::SetPathConstraints>::SharedPtr set_path_constraints_srv_;
  rclcpp::Service<arm_api2_msgs::srv::ClearPathConstraints>::SharedPtr clear_path_constraints_srv_;
  rclcpp::Service<arm_api2_msgs::srv::AddCollisionObject>::SharedPtr arm_add_collision_object_srv_;
  rclcpp::Service<arm_api2_msgs::srv::AddCollisionMesh>::SharedPtr add_collision_mesh_srv_;
  rclcpp::Service<arm_api2_msgs::srv::NameRequest>::SharedPtr remove_collision_object_srv_;
  rclcpp::Service<arm_api2_msgs::srv::ListCollisionObjects>::SharedPtr list_collision_objects_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_collision_objects_srv_;
  void init_scene_interfaces();
};

#endif
