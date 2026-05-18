/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Crobotic Solutions d.o.o.
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

/*      Title       : moveit2_simple_iface.cpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : The core robot manipulator and MoveIt2! ROS 2 interfacing header class.
 */

#include "arm_api2/moveit2_simple_iface.hpp"

#include <algorithm>
#include <cctype>
#include <optional>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

m2SimpleIface::m2SimpleIface(const rclcpp::NodeOptions &options)
    : Node("moveit2_simple_iface", options), node_(std::make_shared<rclcpp::Node>("moveit2_simple_iface_node", options)), 
     executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()), gripper_(node_) 
{   
    // USE_SIM_TIME HACK TO TEST SERVO!
    this->set_parameter(rclcpp::Parameter("use_sim_time", false));
    this->get_parameter("config_path", config_path);
    this->get_parameter("enable_servo", enable_servo);
    this->get_parameter("dt", dt); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config!");

    // TODO: Add as reconfigurable param 
    std::chrono::duration<double> SYSTEM_DT(dt);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&m2SimpleIface::run, this));

    // Load arm basically --> two important params
    // Manual param specification --> https://github.com/moveit/moveit2_tutorials/blob/8eaef05bfbabde3f35910ad054a819d79e70d3fc/doc/tutorials/quickstart_in_rviz/launch/demo.launch.py#L105
    config              = init_config(config_path);  
    PLANNING_GROUP      = config["robot"]["arm_name"].as<std::string>(); 
    EE_LINK_NAME        = config["robot"]["ee_link_name"].as<std::string>();
    ROBOT_DESC          = config["robot"]["robot_desc"].as<std::string>();  
    PLANNING_FRAME      = config["robot"]["planning_frame"].as<std::string>(); 
    PLANNING_SCENE      = config["robot"]["planning_scene"].as<std::string>(); 
    MOVE_GROUP_NS       = config["robot"]["move_group_ns"].as<std::string>(); 
    NUM_CART_PTS        = config["robot"]["num_cart_pts"].as<int>(); 
    JOINT_STATES        = config["robot"]["joint_states"].as<std::string>(); 
    SERVO_TRAJECTORY_TOPIC = "scaled_joint_trajectory_controller/joint_trajectory";
    if (config["robot"]["servo_trajectory_topic"]) {
        SERVO_TRAJECTORY_TOPIC = config["robot"]["servo_trajectory_topic"].as<std::string>();
    }
    max_vel_scaling_factor = config["robot"]["max_vel_scaling_factor"].as<float>();
    max_acc_scaling_factor = config["robot"]["max_acc_scaling_factor"].as<float>();

    /* Optional override for OMPL planning time. Default 1.5 s — see hpp comment. */
    if (config["robot"]["pose_plan_time_sec"]) {
        const double v = config["robot"]["pose_plan_time_sec"].as<double>();
        if (v > 0.0) pose_plan_time_sec_ = v;
    }

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

    // Currently not used :) [ns]
    ns_ = this->get_namespace();
    if (MOVE_GROUP_NS.empty() || MOVE_GROUP_NS == "null")
    {
        MOVE_GROUP_NS = (ns_ == "/") ? "" : ns_;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

    init_publishers(); 
    init_subscribers(); 
    init_services(); 
    init_moveit(); 
    if (enable_servo) {servoPtr = init_servo();}; 

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node!"); 

    // Init anything for the old pose because it is non-existent at the beggining
    m_oldPoseCmd.pose.position.x = 5.0; 
    nodeInit = true; 

}

YAML::Node m2SimpleIface::init_config(std::string yaml_path)
{   
    RCLCPP_INFO_STREAM(this->get_logger(), "Config yaml path is: " << yaml_path); 
    return YAML::LoadFile(yaml_path);
}

void m2SimpleIface::init_publishers()
{   
    auto pose_state_name = config["topic"]["pub"]["current_pose"]["name"].as<std::string>(); 
    pose_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_state_name, 1); 
    auto current_robot_state_name = config["topic"]["pub"]["current_robot_state"]["name"].as<std::string>(); 
    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>(current_robot_state_name, 1);
    auto gripper_state_name = config["topic"]["pub"]["gripper_state"]["name"].as<std::string>();
    gripper_state_pub_ = this->create_publisher<std_msgs::msg::String>(gripper_state_name, 1);
    servo_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        SERVO_TRAJECTORY_TOPIC,
        10);
    servo_forward_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "forward_position_controller/commands",
        10);
    servo_status_pub_ = this->create_publisher<moveit_msgs::msg::ServoStatus>(
        "moveit2_iface/status",
        10);

    /* Plan-status: latched so a late-joining GUI client sees the most recent
     * attempt without waiting for the next click. */
    std::string plan_status_name = "arm/state/plan_status";
    if (config["topic"]["pub"]["plan_status"] && config["topic"]["pub"]["plan_status"]["name"]) {
        plan_status_name = config["topic"]["pub"]["plan_status"]["name"].as<std::string>();
    }
    rclcpp::QoS plan_status_qos(rclcpp::KeepLast(1));
    plan_status_qos.transient_local();
    plan_status_pub_ = this->create_publisher<arm_api2_msgs::msg::PlanStatus>(plan_status_name, plan_status_qos);
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized publishers!");
}

void m2SimpleIface::init_subscribers()
{
    auto pose_cmd_name = config["topic"]["sub"]["cmd_pose"]["name"].as<std::string>(); 
    auto cart_traj_cmd_name = config["topic"]["sub"]["cmd_traj"]["name"].as<std::string>(); 
    auto joint_states_name = config["topic"]["sub"]["joint_states"]["name"].as<std::string>();
    pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_cmd_name, 1, std::bind(&m2SimpleIface::pose_cmd_cb, this, _1));
    ctraj_cmd_sub_ = this->create_subscription<arm_api2_msgs::msg::CartesianWaypoints>(cart_traj_cmd_name, 1, std::bind(&m2SimpleIface::cart_poses_cb, this, _1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_states_name, 1, std::bind(&m2SimpleIface::joint_state_cb, this, _1));
    servo_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "moveit2_iface/servo_twist_cmd",
        10,
        std::bind(&m2SimpleIface::servo_twist_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!"); 
}

void m2SimpleIface::init_services()
{
    auto change_state_name = config["srv"]["change_robot_state"]["name"].as<std::string>(); 
    auto set_vel_acc_name  = config["srv"]["set_vel_acc"]["name"].as<std::string>();
    auto set_planner_name  = config["srv"]["set_planner"]["name"].as<std::string>();
    auto open_gripper_name = config["srv"]["open_gripper"]["name"].as<std::string>(); 
    auto close_gripper_name= config["srv"]["close_gripper"]["name"].as<std::string>();
    change_state_srv_ = this->create_service<arm_api2_msgs::srv::ChangeState>(change_state_name, std::bind(&m2SimpleIface::change_state_cb, this, _1, _2)); 
    set_vel_acc_srv_  = this->create_service<arm_api2_msgs::srv::SetVelAcc>(set_vel_acc_name, std::bind(&m2SimpleIface::set_vel_acc_cb, this, _1, _2));
    set_planner_srv_  = this->create_service<arm_api2_msgs::srv::SetStringParam>(set_planner_name, std::bind(&m2SimpleIface::set_planner_cb, this, _1, _2));
    open_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(open_gripper_name, std::bind(&m2SimpleIface::open_gripper_cb, this, _1, _2));
    close_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(close_gripper_name, std::bind(&m2SimpleIface::close_gripper_cb, this, _1, _2));
    add_collision_object_srv_ = this->create_service<arm_api2_msgs::srv::AddCollisionObject>("add_collision_object", std::bind(&m2SimpleIface::add_collision_object_cb, this, _1, _2));

    std::string check_reachability_name = "arm/check_reachability";
    if (config["srv"]["check_reachability"] && config["srv"]["check_reachability"]["name"]) {
        check_reachability_name = config["srv"]["check_reachability"]["name"].as<std::string>();
    }
    check_reachability_srv_ = this->create_service<arm_api2_msgs::srv::CheckReachability>(
        check_reachability_name,
        std::bind(&m2SimpleIface::check_reachability_cb, this, _1, _2));
    configure_controller_client_ =
        node_->create_client<controller_manager_msgs::srv::ConfigureController>(
            "controller_manager/configure_controller");
    load_controller_client_ =
        node_->create_client<controller_manager_msgs::srv::LoadController>(
            "controller_manager/load_controller");
    switch_controller_client_ =
        node_->create_client<controller_manager_msgs::srv::SwitchController>(
            "controller_manager/switch_controller");
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized services!"); 
}

void m2SimpleIface::init_moveit()
{

    RCLCPP_INFO_STREAM(this->get_logger(), "robot_description: " << ROBOT_DESC); 
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_group: " << PLANNING_GROUP);
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_frame: " << PLANNING_FRAME); 
    RCLCPP_INFO_STREAM(this->get_logger(), "move_group_ns: " << MOVE_GROUP_NS);  
    // MoveIt related things!
    moveGroupInit       = setMoveGroup(node_, PLANNING_GROUP, MOVE_GROUP_NS); 
    pSceneMonitorInit   = setPlanningSceneMonitor(node_, ROBOT_DESC);
    robotModelInit      = setRobotModel(node_);
}

// TODO: Try to replace with auto
// TODO: Try to replace with auto
std::unique_ptr<moveit_servo::Servo> m2SimpleIface::init_servo()
{   
    // New Jazzy API - use ParamListener instead of ServoParameters
    servo_param_listener_ = std::make_shared<servo::ParamListener>(node_, "moveit_servo");
    auto servo_params = servo_param_listener_->get_params();
    RCLCPP_INFO_STREAM(this->get_logger(), "Servo move_group_name: " << servo_params.move_group_name);  

    auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_param_listener_, m_pSceneMonitorPtr); 
    RCLCPP_INFO(this->get_logger(), "Servo initialized!"); 
    return servo;
}

void m2SimpleIface::pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    geometry_msgs::msg::PoseStamped cmd_plan;
    const std::string source_frame = msg->header.frame_id;

    if (source_frame.empty() || source_frame == PLANNING_FRAME) {
        cmd_plan = *msg;
        cmd_plan.header.frame_id = PLANNING_FRAME;
    } else {
        try {
            cmd_plan = tf_buffer_->transform(*msg, PLANNING_FRAME);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(
                this->get_logger(),
                "pose_cmd_cb: cannot transform '%s' -> '%s': %s — ignoring pose command",
                source_frame.c_str(), PLANNING_FRAME.c_str(), ex.what());
            return;
        }
    }

    std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
    m_currPoseCmd = cmd_plan;
    m_currPoseCmd.header.frame_id = PLANNING_FRAME;
    if (!utils::comparePose(m_currPoseCmd, m_oldPoseCmd)) {
        recivCmd = true;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "recivCmd: " << recivCmd);
}

void m2SimpleIface::cart_poses_cb(const arm_api2_msgs::msg::CartesianWaypoints::SharedPtr msg)
{
    // TODO: Maybe implement same check for as the pose_cmd
    m_cartesianWaypoints = msg->poses; 
    recivTraj = true; 
}

void m2SimpleIface::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
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

    /* Piper driver uses `gripper`; MoveIt URDF uses parallel prismatic joint7 + joint8 (see piper_read_slave_joint). */
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

void m2SimpleIface::servo_twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    latest_twist_cmd_ = *msg;
    new_twist_cmd_ = true;
}

void m2SimpleIface::processServoCommand()
{
    if (!servoPtr || !new_twist_cmd_ || !m_moveGroupPtr) return;

    const auto time_since_servo_entered = (this->now() - servo_entered_time_).seconds();
    if (time_since_servo_entered < 0.5) {
        new_twist_cmd_ = false;
        return;
    }

    new_twist_cmd_ = false;

    const bool all_zero = (std::abs(latest_twist_cmd_.twist.linear.x) < 1e-6 &&
                           std::abs(latest_twist_cmd_.twist.linear.y) < 1e-6 &&
                           std::abs(latest_twist_cmd_.twist.linear.z) < 1e-6 &&
                           std::abs(latest_twist_cmd_.twist.angular.x) < 1e-6 &&
                           std::abs(latest_twist_cmd_.twist.angular.y) < 1e-6 &&
                           std::abs(latest_twist_cmd_.twist.angular.z) < 1e-6);
    if (all_zero) {
        return;
    }

    try {
        moveit_servo::TwistCommand twist_cmd;
        twist_cmd.frame_id = latest_twist_cmd_.header.frame_id.empty()
            ? PLANNING_FRAME
            : latest_twist_cmd_.header.frame_id;
        twist_cmd.velocities[0] = latest_twist_cmd_.twist.linear.x;
        twist_cmd.velocities[1] = latest_twist_cmd_.twist.linear.y;
        twist_cmd.velocities[2] = latest_twist_cmd_.twist.linear.z;
        twist_cmd.velocities[3] = latest_twist_cmd_.twist.angular.x;
        twist_cmd.velocities[4] = latest_twist_cmd_.twist.angular.y;
        twist_cmd.velocities[5] = latest_twist_cmd_.twist.angular.z;

        auto current_state = m_moveGroupPtr->getCurrentState(1.0);
        if (!current_state) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Could not get current robot state for servo command");
            return;
        }

        servoPtr->setCommandType(moveit_servo::CommandType::TWIST);
        moveit_servo::KinematicState next_state =
            servoPtr->getNextJointState(current_state, twist_cmd);

        const auto status = servoPtr->getStatus();
        const auto status_msg_str = servoPtr->getStatusMessage();

        moveit_msgs::msg::ServoStatus status_msg;
        status_msg.code = static_cast<int8_t>(status);
        status_msg.message = status_msg_str;
        servo_status_pub_->publish(status_msg);

        if (status == moveit_servo::StatusCode::INVALID ||
            status == moveit_servo::StatusCode::HALT_FOR_SINGULARITY ||
            status == moveit_servo::StatusCode::HALT_FOR_COLLISION) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Servo status: %s",
                status_msg_str.c_str());
            return;
        }

        if (next_state.joint_names.empty() || next_state.positions.size() == 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Servo returned empty state");
            return;
        }

        std_msgs::msg::Float64MultiArray command;
        for (size_t i = 0; i < next_state.positions.size(); ++i) {
            command.data.push_back(next_state.positions[i]);
        }
        servo_forward_position_pub_->publish(command);
        last_servo_state_ = next_state;
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Servo command processing failed: %s",
            e.what());
    }
}

void m2SimpleIface::open_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    const bool success = sendGripperCmd(0.0);
    if (!success) {
        res->success = false;
        res->message = "failed";
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Gripper opened.");
    std_msgs::msg::String state_msg;
    state_msg.data = "open";
    gripper_state_pub_->publish(state_msg);
    res->success = true;
    res->message = "ok";
}

void m2SimpleIface::close_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                     const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    const bool success = sendGripperCmd(0.8);
    if (!success) {
        res->success = false;
        res->message = "failed";
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Gripper closed.");
    std_msgs::msg::String state_msg;
    state_msg.data = "closed";
    gripper_state_pub_->publish(state_msg);
    res->success = true;
    res->message = "ok";
}

bool m2SimpleIface::sendGripperCmd(double normalized_position_robotiq, double max_effort)
{
  if (piper_joint_gripper_) {
    return piper_joint_gripper_->send_gripper_command(normalized_position_robotiq, max_effort);
  }
  return gripper_.send_gripper_command(normalized_position_robotiq, max_effort);
}

float m2SimpleIface::gripperMeasuredPositionNormalized()
{
  if (piper_joint_gripper_) {
    return piper_joint_gripper_->get_position();
  }
  return gripper_.get_position();
}

float m2SimpleIface::gripperMeasuredEffort()
{
  if (piper_joint_gripper_) {
    return piper_joint_gripper_->get_effort();
  }
  return gripper_.get_effort();
}

bool m2SimpleIface::gripperMeasuredStalled()
{
  if (piper_joint_gripper_) {
    return piper_joint_gripper_->is_stalled();
  }
  return gripper_.is_stalled();
}

bool m2SimpleIface::gripperMeasuredReachedGoal()
{
  if (piper_joint_gripper_) {
    return piper_joint_gripper_->reached_goal();
  }
  return gripper_.reached_goal();
}

void m2SimpleIface::set_vel_acc_cb(const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Request> req, 
                                   const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Response> res)
{
    if(req->max_vel < 0 || req->max_acc < 0 || req->max_vel > 1 || req->max_acc > 1)
    {
        res->success = false;
        RCLCPP_ERROR_STREAM(this->get_logger(), "Velocity and acceleration must be in the range [0, 1]!");
        return;
    }
    max_vel_scaling_factor = float(req->max_vel);
    max_acc_scaling_factor = float(req->max_acc);
    res->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set velocity and acceleration to " << max_vel_scaling_factor << " " << max_acc_scaling_factor);
}

void m2SimpleIface::set_planner_cb(const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req,
                                   const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res)
{
    std::string planner_string = req->value;
    
    // Parse the planner string (format: "planner_id_type", e.g., "pilz_LIN", "ompl_RRT")
    size_t underscore_pos = planner_string.find('_');
    if (underscore_pos == std::string::npos)
    {
        res->success = false;
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid planner format. Expected 'planner_type' (e.g., 'pilz_LIN', 'ompl_RRT')");
        return;
    }
    
    std::string planner_prefix = planner_string.substr(0, underscore_pos);
    std::string planner_type = planner_string.substr(underscore_pos + 1);
    
    // Map short names to full planner IDs
    std::string planner_id;
    if (planner_prefix == "pilz")
    {
        planner_id = "pilz_industrial_motion_planner";
    }
    else if (planner_prefix == "ompl")
    {
        planner_id = "ompl";
    }
    else
    {
        res->success = false;
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown planner: " << planner_prefix << ". Supported: 'pilz', 'ompl'");
        return;
    }
    
    // Set the planner
    try
    {
        m_moveGroupPtr->setPlanningPipelineId(planner_id);
        m_moveGroupPtr->setPlannerId(planner_type);
        
        current_planner_id_ = planner_id;
        current_planner_type_ = planner_type;
        
        res->success = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "Successfully set planner to: " << planner_id << " / " << planner_type);
    }
    catch (const std::exception& e)
    {
        res->success = false;
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set planner: " << e.what());
    }
}

void m2SimpleIface::add_collision_object_cb(const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Request> req,
                                            const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Adding collision object to planning scene");
    
    // Create a collision object message
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = PLANNING_FRAME;
    collision_object.id = req->id;
    
    // Define primitive based on type
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = req->primitive_type;
    
    // Set dimensions based on type
    if (req->primitive_type == shape_msgs::msg::SolidPrimitive::BOX) {
        if (req->dimensions.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "BOX requires 3 dimensions [x, y, z]");
            res->success = false;
            res->message = "BOX requires 3 dimensions [x, y, z]";
            return;
        }
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = req->dimensions[0];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = req->dimensions[1];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = req->dimensions[2];
    }
    else if (req->primitive_type == shape_msgs::msg::SolidPrimitive::SPHERE) {
        if (req->dimensions.size() != 1) {
            RCLCPP_ERROR(this->get_logger(), "SPHERE requires 1 dimension [radius]");
            res->success = false;
            res->message = "SPHERE requires 1 dimension [radius]";
            return;
        }
        primitive.dimensions.resize(1);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = req->dimensions[0];
    }
    else if (req->primitive_type == shape_msgs::msg::SolidPrimitive::CYLINDER) {
        if (req->dimensions.size() != 2) {
            RCLCPP_ERROR(this->get_logger(), "CYLINDER requires 2 dimensions [height, radius]");
            res->success = false;
            res->message = "CYLINDER requires 2 dimensions [height, radius]";
            return;
        }
        primitive.dimensions.resize(2);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = req->dimensions[0];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = req->dimensions[1];
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported primitive type: %d", req->primitive_type);
        res->success = false;
        res->message = "Unsupported primitive type";
        return;
    }
    
    // Define pose of the object
    geometry_msgs::msg::Pose object_pose;
    object_pose.position = req->position;
    object_pose.orientation = req->orientation;
    
    // If orientation is zero (default), set to identity
    if (object_pose.orientation.w == 0.0 && 
        object_pose.orientation.x == 0.0 && 
        object_pose.orientation.y == 0.0 && 
        object_pose.orientation.z == 0.0) {
        object_pose.orientation.w = 1.0;
    }
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    
    // Add the collision object to the scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    
    m_planningSceneInterface.addCollisionObjects(collision_objects);
    
    RCLCPP_INFO(this->get_logger(), "Added collision object '%s' to planning scene", req->id.c_str());
    res->success = true;
    res->message = "Collision object added successfully";
}

void m2SimpleIface::check_reachability_cb(
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
        } catch (const tf2::TransformException &ex) {
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

    const unsigned int attempts = (req->ik_attempts == 0) ? 4u : static_cast<unsigned int>(req->ik_attempts);
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
        if (!ik_ok) continue;

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

void m2SimpleIface::publishPlanStatus(bool success,
                                      const std::string &error_code,
                                      const std::string &reason,
                                      const geometry_msgs::msg::PoseStamped &requested_pose,
                                      double plan_time_sec,
                                      const std::string &mode)
{
    if (!plan_status_pub_) return;
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

void m2SimpleIface::change_state_cb(const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req, 
                                    const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res)
{
    auto itr = std::find(std::begin(stateNames), std::end(stateNames), req->state); 
    
    if ( itr != std::end(stateNames))
    {
        int wantedIndex_ = std::distance(stateNames, itr); 
        robotState  = (state)wantedIndex_; 
        RCLCPP_INFO_STREAM(this->get_logger(), "Switching state!");
        res->success = true;  
    }else{
        RCLCPP_INFO_STREAM(this->get_logger(), "Failed switching to state " << req->state); 
        res->success = false; 
    } 
}

bool m2SimpleIface::setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs)
{
    // check if moveNs is empty
    if (moveNs == "null") moveNs=""; 

    //https://github.com/moveit/moveit2/issues/496
    m_moveGroupPtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(nodePtr, 
        moveit::planning_interface::MoveGroupInterface::Options(
            groupName,
            "robot_description",
            moveNs));

    double POS_TOL = 0.0000001; 
    // set move group stuff
    m_moveGroupPtr->setEndEffectorLink(EE_LINK_NAME); 
    m_moveGroupPtr->setPoseReferenceFrame(PLANNING_FRAME); 
    m_moveGroupPtr->setGoalPositionTolerance(POS_TOL);
    m_moveGroupPtr->startStateMonitor(); 
    // executor
    executor_->add_node(node_); 
    executor_thread_ = std::thread([this]() {executor_->spin();});
    RCLCPP_INFO_STREAM(this->get_logger(), "Move group interface set up!"); 
    return true; 
}

/* This is not neccessary*/
bool m2SimpleIface::setRobotModel(rclcpp::Node::SharedPtr nodePtr)
{
    robot_model_loader::RobotModelLoader robot_model_loader(nodePtr);
    kinematic_model = robot_model_loader.getModel(); 
    // Find nicer way to do this
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    m_robotStatePtr = kinematic_state;
    m_robotStatePtr->setToDefaultValues();
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot model loaded!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot model frame is: " << kinematic_model->getModelFrame().c_str());
    return true;
}

// TODO: Add service to add collision objects to the scene 
bool m2SimpleIface::setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name)
{
    // https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
    // https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/planning_scene/src/planning_scene_tutorial.cpp
    m_pSceneMonitorPtr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(nodePtr, name); 
    m_pSceneMonitorPtr->startSceneMonitor(PLANNING_SCENE); 
    if (m_pSceneMonitorPtr->getPlanningScene())
    {
        m_pSceneMonitorPtr->startStateMonitor(JOINT_STATES); 
        m_pSceneMonitorPtr->setPlanningScenePublishingFrequency(25);
        m_pSceneMonitorPtr->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
        m_pSceneMonitorPtr->startSceneMonitor(); 
        m_pSceneMonitorPtr->providePlanningSceneService(); 
    }
    else 
    {
        RCLCPP_ERROR(this->get_logger(), "Planning scene not configured!"); 
        return EXIT_FAILURE; 
    }
    
    //TODO: Check what's difference between planning_Scene and planning_scene_monitor
    RCLCPP_INFO_STREAM(this->get_logger(), "Created planning scene monitor!");
    return true; 
}

void m2SimpleIface::stopBeforeAsyncExecute()
{
    if (!m_moveGroupPtr)
    {
        return;
    }
    m_moveGroupPtr->stop();
    /* Allow the trajectory execution action to settle; short sleeps alone were not always enough. */
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
}

bool m2SimpleIface::execMove(bool async_flag)
{
    geometry_msgs::msg::PoseStamped pose_snap;
    {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        pose_snap = m_currPoseCmd;
    }

    if (trajectory_executing_.load(std::memory_order_acquire)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory still executing; ignoring new pose command until it finishes.");
        publishPlanStatus(false, "BUSY_TRAJECTORY_EXECUTING",
                          "Trajectory still executing; ignored new pose command",
                          pose_snap, 0.0, "JOINT_TRAJ_CTL");
        return false;
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
    RCLCPP_INFO_STREAM(this->get_logger(), "poseTarget is: " << cmdPose_.pose.position.x << " "
                                                              << cmdPose_.pose.position.y << " "
                                                              << cmdPose_.pose.position.z);
    const auto plan_t0 = std::chrono::steady_clock::now();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_err = m_moveGroupPtr->plan(plan);
    const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - plan_t0).count();

    if (plan_err != moveit::core::MoveItErrorCode::SUCCESS) {
        const std::string code = utils::moveItErrorCodeToString(plan_err);
        RCLCPP_ERROR_STREAM(this->get_logger(),
            "Planning failed (" << code << ") in " << plan_dt << " s; not executing.");
        publishPlanStatus(false, code,
                          "MoveIt planner did not return SUCCESS for this pose",
                          cmdPose_, plan_dt, "JOINT_TRAJ_CTL");
        return false;
    }

    const bool ok = execPlan_with_plan(plan, async_flag);
    if (ok) {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        m_oldPoseCmd = cmdPose_;
        RCLCPP_INFO_STREAM(this->get_logger(), "Plan succeeded; executing commanded path.");
        publishPlanStatus(true, "SUCCESS", "Plan accepted; executing", cmdPose_, plan_dt, "JOINT_TRAJ_CTL");
    } else {
        publishPlanStatus(false, "EXECUTE_FAILED",
                          "Plan succeeded but execute() did not start",
                          cmdPose_, plan_dt, "JOINT_TRAJ_CTL");
    }
    return ok;
}

bool m2SimpleIface::execPlan_with_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan &plan,
    bool async_flag)
{
    if (trajectory_executing_.load(std::memory_order_acquire)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory already executing; skipping execute.");
        return false;
    }

    stopBeforeAsyncExecute();

    if (!async_flag) {
        trajectory_executing_.store(true, std::memory_order_release);
        const auto exec_err = m_moveGroupPtr->execute(plan);
        trajectory_executing_.store(false, std::memory_order_release);
        if (exec_err != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "execute(plan) did not report SUCCESS");
        }
        return true;
    }

    trajectory_executing_.store(true, std::memory_order_release);
    try {
        std::thread([this, plan]() {
            try {
                const auto exec_err = m_moveGroupPtr->execute(plan);
                if (exec_err != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_WARN(this->get_logger(), "execute(plan) in async worker did not report SUCCESS");
                }
            } catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "execute(plan) async worker exception: %s", ex.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "execute(plan) async worker unknown exception");
            }
            trajectory_executing_.store(false, std::memory_order_release);
        }).detach();
    } catch (const std::system_error &e) {
        trajectory_executing_.store(false, std::memory_order_release);
        RCLCPP_ERROR(this->get_logger(), "Failed to spawn async execute thread: %s", e.what());
        return false;
    }

    return true;
}

/* Legacy helper kept for callers that still want plan+execute in one shot.
 * Prefer execPlan_with_plan() so the caller can attach planning metadata. */
bool m2SimpleIface::execPlan(bool async_flag)
{
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
    if (pose_plan_time_sec_ > 0.0) {
        m_moveGroupPtr->setPlanningTime(pose_plan_time_sec_);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = (m_moveGroupPtr->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        return false;
    }

    if (trajectory_executing_.load(std::memory_order_acquire)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory already executing after plan(); skipping execute.");
        return false;
    }

    stopBeforeAsyncExecute();

    if (!async_flag) {
        trajectory_executing_.store(true, std::memory_order_release);
        const auto exec_err = m_moveGroupPtr->execute(plan);
        trajectory_executing_.store(false, std::memory_order_release);
        if (exec_err != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "execute(plan) did not report SUCCESS");
        }
        return true;
    }

    /* Async: non-blocking for the timer / callback thread — blocking execute runs on a worker thread.
     * A single-flight guard (trajectory_executing_) prevents overlapping MoveGroup executions (SIGSEGV). */
    trajectory_executing_.store(true, std::memory_order_release);
    try {
        std::thread([this, plan]() {
            try {
                const auto exec_err = m_moveGroupPtr->execute(plan);
                if (exec_err != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_WARN(this->get_logger(), "execute(plan) in async worker did not report SUCCESS");
                }
            } catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "execute(plan) async worker exception: %s", ex.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "execute(plan) async worker unknown exception");
            }
            trajectory_executing_.store(false, std::memory_order_release);
        }).detach();
    } catch (const std::system_error &e) {
        trajectory_executing_.store(false, std::memory_order_release);
        RCLCPP_ERROR(this->get_logger(), "Failed to spawn async execute thread: %s", e.what());
        return false;
    }

    return true;
}

bool m2SimpleIface::planExecCartesian(bool async_flag)
{
    geometry_msgs::msg::PoseStamped pose_snap;
    {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        pose_snap = m_currPoseCmd;
    }

    if (trajectory_executing_.load(std::memory_order_acquire)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory still executing; ignoring Cartesian pose command.");
        publishPlanStatus(false, "BUSY_TRAJECTORY_EXECUTING",
                          "Trajectory still executing; ignored Cartesian pose command",
                          pose_snap, 0.0, "CART_TRAJ_CTL");
        return false;
    }

    stopBeforeAsyncExecute();
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

    const auto plan_t0 = std::chrono::steady_clock::now();
    std::vector<geometry_msgs::msg::Pose> cartesianWaypoints =
        utils::createCartesianWaypoints(m_currPoseState.pose, pose_snap.pose, NUM_CART_PTS);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double jumpThr = 0.0;
    double eefStep = 0.02;
    /* computeCartesianPath returns the fraction of the path the planner managed
     * to follow with valid IK + collision checks. <1.0 means the line trims to
     * a partial path; we surface that to the GUI so the user knows. */
    const double fraction = m_moveGroupPtr->computeCartesianPath(
        cartesianWaypoints, eefStep, jumpThr, trajectory);
    const double plan_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - plan_t0).count();

    if (fraction <= 0.0 || trajectory.joint_trajectory.points.empty()) {
        publishPlanStatus(false, "PLAN_NOT_FOUND",
                          "Cartesian path could not be computed (fraction=" + std::to_string(fraction) + ")",
                          pose_snap, plan_dt, "CART_TRAJ_CTL");
        return false;
    }

    const bool ok = execTrajectory(std::move(trajectory), async_flag);
    if (ok) {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        m_oldPoseCmd = pose_snap;
        const std::string reason = (fraction >= 0.999)
            ? "Cartesian plan executing (full path)"
            : "Cartesian plan executing (partial fraction=" + std::to_string(fraction) + ")";
        publishPlanStatus(true, "SUCCESS", reason, pose_snap, plan_dt, "CART_TRAJ_CTL");
    } else {
        publishPlanStatus(false, "EXECUTE_FAILED",
                          "Cartesian plan ready but execute() did not start",
                          pose_snap, plan_dt, "CART_TRAJ_CTL");
    }
    return ok;
}

bool m2SimpleIface::execCartesian(bool async_flag)
{
    if (trajectory_executing_.load(std::memory_order_acquire)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Trajectory still executing; ignoring Cartesian waypoint trajectory.");
        return false;
    }

    stopBeforeAsyncExecute();
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double jumpThr = 0.0;
    double eefStep = 0.02;
    m_moveGroupPtr->computeCartesianPath(m_cartesianWaypoints, eefStep, jumpThr, trajectory);

    const bool ok = execTrajectory(std::move(trajectory), async_flag);
    if (ok) {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        m_oldPoseCmd = m_currPoseCmd;
    }
    return ok;
}

bool m2SimpleIface::execTrajectory(moveit_msgs::msg::RobotTrajectory trajectory, bool async_flag)
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

    /* Callers (planExecCartesian, execCartesian) already invoked stopBeforeAsyncExecute(). */

    if (!async_flag) {
        trajectory_executing_.store(true, std::memory_order_release);
        const auto exec_err = m_moveGroupPtr->execute(trajectory);
        trajectory_executing_.store(false, std::memory_order_release);
        if (exec_err != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "execute(trajectory) did not report SUCCESS");
        }
        return true;
    }

    trajectory_executing_.store(true, std::memory_order_release);
    try {
        std::thread([this, trajectory]() {
            try {
                const auto exec_err = m_moveGroupPtr->execute(trajectory);
                if (exec_err != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_WARN(this->get_logger(), "execute(trajectory) in async worker did not report SUCCESS");
                }
            } catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "execute(trajectory) async worker exception: %s", ex.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "execute(trajectory) async worker unknown exception");
            }
            trajectory_executing_.store(false, std::memory_order_release);
        }).detach();
    } catch (const std::system_error &e) {
        trajectory_executing_.store(false, std::memory_order_release);
        RCLCPP_ERROR(this->get_logger(), "Failed to spawn async execTrajectory thread: %s", e.what());
        return false;
    }

    return true;
}

void m2SimpleIface::getArmState() 
{   
    if (!m_robotStatePtr) {
        RCLCPP_WARN(this->get_logger(), "Robot state pointer is null!");
        return;
    }
    
    const moveit::core::JointModelGroup* joint_model_group = m_robotStatePtr->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> joint_values;
    m_robotStatePtr->copyJointGroupPositions(joint_model_group, joint_values);
    
    // Get current state with timeout to prevent blocking
    m_robotStatePtr = m_moveGroupPtr->getCurrentState(0.1); // 100ms timeout
    if (!m_robotStatePtr) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to get current state");
        return;
    }
    
    // by default timeout is 10 secs
    m_robotStatePtr->update();
    
    Eigen::Isometry3d currentPose_ = m_robotStatePtr->getFrameTransform(EE_LINK_NAME);
    m_currPoseState = utils::convertIsometryToMsg(currentPose_);
    m_currPoseState.header.stamp = this->now();
    m_currPoseState.header.frame_id = PLANNING_FRAME;
}

bool m2SimpleIface::loadController(const std::string& controller_name)
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

bool m2SimpleIface::configureController(const std::string& controller_name)
{
    if (!configure_controller_client_) {
        RCLCPP_ERROR(this->get_logger(), "configure_controller client is not initialized");
        return false;
    }
    if (!configure_controller_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "controller_manager/configure_controller service not available");
        return false;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
    request->name = controller_name;
    auto future = configure_controller_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "configure_controller('%s') timed out", controller_name.c_str());
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

bool m2SimpleIface::switchControllers(
    const std::vector<std::string>& activate,
    const std::vector<std::string>& deactivate)
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

bool m2SimpleIface::enterServoControllerMode()
{
    if (forward_position_controller_active_) {
        return true;
    }
    (void)loadController("forward_position_controller");
    (void)configureController("forward_position_controller");
    const bool ok = switchControllers(
        {"forward_position_controller"},
        {"scaled_joint_trajectory_controller"});
    if (ok) {
        forward_position_controller_active_ = true;
        RCLCPP_INFO(this->get_logger(), "forward_position_controller active for SERVO_CTL");
    }
    return ok;
}

bool m2SimpleIface::leaveServoControllerMode()
{
    if (!forward_position_controller_active_) {
        return true;
    }
    (void)loadController("scaled_joint_trajectory_controller");
    (void)configureController("scaled_joint_trajectory_controller");
    const bool ok = switchControllers(
        {"scaled_joint_trajectory_controller"},
        {"forward_position_controller"});
    if (ok) {
        forward_position_controller_active_ = false;
        RCLCPP_INFO(this->get_logger(), "scaled_joint_trajectory_controller restored");
    }
    return ok;
}

bool m2SimpleIface::run()
{
    if(!nodeInit)       {RCLCPP_ERROR(this->get_logger(), "Node not fully initialized!"); return false;} 
    if(!moveGroupInit)  {RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized!"); return false;} 

    getArmState(); 
    pose_state_pub_->publish(m_currPoseState);
    robot_state_pub_->publish(utils::stateToMsg(robotState));

    rclcpp::Clock steady_clock; 
    int LOG_STATE_TIMEOUT=10000; 

    // STATE MACHINE
    if (robotState == IDLE)
    {   
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT, "arm_api2 is in IDLE mode."); 
    }
    else{
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT, "arm_api2 is in " << stateNames[robotState] << " mode."); 
    }

    // Check if servo active, to deactivate before sending to another pose 
    if (robotState != SERVO_CTL && servoEntered) {
        servoPtr->setCollisionChecking(false);
        servoEntered = false;
        new_twist_cmd_ = false;
        (void)leaveServoControllerMode();
    }

    /* Single-shot semantics: a pose command is consumed exactly once. If planning
     * fails, the failure is surfaced via /arm/state/plan_status; we do NOT silently
     * retry the same goal on every tick (that turns one impossible click into
     * a 5 s × ∞ planner spin and floods the logs). */
    if (robotState == JOINT_TRAJ_CTL)
    {
        if (recivCmd) {
            recivCmd = false;
            (void)execMove(async);
        }
    }

    if (robotState == CART_TRAJ_CTL)
    {
        if (recivCmd) {
            recivCmd = false;
            (void)planExecCartesian(async);
        }

        if (recivTraj) {
            recivTraj = false;
            (void)execCartesian(async);
        }
    }

    if (robotState == SERVO_CTL)
    {   
        if (!servoEntered)
        {   
            latest_twist_cmd_ = geometry_msgs::msg::TwistStamped();
            latest_twist_cmd_.header.frame_id = PLANNING_FRAME;
            latest_twist_cmd_.twist.linear.x = 0.0;
            latest_twist_cmd_.twist.linear.y = 0.0;
            latest_twist_cmd_.twist.linear.z = 0.0;
            latest_twist_cmd_.twist.angular.x = 0.0;
            latest_twist_cmd_.twist.angular.y = 0.0;
            latest_twist_cmd_.twist.angular.z = 0.0;
            new_twist_cmd_ = false;
            // The GUI sim can report near-collision continuously because the planning
            // scene includes fixtures/gripper geometry close to the robot. Let the
            // trajectory controller enforce execution while Servo handles kinematics.
            servoPtr->setCollisionChecking(false);
            if (!enterServoControllerMode()) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Cannot enter SERVO_CTL: failed to activate forward_position_controller");
                return false;
            }
            servo_entered_time_ = this->now();
            servoEntered = true; 
            RCLCPP_INFO(this->get_logger(), "Servo mode activated! Send twist commands to moveit2_iface/servo_twist_cmd");
        }
        processServoCommand();
    }

    return true;     
}



