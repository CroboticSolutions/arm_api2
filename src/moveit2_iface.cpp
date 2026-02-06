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

/*      Title       : moveit2_iface.cpp
 *      Project     : arm_api2
 *      Created     : 06/08/2025
 *      Author      : Filip Zoric
 *      Contributors: Edgar Welte
 *      Description : The core robot manipulator and MoveIt2! ROS 2 interfacing header class.
 */

#include "arm_api2/moveit2_iface.hpp"
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <atomic>
#include <chrono>
#include <future>
#include <regex>
#include <stdexcept>
#include <std_msgs/msg/string.hpp>
#include <vector>

rclcpp::Node::SharedPtr m2Iface::createMoveitNode(rclcpp::Node* parent)
{
    std::string config_path;
    parent->get_parameter("config_path", config_path);
    const YAML::Node config = YAML::LoadFile(config_path);
    std::string MOVE_GROUP_NS = config["robot"]["move_group_ns"].as<std::string>();
    std::string joint_states_topic;
    if (config["robot"]["joint_states"]) {
        joint_states_topic = config["robot"]["joint_states"].as<std::string>();
    } else {
        joint_states_topic = (MOVE_GROUP_NS.empty() || MOVE_GROUP_NS == "null")
            ? "joint_states"
            : MOVE_GROUP_NS + "/joint_states";
    }

    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    std::vector<std::string> args = {"--ros-args", "-r", "__ns:=/"};
    if (!joint_states_topic.empty() && joint_states_topic != "joint_states") {
        args.push_back("-r");
        args.push_back("joint_states:=" + joint_states_topic);
    }
    opts.arguments(args);
    return std::make_shared<rclcpp::Node>("moveit2_iface_node", opts);
}

m2Iface::m2Iface(const rclcpp::NodeOptions &options)
    : Node("moveit2_iface", options),
     node_(createMoveitNode(this)),
     executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()), gripper(node_)
{
    // NOTE: use_sim_time is now passed via launch parameters, not hardcoded
    // this->set_parameter(rclcpp::Parameter("use_sim_time", false));

    // Get use_sim_time parameter and apply it to the internal node
    bool use_sim_time = false;
    this->get_parameter("use_sim_time", use_sim_time);
    node_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

    this->get_parameter("config_path", config_path);
    this->get_parameter("enable_servo", enable_servo);
    this->get_parameter("dt", dt);

    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config!");

    // TODO: Add as reconfigurable param 
    std::chrono::duration<double> SYSTEM_DT(dt);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&m2Iface::run, this));

    // Load arm basically --> two important params
    // Manual param specification --> https://github.com/moveit/moveit2_tutorials/blob/8eaef05bfbabde3f35910ad054a819d79e70d3fc/doc/tutorials/quickstart_in_rviz/launch/demo.launch.py#L105
    // TODO: Add as reconfigurable ROS 2 params
    config              = init_config(config_path);  
    PLANNING_GROUP      = config["robot"]["arm_name"].as<std::string>(); 
    EE_LINK_NAME        = config["robot"]["ee_link_name"].as<std::string>();
    ROBOT_DESC          = config["robot"]["robot_desc"].as<std::string>();  
    PLANNING_FRAME      = config["robot"]["planning_frame"].as<std::string>(); 
    PLANNING_SCENE      = config["robot"]["planning_scene"].as<std::string>(); 
    MOVE_GROUP_NS       = config["robot"]["move_group_ns"].as<std::string>();
    if (config["robot"]["joint_states"]) {
        JOINT_STATES = config["robot"]["joint_states"].as<std::string>();
    } else {
        JOINT_STATES = (MOVE_GROUP_NS.empty() || MOVE_GROUP_NS == "null")
            ? "joint_states"
            : MOVE_GROUP_NS + "/joint_states";
    }
    WITH_PLANNER        = config["robot"]["with_planner"].as<bool>();
    INIT_VEL_SCALING    = 0.05; 
    INIT_ACC_SCALING    = 0.05; 
    eager_execution     = true; 
    max_vel_scaling_factor = config["robot"]["max_vel_scaling_factor"].as<float>();
    max_acc_scaling_factor = config["robot"]["max_acc_scaling_factor"].as<float>();
    
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Calling init_publishers...");
    init_publishers();
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Calling init_subscribers...");
    init_subscribers();
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Calling init_services...");
    init_services();
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Calling init_moveit...");
    init_moveit();
    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_moveit completed. Calling init_actionservers...");
    init_actionservers();
    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_actionservers completed.");

    if (enable_servo) {
        RCLCPP_INFO(this->get_logger(), "[DEBUG] enable_servo=true, calling init_servo...");
        try {
            servoPtr = init_servo();
            RCLCPP_INFO(this->get_logger(), "[DEBUG] init_servo completed successfully.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[DEBUG] init_servo EXCEPTION: %s", e.what());
            throw;
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "[DEBUG] enable_servo=false, skipping init_servo.");
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node!"); 

    // Init anything for the old pose because it is non-existent at the beggining
    nodeInit = true; 
}

YAML::Node m2Iface::init_config(std::string yaml_path)
{   
    RCLCPP_INFO_STREAM(this->get_logger(), "Config yaml path is: " << yaml_path); 
    return YAML::LoadFile(yaml_path);
}

void m2Iface::init_publishers()
{
    auto pose_state_name    = config["topic"]["pub"]["current_pose"]["name"].as<std::string>();
    auto robot_state_name   = config["topic"]["pub"]["current_robot_state"]["name"].as<std::string>();
    pose_state_pub_         = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_state_name, 1);
    robot_state_pub_        = this->create_publisher<std_msgs::msg::String>(robot_state_name, 1);
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized publishers!");
}

void m2Iface::init_subscribers()
{
    auto joint_states_name  = config["topic"]["sub"]["joint_states"]["name"].as<std::string>();
    joint_state_sub_        = this->create_subscription<sensor_msgs::msg::JointState>(joint_states_name, 1, std::bind(&m2Iface::joint_state_cb, this, _1));
    // Servo twist subscriber
    servo_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "~/servo_twist_cmd", 10, std::bind(&m2Iface::servo_twist_cb, this, _1));
    // Servo position output publisher (Float64MultiArray for forward_position_controller)
    std::string servo_cmd_topic = "forward_position_controller/commands";
    if (config["robot"]["servo_command_topic"]) {
        servo_cmd_topic = config["robot"]["servo_command_topic"].as<std::string>();
    }
    servo_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(servo_cmd_topic, 10);
    // Servo status publisher
    servo_status_pub_ = this->create_publisher<moveit_msgs::msg::ServoStatus>(
        "~/status", 10);
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!"); 
}

void m2Iface::init_services()
{
    auto change_state_name  = config["srv"]["change_robot_state"]["name"].as<std::string>();
    auto set_vel_acc_name   = config["srv"]["set_vel_acc"]["name"].as<std::string>();
    auto set_planner_name   = config["srv"]["set_planner"]["name"].as<std::string>();
    auto set_eelink_name    = config["srv"]["set_eelink"]["name"].as<std::string>();
    auto set_plan_only_name = config["srv"]["set_planonly"]["name"].as<std::string>();
    change_state_srv_       = this->create_service<arm_api2_msgs::srv::ChangeState>(change_state_name, std::bind(&m2Iface::change_state_cb, this, _1, _2));
    set_vel_acc_srv_        = this->create_service<arm_api2_msgs::srv::SetVelAcc>(set_vel_acc_name, std::bind(&m2Iface::set_vel_acc_cb, this, _1, _2));
    set_planner_srv_        = this->create_service<arm_api2_msgs::srv::SetStringParam>(set_planner_name, std::bind(&m2Iface::set_planner_cb, this, _1, _2));
    set_eelink_srv_         = this->create_service<arm_api2_msgs::srv::SetStringParam>(set_eelink_name, std::bind(&m2Iface::set_eelink_cb, this, _1, _2));
    set_plan_only_srv_      = this->create_service<std_srvs::srv::SetBool>(set_plan_only_name, std::bind(&m2Iface::set_plan_only_cb, this, _1, _2));
    add_collision_object_srv_ = this->create_service<arm_api2_msgs::srv::AddCollisionObject>("add_collision_object", std::bind(&m2Iface::add_collision_object_cb, this, _1, _2));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized services!"); 
}

void m2Iface::init_actionservers()
{
    auto move_to_pose_name      = config["action"]["move_to_pose"]["name"].as<std::string>();
    auto move_to_joint_name     = config["action"]["move_to_joint"]["name"].as<std::string>();
    auto move_to_pose_path_name = config["action"]["move_to_pose_path"]["name"].as<std::string>();
    auto gripper_control_name   = config["action"]["gripper_control"]["name"].as<std::string>();

    RCLCPP_INFO(this->get_logger(), "[DEBUG] Action names: move_to_pose='%s'",
                move_to_pose_name.c_str());

    move_to_pose_as_    = rclcpp_action::create_server<arm_api2_msgs::action::MoveCartesian>(this,
                                                                                        move_to_pose_name,
                                                                                        std::bind(&m2Iface::move_to_pose_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::move_to_pose_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::move_to_pose_accepted_cb, this, _1));
    move_to_joint_as_   = rclcpp_action::create_server<arm_api2_msgs::action::MoveJoint>(this,
                                                                                        move_to_joint_name,
                                                                                        std::bind(&m2Iface::move_to_joint_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::move_to_joint_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::move_to_joint_accepted_cb, this, _1));
    move_to_pose_path_as_ = rclcpp_action::create_server<arm_api2_msgs::action::MoveCartesianPath>(this,
                                                                                        move_to_pose_path_name,
                                                                                        std::bind(&m2Iface::move_to_pose_path_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::move_to_pose_path_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::move_to_pose_path_accepted_cb, this, _1));
    gripper_control_as_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(this,
                                                                                        gripper_control_name,
                                                                                        std::bind(&m2Iface::gripper_control_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::gripper_control_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::gripper_control_accepted_cb, this, _1));

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized action servers!");
}
void m2Iface::init_moveit()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "robot_description: " << ROBOT_DESC);
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_group: " << PLANNING_GROUP);
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_frame: " << PLANNING_FRAME);
    RCLCPP_INFO_STREAM(this->get_logger(), "move_group_ns: " << MOVE_GROUP_NS);

    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_moveit: fetchAndSetRobotDescription...");
    if (!fetchAndSetRobotDescription()) {
        throw std::runtime_error("Failed to fetch robot description. Ensure move_group and robot_state_publisher are running.");
    }
    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_moveit: setMoveGroup...");
    moveGroupInit       = setMoveGroup(node_, PLANNING_GROUP, MOVE_GROUP_NS);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_moveit: setPlanningSceneMonitor...");
    pSceneMonitorInit   = setPlanningSceneMonitor(node_, ROBOT_DESC);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_moveit: setRobotModel...");
    robotModelInit      = setRobotModel(node_);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_moveit: PlanningSceneInterface...");
    // Sanitize the namespace string. If "null", use empty string (root/default).
    std::string interface_ns = (MOVE_GROUP_NS == "null") ? "" : MOVE_GROUP_NS;
    m_planningSceneInterface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(interface_ns);
    RCLCPP_INFO(this->get_logger(), "PlanningSceneInterface initialized!");
}

// TODO: Try to replace with auto
// TODO: Try to replace with auto
std::unique_ptr<moveit_servo::Servo> m2Iface::init_servo()
{
    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_servo: Creating ParamListener for 'moveit_servo'...");
    // New Jazzy API - use ParamListener instead of ServoParameters
    // Use main node's parameters (servo params are loaded on this node, not node_)
    servo_param_listener_ = std::make_shared<servo::ParamListener>(
        this->get_node_parameters_interface(),
        this->get_logger(),
        "moveit_servo");
    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_servo: ParamListener created, calling get_params()...");
    auto servo_params = servo_param_listener_->get_params();
    RCLCPP_INFO_STREAM(this->get_logger(), "Servo move_group_name: " << servo_params.move_group_name);

    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_servo: Creating Servo instance...");
    auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_param_listener_, m_pSceneMonitorPtr);
    RCLCPP_INFO(this->get_logger(), "Servo initialized!");
    return servo;
}

void m2Iface::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{   
    std::vector<std::string> jointNames = msg->name;
    std::vector<double> jointPositions = msg->position;
    if(robotModelInit) {m_robotStatePtr->setVariablePositions(jointNames, jointPositions);}; 

}

void m2Iface::servo_twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    latest_twist_cmd_ = *msg;
    new_twist_cmd_ = true;
}

void m2Iface::processServoCommand()
{
    if (!servoPtr || !new_twist_cmd_ || !m_moveGroupPtr) return;

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
        twist_cmd.frame_id = latest_twist_cmd_.header.frame_id.empty() ? EE_LINK_NAME : latest_twist_cmd_.header.frame_id;
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
            status == moveit_servo::StatusCode::HALT_FOR_COLLISION) {
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
        
        // Create and publish Float64MultiArray for forward_position_controller
        std_msgs::msg::Float64MultiArray pos_msg;
        pos_msg.data.reserve(next_state.positions.size());
        for (size_t i = 0; i < next_state.positions.size(); ++i) {
            pos_msg.data.push_back(next_state.positions[i]);
        }
        
        servo_position_pub_->publish(pos_msg);
        last_servo_state_ = next_state;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Servo command processing failed: %s", e.what());
    }
}


void m2Iface::set_vel_acc_cb(const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Request> req, const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Response> res)
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

void m2Iface::set_eelink_cb(const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req, const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res)
{
    EE_LINK_NAME = req->value;
    m_moveGroupPtr->setEndEffectorLink(EE_LINK_NAME);
    res->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set end effector link to " << req->value);
}

void m2Iface::set_planner_cb(const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req,
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

void m2Iface::set_plan_only_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    planOnly = req->data;
    res->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set plan only to " << req->data);
}

void m2Iface::add_collision_object_cb(const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Request> req,
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
    if (req->primitive_type == shape_msgs::msg::SolidPrimitive::SPHERE && req->dimensions.size() != 1) {
        res->success = false;
        res->message = "SPHERE requires 1 dimension [radius]";
        RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
        return;
    }
    if (req->primitive_type == shape_msgs::msg::SolidPrimitive::CYLINDER && req->dimensions.size() != 2) {
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
    for (const auto& dim : req->dimensions) {
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

rclcpp_action::GoalResponse m2Iface::move_to_joint_goal_cb(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const arm_api2_msgs::action::MoveJoint::Goal> goal)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for joint control! Number of joints: "  << goal->joint_state.position.size());
    if(robotState != JOINT_TRAJ_CTL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in joint control mode!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if(goal->joint_state.position.size() != m_currJointPosition.size())
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Number of joint positions does not match the number of joints!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_joint_cancel_cb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>> goal_handle)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel joint control!");
    (void)goal_handle;
    m_moveGroupPtr->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_joint_accepted_cb(std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>> goal_handle)
{
    m_moveToJointGoalHandle_ = goal_handle;
    recivCmd = true;
}

rclcpp_action::GoalResponse m2Iface::move_to_pose_goal_cb(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const arm_api2_msgs::action::MoveCartesian::Goal> goal)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for Cartesian control!");
    if(robotState != CART_TRAJ_CTL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in Cartesian control mode!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if(goal->goal.header.frame_id != PLANNING_FRAME)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Pose frame_id is not planning frame! PLANNING_FRAME: " << PLANNING_FRAME);
        return rclcpp_action::GoalResponse::REJECT;
    }
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_pose_cancel_cb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>> goal_handle)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel Cartesian control!");
    (void)goal_handle;
    m_moveGroupPtr->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_pose_accepted_cb(std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>> goal_handle)
{
    m_moveToPoseGoalHandle_ = goal_handle;
    recivCmd = true;
}

rclcpp_action::GoalResponse m2Iface::move_to_pose_path_goal_cb(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const arm_api2_msgs::action::MoveCartesianPath::Goal> goal)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for Cartesian path control!");
    if(robotState != CART_TRAJ_CTL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in Cartesian control mode!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if(goal->poses.size() < 2)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Number of poses in path is less than 2!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if(goal->poses[0].header.frame_id != PLANNING_FRAME)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Path frame_id is not planning frame! PLANNING_FRAME: " << PLANNING_FRAME);
        return rclcpp_action::GoalResponse::REJECT;
    }
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_pose_path_cancel_cb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>> goal_handle)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel Cartesian path control!");
    (void)goal_handle;
    m_moveGroupPtr->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_pose_path_accepted_cb(std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>> goal_handle)
{
    m_moveToPosePathGoalHandle_ = goal_handle;
    recivTraj = true;
}

rclcpp_action::GoalResponse m2Iface::gripper_control_goal_cb(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for gripper control!");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::gripper_control_cancel_cb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::gripper_control_accepted_cb(std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
{
    m_gripperControlGoalHandle_ = goal_handle;
    recivGripperCmd = true;
}

bool m2Iface::switchControllersForServo(bool enter_servo)
{
    if (!switch_controller_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "controller_manager/switch_controller not available");
        return false;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->strictness = controller_manager_msgs::srv::SwitchController_Request::BEST_EFFORT;
    request->activate_asap = true;
    request->timeout.sec = 5;
    request->timeout.nanosec = 0;

    if (enter_servo) {
        request->activate_controllers = { forward_position_controller_name_ };
        request->deactivate_controllers = { scaled_joint_trajectory_controller_name_ };
    } else {
        request->activate_controllers = { scaled_joint_trajectory_controller_name_ };
        request->deactivate_controllers = { forward_position_controller_name_ };
    }

    auto future = switch_controller_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "switch_controller call failed (timeout)");
        return false;
    }
    auto response = future.get();
    if (!response->ok) {
        RCLCPP_ERROR(this->get_logger(), "switch_controller failed: %s", response->message.c_str());
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Controllers switched: %s -> %s",
        enter_servo ? "scaled_joint_trajectory" : "forward_position",
        enter_servo ? "forward_position" : "scaled_joint_trajectory");
    return true;
}

void m2Iface::change_state_cb(const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req, 
                              const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res)
{
    auto itr = std::find(std::begin(stateNames), std::end(stateNames), req->state); 
    
    if ( itr != std::end(stateNames))
    {
        int wantedIndex_ = std::distance(stateNames, itr); 
        state newState = static_cast<state>(wantedIndex_);
        state prevState = robotState;

        if (enable_servo && newState == SERVO_CTL && prevState != SERVO_CTL) {
            if (!switchControllersForServo(true)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to switch controllers for servo mode");
                res->success = false;
                return;
            }
        } else if (enable_servo && prevState == SERVO_CTL && newState != SERVO_CTL) {
            if (!switchControllersForServo(false)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to switch controllers when leaving servo mode");
                res->success = false;
                return;
            }
        }

        robotState = newState;
        RCLCPP_INFO_STREAM(this->get_logger(), "Switching state!");
        res->success = true;  
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "Failed switching to state " << req->state); 
        res->success = false; 
    } 
}

bool m2Iface::fetchAndSetRobotDescription()
{
    std::string urdf_topic;
    std::string srdf_topic;
    if (MOVE_GROUP_NS.empty() || MOVE_GROUP_NS == "null") {
        urdf_topic = "/robot_description";
        srdf_topic = "/move_group/robot_description_semantic";
    } else {
        urdf_topic = "/" + MOVE_GROUP_NS + "/robot_description";
        srdf_topic = "/" + MOVE_GROUP_NS + "/move_group/robot_description_semantic";
    }
    constexpr double timeout_sec = 10.0;

    std::string fetch_node_name = "moveit2_iface_fetch_robot_desc";
    if (!MOVE_GROUP_NS.empty() && MOVE_GROUP_NS != "null") {
        fetch_node_name += "_" + MOVE_GROUP_NS;
    }
    auto fetch_node = std::make_shared<rclcpp::Node>(fetch_node_name);
    bool use_sim_time = false;
    if (this->get_parameter("use_sim_time", use_sim_time)) {
        fetch_node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
    }

    std::string urdf_string;
    std::string srdf_string;
    auto urdf_received = std::make_shared<std::promise<void>>();
    auto srdf_received = std::make_shared<std::promise<void>>();
    auto urdf_done = std::make_shared<std::atomic<bool>>(false);
    auto srdf_done = std::make_shared<std::atomic<bool>>(false);

    rclcpp::QoS qos = rclcpp::QoS(1).transient_local();

    auto urdf_sub = fetch_node->create_subscription<std_msgs::msg::String>(
        urdf_topic, qos,
        [&urdf_string, urdf_received, urdf_done](const std_msgs::msg::String::SharedPtr msg) {
            if (!urdf_done->exchange(true)) {
                urdf_string = msg->data;
                urdf_received->set_value();
            }
        });
    auto srdf_sub = fetch_node->create_subscription<std_msgs::msg::String>(
        srdf_topic, qos,
        [&srdf_string, srdf_received, srdf_done](const std_msgs::msg::String::SharedPtr msg) {
            if (!srdf_done->exchange(true)) {
                srdf_string = msg->data;
                srdf_received->set_value();
            }
        });

    RCLCPP_INFO(this->get_logger(), "Waiting for %s and %s (timeout: %.1fs)", urdf_topic.c_str(), srdf_topic.c_str(), timeout_sec);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(fetch_node);

    auto urdf_future = urdf_received->get_future();
    auto srdf_future = srdf_received->get_future();

    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some(std::chrono::milliseconds(100));
        if (urdf_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready &&
            srdf_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            break;
        }
    }

    executor.remove_node(fetch_node);

    if (urdf_string.empty() || srdf_string.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for robot_description topics.");
        return false;
    }

    /* Debug: extract and log robot names from URDF/SRDF to diagnose "Semantic description is not specified for the same robot" */
    std::regex urdf_name_regex(R"(<robot[^>]*\s+name\s*=\s*["']([^"']+)["'])");
    std::regex srdf_name_regex(R"(<robot[^>]*\s+name\s*=\s*["']([^"']+)["'])");
    std::smatch urdf_match, srdf_match;
    std::string urdf_robot_name = "(not found)";
    std::string srdf_robot_name = "(not found)";
    if (std::regex_search(urdf_string, urdf_match, urdf_name_regex)) {
        urdf_robot_name = urdf_match[1].str();
    }
    if (std::regex_search(srdf_string, srdf_match, srdf_name_regex)) {
        srdf_robot_name = srdf_match[1].str();
    }
    RCLCPP_INFO(this->get_logger(), "[DEBUG] URDF robot name: '%s' | SRDF robot name: '%s' | Match: %s",
        urdf_robot_name.c_str(), srdf_robot_name.c_str(),
        (urdf_robot_name == srdf_robot_name ? "OK" : "MISMATCH - ensure same robot/ur_type in move_group and robot_state_publisher"));

    node_->declare_parameter<std::string>("robot_description", "");
    node_->declare_parameter<std::string>("robot_description_semantic", "");
    node_->set_parameters({
        rclcpp::Parameter("robot_description", urdf_string),
        rclcpp::Parameter("robot_description_semantic", srdf_string),
    });

    RCLCPP_INFO(this->get_logger(), "Robot description fetched and set on parameter server.");
    return true;
}

bool m2Iface::setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs)
{
    // check if moveNs is empty
    if (moveNs == "null") moveNs=""; 

    //https://github.com/moveit/moveit2/issues/496
    // Use 30s timeout instead of default -1 (unlimited) to avoid indefinite block
    constexpr int WAIT_FOR_SERVERS_SEC = 30;
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setMoveGroup: Creating MoveGroupInterface (wait_for_servers=%ds)...", WAIT_FOR_SERVERS_SEC);
    m_moveGroupPtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        nodePtr,
        moveit::planning_interface::MoveGroupInterface::Options(groupName, "robot_description", moveNs),
        nullptr,  // tf_buffer
        rclcpp::Duration::from_seconds(WAIT_FOR_SERVERS_SEC));
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setMoveGroup: MoveGroupInterface created.");

    double POS_TOL = 0.0000001; 
    // set move group stuff
    m_moveGroupPtr->setEndEffectorLink(EE_LINK_NAME); 
    m_moveGroupPtr->setPoseReferenceFrame(PLANNING_FRAME); 
    m_moveGroupPtr->setGoalPositionTolerance(POS_TOL);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setMoveGroup: Calling startStateMonitor()...");
    m_moveGroupPtr->startStateMonitor();
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setMoveGroup: startStateMonitor() returned."); 

    // velocity scaling
    m_moveGroupPtr->setMaxVelocityScalingFactor(INIT_VEL_SCALING);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(INIT_ACC_SCALING);
    // executor
    executor_->add_node(node_); 
    executor_thread_ = std::thread([this]() {executor_->spin();});
    RCLCPP_INFO_STREAM(this->get_logger(), "Move group interface set up!"); 
    return true; 
}

/* This is not neccessary*/
bool m2Iface::setRobotModel(rclcpp::Node::SharedPtr nodePtr)
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

bool m2Iface::setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name)
{
    // https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
    // https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/planning_scene/src/planning_scene_tutorial.cpp
    m_pSceneMonitorPtr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(nodePtr, name);

    // Use move_group's monitored_planning_scene topic (namespaced for dual-arm to avoid cross-talk)
    std::string scene_topic = MOVE_GROUP_NS.empty()
        ? "/monitored_planning_scene"
        : "/" + MOVE_GROUP_NS + "/monitored_planning_scene";
    m_pSceneMonitorPtr->startSceneMonitor(scene_topic);

    if (m_pSceneMonitorPtr->getPlanningScene())
    {
        m_pSceneMonitorPtr->startStateMonitor(JOINT_STATES);
        m_pSceneMonitorPtr->setPlanningScenePublishingFrequency(25);

        // Publish to namespaced topic for dual-arm to avoid both robots overwriting the same topic
        std::string publish_topic = MOVE_GROUP_NS.empty()
            ? "/moveit_servo/publish_planning_scene"
            : "/" + MOVE_GROUP_NS + "/moveit_servo/publish_planning_scene";
        m_pSceneMonitorPtr->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         publish_topic);
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

void m2Iface::planAndExecJoint()
{   
    const auto feedback = std::make_shared<arm_api2_msgs::action::MoveJoint::Feedback>();
    const auto result = std::make_shared<arm_api2_msgs::action::MoveJoint::Result>();
    feedback->set__status("planning");
    m_moveToJointGoalHandle_->publish_feedback(feedback);

    const auto goalJointState = m_moveToJointGoalHandle_->get_goal()->joint_state;
    m_moveGroupPtr->setJointValueTarget(goalJointState);
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success = planWithPlanner(plan, eager_execution);
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning to joint space goal: " << (success ? "SUCCEEDED" : "FAILED"));
    
    if (success && planOnly) {
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        RCLCPP_INFO_STREAM(this->get_logger(), "                   Plan only mode!");
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        result->success = true;
        m_moveToJointGoalHandle_->succeed(result);
    }
    else if (success) {
        //addTimestampsToTrajectory(plan.trajectory);
        //printTimestamps(plan.trajectory);

        feedback->set__status("executing");
        m_moveToJointGoalHandle_->publish_feedback(feedback);
        auto errorcode = m_moveGroupPtr->execute(plan);
        if(errorcode == moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_INFO_STREAM(this->get_logger(), "Execution succeeded!");
            result->success = true;
            m_moveToJointGoalHandle_->succeed(result);
        }
        else{
            RCLCPP_ERROR_STREAM(this->get_logger(), "Execution failed with error code");
            result->success = true;
            m_moveToJointGoalHandle_->abort(result); 
        }
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        result->success = false;
        m_moveToJointGoalHandle_->abort(result); 
    }
}

void m2Iface::planAndExecPose()
{   
    const auto feedback = std::make_shared<arm_api2_msgs::action::MoveCartesian::Feedback>();
    const auto result = std::make_shared<arm_api2_msgs::action::MoveCartesian::Result>();
    feedback->set__status("planning");
    m_moveToPoseGoalHandle_->publish_feedback(feedback);

    auto goalPose = m_moveToPoseGoalHandle_->get_goal()->goal;
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning to Cartesian Pose!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Current pose is: " << m_currPoseState.pose.position.x << " " << m_currPoseState.pose.position.y << " " << m_currPoseState.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Target pose is: " << goalPose.pose.position.x << " " << goalPose.pose.position.y << " " << goalPose.pose.position.z);

    m_moveGroupPtr->setPoseTarget(goalPose);
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success = planWithPlanner(plan, eager_execution);
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning to pose goal: " << (success ? "SUCCEEDED" : "FAILED"));

    if (success && planOnly) {
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        RCLCPP_INFO_STREAM(this->get_logger(), "                   Plan only mode!");
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        result->success = true;
        m_moveToPoseGoalHandle_->succeed(result);
    }
    else if(success){
        //addTimestampsToTrajectory(plan.trajectory);
        //printTimestamps(plan.trajectory);

        feedback->set__status("executing");
        m_moveToPoseGoalHandle_->publish_feedback(feedback);
        auto errorcode = m_moveGroupPtr->execute(plan);
        if(errorcode == moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_INFO_STREAM(this->get_logger(), "Execution succeeded!");
            result->success = true;
            m_moveToPoseGoalHandle_->succeed(result);
        }
        else{
            RCLCPP_ERROR_STREAM(this->get_logger(), "Execution failed with error code, time stamps:");
            printTimestamps(plan.trajectory);
            result->success = false;
            m_moveToPoseGoalHandle_->abort(result); 
        }
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        result->success = false;
        m_moveToPoseGoalHandle_->abort(result); 
    }
}

void m2Iface::planAndExecPosePath()
{   
    const auto feedback = std::make_shared<arm_api2_msgs::action::MoveCartesianPath::Feedback>();
    const auto result = std::make_shared<arm_api2_msgs::action::MoveCartesianPath::Result>();
    feedback->set__status("planning");
    m_moveToPosePathGoalHandle_->publish_feedback(feedback);

    auto goalPoseStampeds = m_moveToPosePathGoalHandle_->get_goal()->poses;
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning Cartesian path!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Current pose is: " << m_currPoseState.pose.position.x << " " << m_currPoseState.pose.position.y << " " << m_currPoseState.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Target pose is: " << goalPoseStampeds[goalPoseStampeds.size()-1].pose.position.x << " " << goalPoseStampeds[goalPoseStampeds.size()-1].pose.position.y << " " << goalPoseStampeds[goalPoseStampeds.size()-1].pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Creating Cartesian waypoints!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Number of waypoints in the path: " << goalPoseStampeds.size());

    // extract all poses from the PoseStamped messages
    std::vector<geometry_msgs::msg::Pose> goalPoses;
    for (auto pose : goalPoseStampeds)
    {
        goalPoses.push_back(pose.pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    // TODO: Set as params that can be configured in YAML!
    double jumpThr = 0.0; 
    double eefStep = 0.02; 
    bool success = (m_moveGroupPtr->computeCartesianPath(goalPoses, eefStep, jumpThr, trajectory, true) == 1.0);

    if (success && planOnly) {
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        RCLCPP_INFO_STREAM(this->get_logger(), "                   Plan only mode!");
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        result->success = true;
        m_moveToPosePathGoalHandle_->succeed(result);
    }
    else if(success){
        addTimestampsToTrajectory(trajectory);

        feedback->set__status("executing");
        m_moveToPosePathGoalHandle_->publish_feedback(feedback);
        m_moveGroupPtr->execute(trajectory);
        result->success = true;
        m_moveToPosePathGoalHandle_->succeed(result);
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        result->success = false;
        m_moveToPosePathGoalHandle_->abort(result); 
    }
    

}

void m2Iface::printTimestamps(const moveit_msgs::msg::RobotTrajectory &trajectory){
    trajectory_msgs::msg::JointTrajectory jointTrajectory = trajectory.joint_trajectory;
    std::vector<std::string> joint_names = jointTrajectory.joint_names;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points = jointTrajectory.points;
    for (long unsigned int i = 0; i < points.size(); i++){
        trajectory_msgs::msg::JointTrajectoryPoint point = points[i];
        rclcpp::Duration duration = point.time_from_start;
        RCLCPP_INFO_STREAM(this->get_logger(), "Point " << i << " - time_from_start [s]: " << duration.seconds());
    } 
}

void m2Iface::addTimestampsToTrajectory(moveit_msgs::msg::RobotTrajectory &trajectory){
    // The trajectory created with computeCartesianPath() needs to be modified so it will include velocities as well.
    // reference: https://groups.google.com/g/moveit-users/c/MOoFxy2exT4
    auto current_state = m_moveGroupPtr->getCurrentState(1.0);
    if (!current_state) {
        RCLCPP_WARN(this->get_logger(), "addTimestampsToTrajectory: Failed to fetch current robot state");
        return;
    }
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(current_state->getRobotModel(), PLANNING_GROUP);
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*current_state, trajectory);
    // Thrid create a TimeOptimalTrajectoryGeneration object
    trajectory_processing::TimeOptimalTrajectoryGeneration iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt, max_vel_scaling_factor, max_acc_scaling_factor);
    RCLCPP_INFO_STREAM(this->get_logger(), "Computed time stamp " << (success ? "SUCCEEDED" : "FAILED"));
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);
} 

bool m2Iface::planWithPlanner(moveit::planning_interface::MoveGroupInterface::Plan &plan, bool eagerExecution){
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
    for(int i = 0; i < int(tries_per_planner*planners.size()); i++){

        int planner_index = i / tries_per_planner;
        int planner_try = i % tries_per_planner;

        m_moveGroupPtr->setPlanningPipelineId(planners[planner_index].first);
        m_moveGroupPtr->setPlannerId(planners[planner_index].second);

        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        success = static_cast<bool>(m_moveGroupPtr->plan(plan_));

        if(success){
            RCLCPP_INFO(this->get_logger(), "%s found plan %d with %d points", 
                planners[planner_index].second.c_str(), i, int(plan.trajectory.joint_trajectory.points.size()));
            if (eagerExecution) // if eager execution is true, return after first successful plan
            {   // Set local planned path as the plan to be executed
                plan = plan_; 
                return true;
            } 
            all_plans.push_back(plan_);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "%s failed to find plan %d", 
                planners[planner_index].second.c_str(), i);
        }

        if(planner_try == tries_per_planner - 1 && all_plans.size() >= 3){
            RCLCPP_INFO(this->get_logger(), "Found %d plans, stopping planning", int(all_plans.size()));
        }
    }

    if(all_plans.size() == 0){
        RCLCPP_INFO_STREAM(this->get_logger(), "All planners failed!");
        return false;
    }
    else{
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << all_plans.size() << " plans!");
    }

    // find the best plan from the list of plans
    auto best_plan = std::min_element(all_plans.begin(), all_plans.end(), [](auto const& a, auto const& b){
        return a.trajectory.joint_trajectory.points.size() < b.trajectory.joint_trajectory.points.size();
    });

    plan = *best_plan;
    RCLCPP_INFO(this->get_logger(), "Best plan selected with %d points.", int(best_plan->trajectory.joint_trajectory.points.size()));
    return true;
}

void m2Iface::getArmState() 
{   
    auto current_state = m_moveGroupPtr->getCurrentState(1.0);
    if (!current_state) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "getArmState: Failed to fetch current robot state");
        return;
    }
    m_robotStatePtr = current_state;
    m_robotStatePtr->update();

    const moveit::core::JointModelGroup* joint_model_group = m_robotStatePtr->getJointModelGroup(PLANNING_GROUP);
    if (joint_model_group) {
        m_robotStatePtr->copyJointGroupPositions(joint_model_group, m_currJointPosition);
    }
    Eigen::Isometry3d currentPose_ = m_robotStatePtr->getFrameTransform(EE_LINK_NAME);
    m_currPoseState = utils::convertIsometryToMsg(currentPose_);
    m_currPoseState.header.frame_id = m_moveGroupPtr->getPlanningFrame();
}

bool m2Iface::run()
{
    if(!nodeInit)       {RCLCPP_ERROR(this->get_logger(), "Node not fully initialized!"); return false;} 
    if(!moveGroupInit)  {RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized!"); return false;} 

    getArmState(); 
    pose_state_pub_->publish(m_currPoseState);
    std_msgs::msg::String stateMsg;
    stateMsg.data = stateNames[robotState];
    robot_state_pub_->publish(stateMsg);

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
    if (robotState != SERVO_CTL && servoEntered) {servoPtr->setCollisionChecking(false); servoEntered=false;} // New API: no setPaused 

    if (robotState == JOINT_TRAJ_CTL)
    {
       if (recivCmd) {
           planAndExecJoint();
           recivCmd = false; 
       } 
    }

    if (robotState == CART_TRAJ_CTL)
    {   
        // TODO: Beware if both are true at the same time, shouldn't occur, 
        if (recivCmd) {
            planAndExecPose();
            recivCmd = false; 
        } 

        if (recivTraj){
            planAndExecPosePath();
            recivTraj = false; 
        }
    }

    if (robotState == SERVO_CTL && servoPtr)
    {
        if (!servoEntered)
        {
            // Clear any old twist commands
            latest_twist_cmd_ = geometry_msgs::msg::TwistStamped();
            latest_twist_cmd_.header.frame_id = "base_link";
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
                zero_twist.frame_id = "base_link";
                zero_twist.velocities.fill(0.0);
                auto current_state = m_moveGroupPtr->getCurrentState(1.0);
                if (current_state) {
                    servoPtr->setCommandType(moveit_servo::CommandType::TWIST);
                    servoPtr->getNextJointState(current_state, zero_twist);
                }
                servoPtr->setCollisionChecking(true);
            } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "Servo initialization failed: %s", e.what()); }
            RCLCPP_INFO(this->get_logger(), "Servo mode activated! Send twist commands to ~/servo_twist_cmd");
            servo_entered_time_ = this->now();
            servoEntered = true;
        }
        // Process servo commands every cycle
        processServoCommand();

    }

    if(recivGripperCmd){
    
        auto goal = m_gripperControlGoalHandle_->get_goal();
        float position = goal->command.position;
        float effort = goal->command.max_effort;

        auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
        bool success = gripper.send_gripper_command(position, effort);

        if(success){
            // TODO: Wrap this in methods
            RCLCPP_INFO_STREAM(this->get_logger(), "Gripper command succeeded!");
            result->position = gripper.get_position();
            result->effort = gripper.get_effort();
            result->stalled = gripper.is_stalled();
            result->reached_goal = gripper.reached_goal();
            m_gripperControlGoalHandle_->succeed(result);
        }
        else{
            RCLCPP_ERROR_STREAM(this->get_logger(), "Gripper command failed!");
            result->reached_goal = gripper.reached_goal();
            m_gripperControlGoalHandle_->abort(result);
        }
        recivGripperCmd = false;
    }

    return true;     
}



