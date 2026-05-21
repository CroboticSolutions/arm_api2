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

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <thread>
#include <vector>

#include <std_msgs/msg/string.hpp>

rclcpp::Node::SharedPtr m2SimpleIface::createMoveitNode(rclcpp::Node* parent)
{
    std::string cfg_path;
    parent->get_parameter("config_path", cfg_path);
    const YAML::Node cfg = YAML::LoadFile(cfg_path);
    std::string move_ns = cfg["robot"]["move_group_ns"].as<std::string>();
    if (move_ns == "null") move_ns = "";
    if (parent->has_parameter("move_group_ns")) {
        std::string param_ns = parent->get_parameter("move_group_ns").get_value<std::string>();
        if (!param_ns.empty() && param_ns != "null") {
            move_ns = param_ns;
        }
    }
    std::string joint_states_topic;
    if (!move_ns.empty()) {
        joint_states_topic = move_ns + "/joint_states";
    } else if (cfg["robot"]["joint_states"]) {
        joint_states_topic = cfg["robot"]["joint_states"].as<std::string>();
    } else {
        joint_states_topic = "joint_states";
    }

    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    opts.automatically_declare_parameters_from_overrides(false);
    std::vector<std::string> args;
    if (!move_ns.empty()) {
        args = {"--ros-args", "-r", "__ns:=/" + move_ns};
        RCLCPP_INFO(parent->get_logger(), "MoveIt node in namespace /%s (avoids dual-robot cache collision)", move_ns.c_str());
    } else {
        args = {"--ros-args", "-r", "__ns:=/"};
    }
    if (!joint_states_topic.empty() && joint_states_topic != "joint_states") {
        std::string remap_target = (joint_states_topic[0] == '/') ? joint_states_topic : "/" + joint_states_topic;
        args.push_back("-r");
        args.push_back("joint_states:=" + remap_target);
        RCLCPP_INFO(parent->get_logger(), "Remapping joint_states to %s (move_ns=%s)", remap_target.c_str(), move_ns.c_str());
    }
    opts.arguments(args);

    // Copy moveit_servo and kinematics params from parent so MoveIt node has them (RobotModelLoader/KinematicsPluginLoader)
    std::vector<rclcpp::Parameter> param_overrides;
    auto servo_result = parent->list_parameters({"moveit_servo"}, 0);
    for (const auto& name : servo_result.names) {
        if (parent->has_parameter(name)) {
            param_overrides.push_back(parent->get_parameter(name));
        }
    }
    auto kin_result = parent->list_parameters({"manipulator"}, 0);
    for (const auto& name : kin_result.names) {
        if (parent->has_parameter(name)) {
            param_overrides.push_back(parent->get_parameter(name));
        }
    }
    if (!param_overrides.empty()) {
        opts.parameter_overrides(param_overrides);
    }

    std::string node_name = "moveit2_simple_iface_node";
    if (!move_ns.empty()) {
        node_name += "_" + move_ns;
    }
    return std::make_shared<rclcpp::Node>(node_name, opts);
}

bool m2SimpleIface::fetchAndSetRobotDescription()
{
    std::string urdf_topic;
    std::string srdf_topic;
    if (MOVE_GROUP_NS.empty() || MOVE_GROUP_NS == "null") {
        urdf_topic = "/robot_description";
        srdf_topic = "/robot_description_semantic";
    } else {
        urdf_topic = "/" + MOVE_GROUP_NS + "/robot_description";
        srdf_topic = "/" + MOVE_GROUP_NS + "/robot_description_semantic";
    }
    constexpr double timeout_sec = 10.0;

    std::string fetch_node_name = "moveit2_simple_iface_fetch_robot_desc";
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

    RCLCPP_INFO(this->get_logger(), "Waiting for %s and %s (timeout: %.1fs)",
                urdf_topic.c_str(), srdf_topic.c_str(), timeout_sec);

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

    std::string rd_param = ROBOT_DESC;
    std::string rd_semantic_param = rd_param + "_semantic";
    if (!MOVE_GROUP_NS.empty() && MOVE_GROUP_NS != "null") {
        rd_param = "robot_description_" + MOVE_GROUP_NS;
        rd_semantic_param = rd_param + "_semantic";
    }
    node_->declare_parameter(rd_param, "");
    node_->declare_parameter(rd_semantic_param, "");
    node_->set_parameters({
        rclcpp::Parameter(rd_param, urdf_string),
        rclcpp::Parameter(rd_semantic_param, srdf_string),
    });

    RCLCPP_INFO(this->get_logger(), "Robot description fetched and set (param=%s).", rd_param.c_str());
    return true;
}

m2SimpleIface::m2SimpleIface(const rclcpp::NodeOptions &options)
    : Node("moveit2_simple_iface", options), node_(createMoveitNode(this)), gripper(node_) 
{   
    this->get_parameter("config_path", config_path);
    this->get_parameter("enable_servo", enable_servo);
    this->get_parameter("dt", dt);

    bool use_sim_time = false;
    this->get_parameter("use_sim_time", use_sim_time);
    node_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

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
    max_vel_scaling_factor = config["robot"]["max_vel_scaling_factor"].as<float>();
    max_acc_scaling_factor = config["robot"]["max_acc_scaling_factor"].as<float>();
    
    ns_ = this->get_namespace();
    if (!this->has_parameter("move_group_ns")) {
        this->declare_parameter<std::string>("move_group_ns", "");
    }
    std::string param_move_group_ns = this->get_parameter("move_group_ns").get_value<std::string>();
    if (!param_move_group_ns.empty()) {
        MOVE_GROUP_NS = param_move_group_ns;
    } 	
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

std::string m2SimpleIface::resolve_topic_name(const std::string& name) const
{
  if (ns_.empty() || ns_ == "/") return name;
  return (ns_.back() == '/' ? ns_ : ns_ + "/") + name;
}

void m2SimpleIface::init_publishers()
{   
    auto pose_state_name = config["topic"]["pub"]["current_pose"]["name"].as<std::string>(); 
    pose_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(resolve_topic_name(pose_state_name), 1); 
    auto current_robot_state_name = config["topic"]["pub"]["current_robot_state"]["name"].as<std::string>(); 
    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>(resolve_topic_name(current_robot_state_name), 1);
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized publishers!");
}

void m2SimpleIface::init_subscribers()
{
    auto pose_cmd_name = config["topic"]["sub"]["cmd_pose"]["name"].as<std::string>(); 
    auto cart_traj_cmd_name = config["topic"]["sub"]["cmd_traj"]["name"].as<std::string>(); 
    auto joint_states_name = config["topic"]["sub"]["joint_states"]["name"].as<std::string>();
    pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(resolve_topic_name(pose_cmd_name), 1, std::bind(&m2SimpleIface::pose_cmd_cb, this, _1));
    ctraj_cmd_sub_ = this->create_subscription<arm_api2_msgs::msg::CartesianWaypoints>(resolve_topic_name(cart_traj_cmd_name), 1, std::bind(&m2SimpleIface::cart_poses_cb, this, _1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(resolve_topic_name(joint_states_name), 1, std::bind(&m2SimpleIface::joint_state_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!"); 
}

void m2SimpleIface::init_services()
{
    auto change_state_name = config["srv"]["change_robot_state"]["name"].as<std::string>(); 
    auto set_vel_acc_name  = config["srv"]["set_vel_acc"]["name"].as<std::string>();
    auto set_planner_name  = config["srv"]["set_planner"]["name"].as<std::string>();
    auto open_gripper_name = config["srv"]["open_gripper"]["name"].as<std::string>(); 
    auto close_gripper_name= config["srv"]["close_gripper"]["name"].as<std::string>();
    change_state_srv_ = this->create_service<arm_api2_msgs::srv::ChangeState>(resolve_topic_name(change_state_name), std::bind(&m2SimpleIface::change_state_cb, this, _1, _2)); 
    set_vel_acc_srv_  = this->create_service<arm_api2_msgs::srv::SetVelAcc>(resolve_topic_name(set_vel_acc_name), std::bind(&m2SimpleIface::set_vel_acc_cb, this, _1, _2));
    set_planner_srv_  = this->create_service<arm_api2_msgs::srv::SetStringParam>(resolve_topic_name(set_planner_name), std::bind(&m2SimpleIface::set_planner_cb, this, _1, _2));
    open_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(resolve_topic_name(open_gripper_name), std::bind(&m2SimpleIface::open_gripper_cb, this, _1, _2));
    close_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(resolve_topic_name(close_gripper_name), std::bind(&m2SimpleIface::close_gripper_cb, this, _1, _2));
    add_collision_object_srv_ = this->create_service<arm_api2_msgs::srv::AddCollisionObject>(resolve_topic_name("add_collision_object"), std::bind(&m2SimpleIface::add_collision_object_cb, this, _1, _2));
    add_grasped_object_srv_ = this->create_service<arm_api2_msgs::srv::AddGraspedObject>(resolve_topic_name("arm/add_grasped_object"), std::bind(&m2SimpleIface::add_grasped_object_cb, this, _1, _2));
    set_path_constraints_srv_ = this->create_service<arm_api2_msgs::srv::SetPathConstraints>(resolve_topic_name("arm/set_path_constraints"), std::bind(&m2SimpleIface::set_path_constraints_cb, this, _1, _2));
    clear_path_constraints_srv_ = this->create_service<arm_api2_msgs::srv::ClearPathConstraints>(resolve_topic_name("arm/clear_path_constraints"), std::bind(&m2SimpleIface::clear_path_constraints_cb, this, _1, _2));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized services!"); 
}

void m2SimpleIface::init_moveit()
{
    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_moveit ENTER");
    RCLCPP_INFO_STREAM(this->get_logger(), "robot_description: " << ROBOT_DESC);
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_group: " << PLANNING_GROUP);
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_frame: " << PLANNING_FRAME);
    RCLCPP_INFO_STREAM(this->get_logger(), "move_group_ns: " << MOVE_GROUP_NS);

    RCLCPP_INFO(this->get_logger(), "[DEBUG] fetchAndSetRobotDescription...");
    if (!fetchAndSetRobotDescription()) {
        throw std::runtime_error("Failed to fetch robot description. Ensure move_group and robot_state_publisher are running.");
    }

    RCLCPP_INFO(this->get_logger(), "[DEBUG] Calling setMoveGroup...");
    moveGroupInit = setMoveGroup(node_, PLANNING_GROUP, MOVE_GROUP_NS);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setMoveGroup returned: %s", moveGroupInit ? "true" : "false");
    if (!moveGroupInit) {
        RCLCPP_ERROR(this->get_logger(), "[DEBUG] setMoveGroup FAILED - aborting init_moveit");
        return;
    }

    std::string robot_desc_param = ROBOT_DESC;
    if (!MOVE_GROUP_NS.empty() && MOVE_GROUP_NS != "null") {
        robot_desc_param = "robot_description_" + MOVE_GROUP_NS;
    }
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Calling setPlanningSceneMonitor...");
    pSceneMonitorInit = setPlanningSceneMonitor(node_, robot_desc_param);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setPlanningSceneMonitor returned: %s", pSceneMonitorInit ? "true" : "false");

    std::string interface_ns = (MOVE_GROUP_NS == "null") ? "" : MOVE_GROUP_NS;
    m_planningSceneInterface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(interface_ns);
    RCLCPP_INFO(this->get_logger(), "PlanningSceneInterface initialized!");

    RCLCPP_INFO(this->get_logger(), "[DEBUG] Calling setRobotModel...");
    robotModelInit = setRobotModel(node_, robot_desc_param);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setRobotModel returned: %s", robotModelInit ? "true" : "false");

    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_moveit COMPLETE");
}

// TODO: Try to replace with auto
std::unique_ptr<moveit_servo::Servo> m2SimpleIface::init_servo()
{
    // moveit_servo params are passed via parameter_overrides in createMoveitNode (when namespaced)
    auto servoParams = moveit_servo::ServoParameters::makeServoParameters(node_);
    RCLCPP_INFO_STREAM(this->get_logger(), "ee_frame_name: " << servoParams->ee_frame_name);
    servoParams->get("moveit_servo", node_->get_node_parameters_interface());

    auto servo = std::make_unique<moveit_servo::Servo>(node_, servoParams, m_pSceneMonitorPtr); 
    RCLCPP_INFO(this->get_logger(), "Servo initialized!"); 
    return servo;
}

void m2SimpleIface::pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        m_currPoseCmd.header.frame_id = PLANNING_FRAME;
        m_currPoseCmd.pose = msg->pose;
    }
    recivCmd.store(true);  // Always accept new commands (enables retry when commander re-sends same pose)
    RCLCPP_INFO_STREAM(this->get_logger(), "recivCmd: " << recivCmd.load());
}

void m2SimpleIface::cart_poses_cb(const arm_api2_msgs::msg::CartesianWaypoints::SharedPtr msg)
{
    // TODO: Maybe implement same check for as the pose_cmd
    {
        std::lock_guard<std::mutex> lock(cart_waypoints_mutex_);
        m_cartesianWaypoints = msg->poses;
    }
    recivTraj.store(true);
}

void m2SimpleIface::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::vector<std::string> jointNames = msg->name;
    std::vector<double> jointPositions = msg->position;
    if (jointNames.size() != jointPositions.size()) {
        return;
    }
    if (robotModelInit) {
        try {
            std::lock_guard<std::mutex> lock(robot_state_mutex_);
            if (m_robotStatePtr) {
                m_robotStatePtr->setVariablePositions(jointNames, jointPositions);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "joint_state_cb: ignored message (%s)", e.what());
        }
    }
}

void m2SimpleIface::open_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    gripper.open();
}

void m2SimpleIface::close_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                     const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    gripper.close(); 
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
    
    if (!waitForMoveGroupExecutionIdle("set_planner")) {
        res->success = false;
        RCLCPP_WARN(this->get_logger(), "set_planner: motion still running or wait timeout — planner not changed");
        return;
    }

    // Set the planner
    try
    {
        std::lock_guard<std::mutex> lock(move_group_mutex_);
        if (!m_moveGroupPtr) {
            throw std::runtime_error("MoveGroup not initialized");
        }
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

    if (!waitForMoveGroupExecutionIdle("add_collision_object")) {
        res->success = false;
        res->message = "Timeout waiting for motion to finish before adding collision object";
        return;
    }

    {
        std::lock_guard<std::mutex> lock(move_group_mutex_);
        m_planningSceneInterface->addCollisionObjects(collision_objects);
    }

    RCLCPP_INFO(this->get_logger(), "Added collision object '%s' to planning scene", req->id.c_str());
    res->success = true;
    res->message = "Collision object added successfully";
}

void m2SimpleIface::add_grasped_object_cb(const std::shared_ptr<arm_api2_msgs::srv::AddGraspedObject::Request> req,
                                          const std::shared_ptr<arm_api2_msgs::srv::AddGraspedObject::Response> res)
{
    if (!m_moveGroupPtr || !m_planningSceneInterface) {
        res->success = false;
        RCLCPP_WARN(this->get_logger(), "add_grasped_object: MoveGroup or PlanningSceneInterface not ready");
        return;
    }
    if (!waitForMoveGroupExecutionIdle("add_grasped_object")) {
        res->success = false;
        RCLCPP_WARN(this->get_logger(), "add_grasped_object: timeout waiting for motion — planning scene not updated");
        return;
    }

    std::lock_guard<std::mutex> lock(move_group_mutex_);

    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = req->attach_object.link_name;
    attached_object.object = req->grasped_object;
    attached_object.touch_links = req->attach_object.touch_links;
    const bool detach = req->grasped_object.operation == moveit_msgs::msg::CollisionObject::REMOVE;

    const bool applied = m_planningSceneInterface->applyAttachedCollisionObject(attached_object);

    /* removeCollisionObjects() SIGSEGVs here on Humble when no world entry exists. applyCollisionObjects
     * REMOVE is the supported diff path to drop a lingering world primitive (RViz green) after detach. */
    if (detach && applied && !req->grasped_object.id.empty()) {
        moveit_msgs::msg::CollisionObject co_rm;
        co_rm.header = req->grasped_object.header;
        if (co_rm.header.frame_id.empty()) {
            co_rm.header.frame_id = PLANNING_FRAME;
            co_rm.header.stamp = this->now();
        }
        co_rm.id = req->grasped_object.id;
        co_rm.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        try {
            m_planningSceneInterface->applyCollisionObjects(std::vector<moveit_msgs::msg::CollisionObject>{co_rm});
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "applyCollisionObjects(REMOVE) for id '%s' failed: %s",
                        req->grasped_object.id.c_str(), e.what());
        }
    }

    res->success = detach ? true : applied;
    if (!res->success && !detach) {
        RCLCPP_WARN(this->get_logger(), "applyAttachedCollisionObject (ADD) failed for id '%s'",
                    req->grasped_object.id.c_str());
    } else if (detach) {
        RCLCPP_INFO(this->get_logger(), "Detached collision id '%s' (attached apply=%s)",
                    req->grasped_object.id.c_str(), applied ? "ok" : "no-op");
    } else {
        RCLCPP_INFO(this->get_logger(), "Attached collision object id '%s'", req->grasped_object.id.c_str());
    }
}

void m2SimpleIface::set_path_constraints_cb(const std::shared_ptr<arm_api2_msgs::srv::SetPathConstraints::Request> req,
                                            const std::shared_ptr<arm_api2_msgs::srv::SetPathConstraints::Response> res)
{
    if (req->joint_constraints.empty()) {
        m_path_constraints_.joint_constraints.clear();
        res->success = true;
        RCLCPP_INFO(this->get_logger(), "Path constraints cleared (empty request)");
        return;
    }

    moveit_msgs::msg::Constraints constraints;
    moveit::core::RobotStatePtr state;
    try {
        state = snapshotRobotStateFromJoints();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "SetPathConstraints: robot state snapshot failed: %s", e.what());
        res->success = false;
        return;
    }

    for (const auto& jc_in : req->joint_constraints) {
        moveit_msgs::msg::JointConstraint jc;
        jc.joint_name = jc_in.joint_name;
        jc.position = jc_in.position;
        jc.tolerance_above = jc_in.tolerance_above;
        jc.tolerance_below = jc_in.tolerance_below;
        jc.weight = (jc_in.weight > 0.0) ? jc_in.weight : 1.0;

        if (std::isnan(jc.position) && state) {
            try {
                moveit::core::RobotModelConstPtr model = state->getRobotModel();
                if (model) {
                    int idx = model->getVariableIndex(jc.joint_name);
                    if (idx >= 0) {
                        jc.position = state->getVariablePosition(idx);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "SetPathConstraints: failed to fill NaN for %s: %s",
                            jc.joint_name.c_str(), e.what());
            }
        }
        constraints.joint_constraints.push_back(jc);
    }

    m_path_constraints_ = constraints;
    res->success = true;
    RCLCPP_INFO(this->get_logger(), "Path constraints set for %zu joint(s)", constraints.joint_constraints.size());
}

void m2SimpleIface::clear_path_constraints_cb(const std::shared_ptr<arm_api2_msgs::srv::ClearPathConstraints::Request> req,
                                             const std::shared_ptr<arm_api2_msgs::srv::ClearPathConstraints::Response> res)
{
    if (!waitForMoveGroupExecutionIdle("clear_path_constraints")) {
        res->success = false;
        RCLCPP_WARN(this->get_logger(), "clear_path_constraints: motion running or wait timeout — not applied");
        return;
    }
    m_path_constraints_.joint_constraints.clear();
    if (m_moveGroupPtr) {
        std::lock_guard<std::mutex> lock(move_group_mutex_);
        m_moveGroupPtr->clearPathConstraints();
    }
    res->success = true;
    RCLCPP_INFO(this->get_logger(), "Path constraints cleared");
}

void m2SimpleIface::change_state_cb(const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req, 
                                    const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res)
{
    auto itr = std::find(std::begin(stateNames), std::end(stateNames), req->state); 
    
    if ( itr != std::end(stateNames))
    {
        int wantedIndex_ = std::distance(stateNames, itr); 
        robotState.store(static_cast<state>(wantedIndex_));
        RCLCPP_INFO_STREAM(this->get_logger(), "Switching state!");
        res->success = true;  
    }else{
        RCLCPP_INFO_STREAM(this->get_logger(), "Failed switching to state " << req->state); 
        res->success = false; 
    } 
}

bool m2SimpleIface::setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs)
{
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setMoveGroup ENTER group=%s moveNs=%s", groupName.c_str(), moveNs.c_str());
    if (moveNs == "null") moveNs = "";

    std::string robot_desc_param = ROBOT_DESC;
    if (!moveNs.empty()) {
        robot_desc_param = "robot_description_" + moveNs;
    }

    std::string move_group_ns_for_options = moveNs;
    if (!moveNs.empty() && moveNs[0] != '/') {
        move_group_ns_for_options = "/" + moveNs;
    }

    constexpr int WAIT_FOR_SERVERS_SEC = 30;
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Creating MoveGroupInterface (robot_desc_param=%s, move_group_ns=%s, wait=%ds)...",
                robot_desc_param.c_str(), move_group_ns_for_options.c_str(), WAIT_FOR_SERVERS_SEC);

    try {
        m_moveGroupPtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            nodePtr,
            moveit::planning_interface::MoveGroupInterface::Options(groupName, robot_desc_param, move_group_ns_for_options),
            nullptr,
            rclcpp::Duration::from_seconds(WAIT_FOR_SERVERS_SEC));
        RCLCPP_INFO(this->get_logger(), "[DEBUG] MoveGroupInterface created.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[DEBUG] MoveGroupInterface EXCEPTION: %s", e.what());
        return false;
    }

    m_moveGroupPtr->setEndEffectorLink(EE_LINK_NAME);
    m_moveGroupPtr->setPoseReferenceFrame(PLANNING_FRAME);
    m_moveGroupPtr->setGoalPositionTolerance(0.0000001);
    m_moveGroupPtr->setNumPlanningAttempts(1);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Calling startStateMonitor()...");
    m_moveGroupPtr->startStateMonitor();
    RCLCPP_INFO(this->get_logger(), "[DEBUG] startStateMonitor() returned.");

    RCLCPP_INFO(this->get_logger(),
                "[DEBUG] Move group ready — spin iface + moveit_ros_node() on one SingleThreadedExecutor (non-blocking gate)");
    return true;
}

/* This is not neccessary*/
bool m2SimpleIface::setRobotModel(rclcpp::Node::SharedPtr nodePtr, const std::string& robot_desc_param)
{
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setRobotModel ENTER (param=%s) - creating RobotModelLoader...", robot_desc_param.c_str());
    robot_model_loader::RobotModelLoader robot_model_loader(nodePtr, robot_desc_param);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] RobotModelLoader created, calling getModel()...");
    kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(this->get_logger(), "[DEBUG] getModel() returned");
    if (!kinematic_model) {
        RCLCPP_ERROR(this->get_logger(),
                     "RobotModelLoader returned null (check URDF/SRDF on MoveIt node).");
        return false;
    }
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    m_robotStatePtr = kinematic_state;
    m_robotStatePtr->setToDefaultValues();
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot model loaded! frame=" << kinematic_model->getModelFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setRobotModel DONE");
    return true;
}

bool m2SimpleIface::setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name)
{
    RCLCPP_INFO(this->get_logger(), "[DEBUG] setPlanningSceneMonitor ENTER name=%s", name.c_str());
    m_pSceneMonitorPtr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(nodePtr, name);

    std::string scene_topic = MOVE_GROUP_NS.empty() || MOVE_GROUP_NS == "null"
        ? "/monitored_planning_scene"
        : "/" + MOVE_GROUP_NS + "/monitored_planning_scene";
    m_pSceneMonitorPtr->startSceneMonitor(scene_topic);

    if (m_pSceneMonitorPtr->getPlanningScene())
    {
        m_pSceneMonitorPtr->startStateMonitor(JOINT_STATES);
        m_pSceneMonitorPtr->setPlanningScenePublishingFrequency(25);
        std::string publish_topic = MOVE_GROUP_NS.empty() || MOVE_GROUP_NS == "null"
            ? "/moveit_servo/publish_planning_scene"
            : "/" + MOVE_GROUP_NS + "/moveit_servo/publish_planning_scene";
        m_pSceneMonitorPtr->startPublishingPlanningScene(
            planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, publish_topic);
        m_pSceneMonitorPtr->startSceneMonitor();
        m_pSceneMonitorPtr->providePlanningSceneService();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Planning scene not configured!");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "[DEBUG] setPlanningSceneMonitor DONE");
    return true;
}

moveit::core::RobotStatePtr m2SimpleIface::snapshotRobotStateFromJoints()
{
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    if (!m_robotStatePtr || !kinematic_model) {
        return nullptr;
    }
    return std::make_shared<moveit::core::RobotState>(*m_robotStatePtr);
}

bool m2SimpleIface::waitForMoveGroupExecutionIdle(const char* caller_reason, double timeout_sec)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (execute_in_flight_.load()) {
        if (std::chrono::steady_clock::now() > deadline) {
            RCLCPP_ERROR(this->get_logger(),
                         "%s: timeout after %.1fs waiting for trajectory execution to finish before planning scene update",
                         caller_reason, timeout_sec);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
}

bool m2SimpleIface::tryCompletePreviousExecution()
{
    constexpr double JOINT_TOLERANCE = 0.008;
    constexpr int SETTLE_TICKS_REQUIRED = 2;
    /* Long joint moves at low vel/acc scaling can exceed 10s; forcing the gate open while MoveGroup is
     * still executing invites a second asyncExecute and SEGVs on Humble. */
    constexpr double TIMEOUT_SEC = 120.0;

    if (!execute_in_flight_.load()) {
        previous_exec_settle_ticks_ = 0;
        execute_in_flight_deadline_valid_ = false;
        return true;
    }
    if (m_last_trajectory_final_positions_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "execute_in_flight without stored goal joints — clearing gate");
        postpone_next_moveit_attempt_ = true;
        m_last_trajectory_joint_names_.clear();
        execute_in_flight_.store(false);
        execute_in_flight_deadline_valid_ = false;
        previous_exec_settle_ticks_ = 0;
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (execute_in_flight_deadline_valid_ &&
        std::chrono::duration<double>(now - execute_in_flight_deadline_start_).count() > TIMEOUT_SEC) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Previous async trajectory did not settle within %.1fs; cancelling execution before allowing a new execute "
            "(overlapping asyncExecute SIGSEGVs on some MoveIt builds)",
            TIMEOUT_SEC);
        postpone_next_moveit_attempt_ = true;
        try {
            std::lock_guard<std::mutex> lock(move_group_mutex_);
            if (m_moveGroupPtr) {
                m_moveGroupPtr->stop();
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "move_group stop() after settle timeout: %s", e.what());
        }
        m_last_trajectory_joint_names_.clear();
        m_last_trajectory_final_positions_.clear();
        execute_in_flight_.store(false);
        execute_in_flight_deadline_valid_ = false;
        previous_exec_settle_ticks_ = 0;
        return true;
    }

    moveit::core::RobotStatePtr state = snapshotRobotStateFromJoints();
    if (!state) {
        return false;
    }

    moveit::core::RobotModelConstPtr rm = state->getRobotModel();
    if (!rm) {
        return false;
    }

    bool all_close = true;
    for (size_t j = 0; j < m_last_trajectory_joint_names_.size() && j < m_last_trajectory_final_positions_.size(); ++j) {
        const std::string& jn = m_last_trajectory_joint_names_[j];
        if (!rm->hasJointModel(jn)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Settle check: trajectory joint \"%s\" not in robot model — cannot match; waiting or timeout",
                jn.c_str());
            all_close = false;
            break;
        }
        double current = state->getVariablePosition(jn);
        double diff = std::abs(current - m_last_trajectory_final_positions_[j]);
        if (diff > JOINT_TOLERANCE) {
            all_close = false;
            break;
        }
    }
    if (!all_close) {
        previous_exec_settle_ticks_ = 0;
        return false;
    }

    previous_exec_settle_ticks_++;
    if (previous_exec_settle_ticks_ < SETTLE_TICKS_REQUIRED) {
        return false;
    }

    m_last_trajectory_joint_names_.clear();
    m_last_trajectory_final_positions_.clear();
    execute_in_flight_.store(false);
    execute_in_flight_deadline_valid_ = false;
    previous_exec_settle_ticks_ = 0;
    return true;
}

bool m2SimpleIface::readyForNewMoveItPlanOrExecute()
{
    if (!tryCompletePreviousExecution()) {
        return false;
    }
    if (postpone_next_moveit_attempt_) {
        postpone_next_moveit_attempt_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "Deferring MoveIt plan/execute one cycle after trajectory gate reset; "
                    "will retry same command on next timer tick.");
        return false;
    }
    return true;
}

bool m2SimpleIface::execMove(bool async=false)
{
    geometry_msgs::msg::PoseStamped cmd_in;
    {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        cmd_in = m_currPoseCmd;
    }
    geometry_msgs::msg::PoseStamped cmdPose_ = utils::normalizeOrientation(cmd_in);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "poseTarget is: "
                           << cmdPose_.pose.position.x << " " << cmdPose_.pose.position.y << " "
                           << cmdPose_.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Executing commanded path!");
    const bool attempted = execPlan(async, cmdPose_.pose);
    if (attempted) {
        {
            std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
            m_oldPoseCmd = cmdPose_;
        }
    }
    return attempted;
}

bool m2SimpleIface::execPlan(bool async, const geometry_msgs::msg::Pose& goal_pose)
{
    if (!readyForNewMoveItPlanOrExecute()) {
        return false;
    }

    moveit::core::RobotStatePtr start_state = snapshotRobotStateFromJoints();
    if (!start_state) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Cannot plan: joint state snapshot unavailable");
        return false;
    }
    start_state->update();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    /* PILZ + MoveGroup::plan() on JOINT_TRAJ_CTL has reproducible SEGVs (LIN path, sometimes PTP) on
     * some Humble builds. Joint-space pose goals are planned with OMPL; pipeline/planner IDs are restored
     * after plan() so Cartesian segments can keep using pilz_* via CART_TRAJ_CTL / set_planner. */
    const bool pilz_joint_plan_workaround = (current_planner_id_.find("pilz") != std::string::npos);

    std::shared_ptr<moveit_msgs::msg::RobotTrajectory> traj_to_run;
    bool will_sync_execute = false;

    try {
        std::lock_guard<std::mutex> lock(move_group_mutex_);
        if (!m_moveGroupPtr) {
            return false;
        }
        m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
        m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
        m_moveGroupPtr->clearPoseTargets();
        m_moveGroupPtr->setPoseTarget(goal_pose, EE_LINK_NAME);
        m_moveGroupPtr->setStartState(*start_state);
        if (pilz_joint_plan_workaround) {
            m_moveGroupPtr->setPlanningPipelineId("ompl");
            m_moveGroupPtr->setPlannerId("RRTConnect");
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "JOINT_TRAJ_CTL: using OMPL/RRTConnect instead of Pilz for this plan() "
                "(pilz segmentation-fault workaround; use CART_TRAJ_CTL for pilz LIN/CIRC).");
        }
        success = (m_moveGroupPtr->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (pilz_joint_plan_workaround) {
            m_moveGroupPtr->setPlanningPipelineId(current_planner_id_);
            m_moveGroupPtr->setPlannerId(current_planner_type_);
        }

        /* Plan under lock; mark in-flight before unlock so nothing mutates MoveGroup between plan and
         * execute. asyncExecute + MultiThreadedExecutor reproduced SIGSEGV right after Execute accepted
         * on some stacks (e.g. TF time jump); use synchronous execute() like Cartesian. */
        if (success) {
            m_async_plan_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(plan);
            traj_to_run =
                std::make_shared<moveit_msgs::msg::RobotTrajectory>(m_async_plan_ptr->trajectory_);
            m_async_trajectory_ptr = traj_to_run;
            const auto& jt = traj_to_run->joint_trajectory;
            if (jt.points.empty() || jt.joint_names.empty() ||
                jt.points.back().positions.size() != jt.joint_names.size()) {
                RCLCPP_WARN(this->get_logger(), "execPlan: plan has invalid joint trajectory, skipping execute");
                success = false;
            } else {
                execute_in_flight_.store(true);
                execute_in_flight_deadline_valid_ = false;
                m_last_trajectory_joint_names_ = jt.joint_names;
                m_last_trajectory_final_positions_ = jt.points.back().positions;
                will_sync_execute = true;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    } catch (const std::exception& e) {
        if (pilz_joint_plan_workaround) {
            std::lock_guard<std::mutex> lock(move_group_mutex_);
            if (m_moveGroupPtr) {
                m_moveGroupPtr->setPlanningPipelineId(current_planner_id_);
                m_moveGroupPtr->setPlannerId(current_planner_type_);
            }
        }
        RCLCPP_ERROR(this->get_logger(), "plan()/execute threw: %s", e.what());
        execute_in_flight_.store(false);
        m_last_trajectory_joint_names_.clear();
        m_last_trajectory_final_positions_.clear();
        return true;
    }

    if (will_sync_execute && traj_to_run && m_moveGroupPtr) {
        moveit::core::MoveItErrorCode ec = moveit::core::MoveItErrorCode::FAILURE;
        try {
            ec = m_moveGroupPtr->execute(*traj_to_run);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "execPlan execute: %s", e.what());
        }
        if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "execPlan: execute failed with code %d", static_cast<int>(ec.val));
        }
        execute_in_flight_.store(false);
        execute_in_flight_deadline_valid_ = false;
        m_last_trajectory_joint_names_.clear();
        m_last_trajectory_final_positions_.clear();
    }

    return true;
}

bool m2SimpleIface::planExecCartesian(bool async=false)
{
    if (!readyForNewMoveItPlanOrExecute()) {
        return false;
    }
    getArmState();

    moveit::core::RobotStatePtr cart_start_state = snapshotRobotStateFromJoints();
    if (!cart_start_state) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Cannot plan Cartesian segment: joint state snapshot unavailable");
        return false;
    }
    cart_start_state->update();

    geometry_msgs::msg::Pose cmd_pose;
    {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        cmd_pose = m_currPoseCmd.pose;
    }
    std::vector<geometry_msgs::msg::Pose> cartesianWaypoints =
        utils::createCartesianWaypoints(m_currPoseState.pose, cmd_pose, NUM_CART_PTS);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double jumpThr = 0.0;
    double eefStep = 0.02;
    double fraction = 0.0;
    const bool had_path_constraints = !m_path_constraints_.joint_constraints.empty();
    std::shared_ptr<moveit_msgs::msg::RobotTrajectory> traj_to_run;
    bool will_sync_execute = false;

    try {
        std::lock_guard<std::mutex> lock(move_group_mutex_);
        if (!m_moveGroupPtr) {
            RCLCPP_WARN(this->get_logger(), "planExecCartesian: MoveGroup not initialized");
            if (had_path_constraints) {
                m_path_constraints_.joint_constraints.clear();
            }
            return false;
        }
        m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
        m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
        m_moveGroupPtr->setStartState(*cart_start_state);

        if (had_path_constraints) {
            moveit_msgs::msg::MoveItErrorCodes err;
            fraction = m_moveGroupPtr->computeCartesianPath(
                cartesianWaypoints, eefStep, jumpThr, trajectory,
                m_path_constraints_, true, &err);
            m_moveGroupPtr->clearPathConstraints();
        } else {
            fraction = m_moveGroupPtr->computeCartesianPath(
                cartesianWaypoints, eefStep, jumpThr, trajectory);
        }

        if (fraction < 0.999) {
            RCLCPP_WARN(
                this->get_logger(),
                "planExecCartesian: refusing partial Cartesian path (fraction=%.3f); likely collision or IK failure",
                fraction);
            return false;
        }

        /* Compute stays under lock; mark execution in-flight before unlocking so no other callback can
         * change pipeline/constraints between compute and execute. Use synchronous execute() for Cartesian:
         * overlapping asyncExecute with OMPL joint motion + Pilz has reproduced SIGSEGV on Humble. */
        if (fraction > 0.0) {
            const auto& jt = trajectory.joint_trajectory;
            if (jt.points.empty() || jt.joint_names.empty() ||
                jt.points.back().positions.size() != jt.joint_names.size()) {
                RCLCPP_WARN(this->get_logger(),
                            "planExecCartesian: invalid joint trajectory (fraction=%.3f), skipping execute", fraction);
            } else {
                m_async_trajectory_ptr =
                    std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
                traj_to_run = m_async_trajectory_ptr;
                execute_in_flight_.store(true);
                execute_in_flight_deadline_valid_ = false;
                m_last_trajectory_joint_names_ = jt.joint_names;
                m_last_trajectory_final_positions_ = jt.points.back().positions;
                will_sync_execute = true;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "planExecCartesian: %s", e.what());
        if (had_path_constraints) {
            m_path_constraints_.joint_constraints.clear();
        }
        return false;
    }

    if (had_path_constraints) {
        m_path_constraints_.joint_constraints.clear();
    }

    if (will_sync_execute && traj_to_run && m_moveGroupPtr) {
        moveit::core::MoveItErrorCode ec = moveit::core::MoveItErrorCode::FAILURE;
        try {
            ec = m_moveGroupPtr->execute(*traj_to_run);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "planExecCartesian execute: %s", e.what());
        }
        if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "planExecCartesian: execute failed with code %d",
                        static_cast<int>(ec.val));
        }
        execute_in_flight_.store(false);
        execute_in_flight_deadline_valid_ = false;
        m_last_trajectory_joint_names_.clear();
        m_last_trajectory_final_positions_.clear();
    }

    if (fraction <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Cartesian path computation failed (fraction=%.2f)", fraction);
    }
    {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        m_oldPoseCmd = m_currPoseCmd;
    }
    return true;
}

bool m2SimpleIface::execCartesian(bool async=false)
{
    if (!readyForNewMoveItPlanOrExecute()) {
        return false;
    }
    getArmState();

    moveit::core::RobotStatePtr cart_start_state = snapshotRobotStateFromJoints();
    if (!cart_start_state) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Cannot exec Cartesian waypoints: joint state snapshot unavailable");
        return false;
    }
    cart_start_state->update();

    std::vector<geometry_msgs::msg::Pose> waypoints_copy;
    {
        std::lock_guard<std::mutex> lock(cart_waypoints_mutex_);
        waypoints_copy = m_cartesianWaypoints;
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    double jumpThr = 0.0;
    double eefStep = 0.02;
    std::shared_ptr<moveit_msgs::msg::RobotTrajectory> traj_to_run;
    bool will_sync_execute = false;

    try {
        std::lock_guard<std::mutex> lock(move_group_mutex_);
        if (!m_moveGroupPtr) {
            RCLCPP_WARN(this->get_logger(), "execCartesian: MoveGroup not initialized");
            return false;
        }
        m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
        m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
        m_moveGroupPtr->setStartState(*cart_start_state);
        const double fraction =
            m_moveGroupPtr->computeCartesianPath(waypoints_copy, eefStep, jumpThr, trajectory);

        if (fraction < 0.999) {
            RCLCPP_WARN(
                this->get_logger(),
                "execCartesian: refusing partial Cartesian path (fraction=%.3f); likely collision or IK failure",
                fraction);
            return false;
        }
        const auto& jt = trajectory.joint_trajectory;
        if (jt.points.empty() || jt.joint_names.empty() ||
            jt.points.back().positions.size() != jt.joint_names.size()) {
            RCLCPP_WARN(this->get_logger(), "execCartesian: invalid joint trajectory, skipping execute");
            return false;
        }
        m_async_trajectory_ptr = std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
        traj_to_run = m_async_trajectory_ptr;
        execute_in_flight_.store(true);
        execute_in_flight_deadline_valid_ = false;
        m_last_trajectory_joint_names_ = jt.joint_names;
        m_last_trajectory_final_positions_ = jt.points.back().positions;
        will_sync_execute = true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "execCartesian: %s", e.what());
        return false;
    }

    if (will_sync_execute && traj_to_run && m_moveGroupPtr) {
        moveit::core::MoveItErrorCode ec = moveit::core::MoveItErrorCode::FAILURE;
        try {
            ec = m_moveGroupPtr->execute(*traj_to_run);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "execCartesian execute: %s", e.what());
        }
        if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "execCartesian: execute failed with code %d",
                        static_cast<int>(ec.val));
        }
        execute_in_flight_.store(false);
        execute_in_flight_deadline_valid_ = false;
        m_last_trajectory_joint_names_.clear();
        m_last_trajectory_final_positions_.clear();
    }

    {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        m_oldPoseCmd = m_currPoseCmd;
    }
    return true;
}

bool m2SimpleIface::execTrajectory(moveit_msgs::msg::RobotTrajectory trajectory, bool async=false)
{
    if (!readyForNewMoveItPlanOrExecute()) {
        return false;
    }
    try {
        std::lock_guard<std::mutex> lock(move_group_mutex_);
        if (!m_moveGroupPtr) {
            return false;
        }
        const auto& jt = trajectory.joint_trajectory;
        if (jt.points.empty() || jt.joint_names.empty() ||
            jt.points.back().positions.size() != jt.joint_names.size()) {
            RCLCPP_WARN(this->get_logger(), "execTrajectory: invalid joint trajectory, skipping execute");
            return false;
        }
        m_async_trajectory_ptr = std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
        const auto ec = m_moveGroupPtr->asyncExecute(*m_async_trajectory_ptr);
        if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "execTrajectory: asyncExecute failed with code %d",
                        static_cast<int>(ec.val));
            return false;
        }
        execute_in_flight_.store(true);
        execute_in_flight_deadline_valid_ = true;
        execute_in_flight_deadline_start_ = std::chrono::steady_clock::now();
        m_last_trajectory_joint_names_ = jt.joint_names;
        m_last_trajectory_final_positions_ = jt.points.back().positions;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "execTrajectory: %s", e.what());
        return false;
    }
    return true;
}

void m2SimpleIface::getArmState() 
{
    auto local = snapshotRobotStateFromJoints();
    if (!local) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to get current state");
        return;
    }

    local->update();
    if (!local->knowsFrameTransform(EE_LINK_NAME)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Robot state has no FK for EE link \"%s\"", EE_LINK_NAME.c_str());
        return;
    }

    try {
        Eigen::Isometry3d currentPose_ = local->getFrameTransform(EE_LINK_NAME);
        m_currPoseState = utils::convertIsometryToMsg(currentPose_);
        m_currPoseState.header.stamp = this->now();
        m_currPoseState.header.frame_id = PLANNING_FRAME;
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "getArmState: getFrameTransform failed: %s", e.what());
    }
}

bool m2SimpleIface::run()
{
    if(!nodeInit)       {RCLCPP_ERROR(this->get_logger(), "Node not fully initialized!"); return false;} 
    if(!moveGroupInit)  {RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized!"); return false;} 

    getArmState(); 
    pose_state_pub_->publish(m_currPoseState);
    robot_state_pub_->publish(utils::stateToMsg(static_cast<int>(robotState.load())));

    rclcpp::Clock steady_clock; 
    int LOG_STATE_TIMEOUT=10000; 

    const state rs = robotState.load();

    // STATE MACHINE
    if (rs == IDLE)
    {   
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT, "arm_api2 is in IDLE mode."); 
    }
    else{
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT, "arm_api2 is in " << stateNames[static_cast<int>(rs)] << " mode."); 
    }

    // Pause servo after leaving servo mode — must guard: change_state does not clear servoEntered.
    if (rs != SERVO_CTL && servoEntered && servoPtr) {
        servoPtr->setPaused(true);
        servoEntered = false;
    }

    if (rs == JOINT_TRAJ_CTL)
    {
       if (recivCmd.load()) {
           if (execMove(async)) {
               recivCmd.store(false);
           }
       }
    }

    if (rs == CART_TRAJ_CTL)
    {   
        // TODO: Beware if both are true at the same time, shouldn't occur, 
        if (recivCmd.load()) {
            if (planExecCartesian(async)) {
                recivCmd.store(false);
            }
        }

        if (recivTraj.load()) {
            if (execCartesian(async)) {
                recivTraj.store(false);
            }
        }
    }

    if (rs == SERVO_CTL)
    {
        if (!servoEntered)
        {
            if (!servoPtr) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                      "SERVO_CTL requested but servo is disabled (enable_servo:=false)");
                return true;
            }
            // Moveit servo status codes: https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/include/moveit_servo/utils/datatypes.hpp
            servoPtr->start();
            servoEntered = true;
        }
    }

    return true;     
}


