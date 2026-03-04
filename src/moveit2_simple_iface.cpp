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
#include <future>
#include <thread>

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
    : Node("moveit2_simple_iface", options), node_(createMoveitNode(this)),
     executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()), gripper(node_) 
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
    add_grasped_object_srv_ = this->create_service<arm_api2_msgs::srv::AddGraspedObject>(resolve_topic_name("add_grasped_object"), std::bind(&m2SimpleIface::add_grasped_object_cb, this, _1, _2));
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
    m_currPoseCmd.header.frame_id = PLANNING_FRAME;
    m_currPoseCmd.pose = msg->pose;
    recivCmd = true;  // Always accept new commands (enables retry when commander re-sends same pose)
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
    std::vector<std::string> jointNames = msg->name;
    std::vector<double> jointPositions = msg->position;
    if (robotModelInit) {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        if (m_robotStatePtr) m_robotStatePtr->setVariablePositions(jointNames, jointPositions);
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
    
    m_planningSceneInterface->addCollisionObjects(collision_objects);
    
    RCLCPP_INFO(this->get_logger(), "Added collision object '%s' to planning scene", req->id.c_str());
    res->success = true;
    res->message = "Collision object added successfully";
}

void m2SimpleIface::add_grasped_object_cb(const std::shared_ptr<arm_api2_msgs::srv::AddGraspedObject::Request> req,
                                          const std::shared_ptr<arm_api2_msgs::srv::AddGraspedObject::Response> res)
{
    moveit_msgs::msg::AttachedCollisionObject attached_object; 
    attached_object.link_name = req->attach_object.link_name;
    attached_object.object = req->grasped_object; 
    attached_object.touch_links = req->attach_object.touch_links;
    attached_object.object.operation = attached_object.object.ADD;
    m_planningSceneInterface->applyAttachedCollisionObject(attached_object);
    res->success = true;
    RCLCPP_INFO(this->get_logger(), "Attached collision object to the end effector.");
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
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Calling startStateMonitor()...");
    m_moveGroupPtr->startStateMonitor();
    RCLCPP_INFO(this->get_logger(), "[DEBUG] startStateMonitor() returned.");

    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Move group interface set up DONE");
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

void m2SimpleIface::execMove(bool async=false)
{   
    geometry_msgs::msg::PoseStamped cmdPose_ = utils::normalizeOrientation(m_currPoseCmd);    
    m_moveGroupPtr->clearPoseTargets(); 
    m_moveGroupPtr->setPoseTarget(cmdPose_.pose, EE_LINK_NAME); 
    RCLCPP_INFO_STREAM(this->get_logger(), "poseTarget is: " << cmdPose_.pose.position.x << " " << cmdPose_.pose.position.y << " " << cmdPose_.pose.position.z); 
    execPlan(async); 
    
    // Thread-safe update of old pose
    {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        m_oldPoseCmd = cmdPose_;
    }
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Executing commanded path!"); 

}

void m2SimpleIface::waitForPreviousExecution()
{
    if (m_last_trajectory_final_positions_.empty()) return;

    constexpr double JOINT_TOLERANCE = 0.02;  // rad (~1.1 deg)
    constexpr double POLL_INTERVAL_MS = 50.0;
    constexpr double TIMEOUT_SEC = 5.0;
    const int max_iters = static_cast<int>(TIMEOUT_SEC * 1000.0 / POLL_INTERVAL_MS);

    for (int i = 0; i < max_iters && rclcpp::ok(); ++i) {
        moveit::core::RobotStatePtr state = m_moveGroupPtr->getCurrentState(0.1);
        if (!state) continue;

        bool all_close = true;
        for (size_t j = 0; j < m_last_trajectory_joint_names_.size() && j < m_last_trajectory_final_positions_.size(); ++j) {
            double current = state->getVariablePosition(m_last_trajectory_joint_names_[j]);
            double diff = std::abs(current - m_last_trajectory_final_positions_[j]);
            if (diff > JOINT_TOLERANCE) { all_close = false; break; }
        }
        if (all_close) {
            m_last_trajectory_joint_names_.clear();
            m_last_trajectory_final_positions_.clear();
            return;
        }
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(POLL_INTERVAL_MS));
    }
    RCLCPP_WARN(this->get_logger(), "Timeout waiting for previous execution; proceeding anyway");
    m_last_trajectory_joint_names_.clear();
    m_last_trajectory_final_positions_.clear();
}

void m2SimpleIface::execPlan(bool async=false)
{
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (m_moveGroupPtr->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        // Wait for previous async execution before sending new one (prevents JOINT->CART race)
        waitForPreviousExecution();
        // Use asyncExecute for all joint plans: blocking execute() crashes intermittently
        // when switching JOINT->CART (MoveIt/controller race). asyncExecute avoids the
        // blocking wait; commander uses is_complete() to detect arrival.
        m_async_plan_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(plan);
        auto trajectory_copy = std::make_shared<moveit_msgs::msg::RobotTrajectory>(m_async_plan_ptr->trajectory_);
        m_moveGroupPtr->asyncExecute(*trajectory_copy);
        m_async_trajectory_ptr = trajectory_copy;
        // Store final positions for next waitForPreviousExecution
        const auto& jt = trajectory_copy->joint_trajectory;
        if (!jt.points.empty() && !jt.joint_names.empty() &&
            jt.points.back().positions.size() == jt.joint_names.size()) {
            m_last_trajectory_joint_names_ = jt.joint_names;
            m_last_trajectory_final_positions_ = jt.points.back().positions;
        }
    }else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!"); 
    }
}

void m2SimpleIface::planExecCartesian(bool async=false)
{   
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
    
    // TODO: Move this method to utils.cpp
    std::vector<geometry_msgs::msg::Pose> cartesianWaypoints = utils::createCartesianWaypoints(m_currPoseState.pose, m_currPoseCmd.pose, NUM_CART_PTS); 
    // TODO: create Cartesian plan, use as first point currentPose 4 now, and as end point use targetPoint 
    moveit_msgs::msg::RobotTrajectory trajectory;
    // TODO: Set as params that can be configured in YAML!
    double jumpThr = 0.0; 
    double eefStep = 0.02; 
    // plan Cartesian path
    m_moveGroupPtr->computeCartesianPath(cartesianWaypoints, eefStep, jumpThr, trajectory);
    execTrajectory(trajectory, async); 
    m_oldPoseCmd = m_currPoseCmd; 
}

void m2SimpleIface::execCartesian(bool async=false)
{   
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
    
    // TODO: create Cartesian plan, use as first point currentPose 4 now, and as end point use targetPoint 
    moveit_msgs::msg::RobotTrajectory trajectory;
    // TODO: Set as params that can be configured in YAML!
    double jumpThr = 0.0; 
    double eefStep = 0.02; 
    // plan Cartesian path
    m_moveGroupPtr->computeCartesianPath(m_cartesianWaypoints, eefStep, jumpThr, trajectory);
    execTrajectory(trajectory, async); 
    m_oldPoseCmd = m_currPoseCmd; 
}

void m2SimpleIface::execTrajectory(moveit_msgs::msg::RobotTrajectory trajectory, bool async=false)
{
    // Always use asyncExecute: blocking execute() crashes intermittently on state transitions
    // (JOINT<->CART). Commander uses is_complete() to detect arrival.
    m_async_trajectory_ptr = std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
    // Wait for previous async execution before sending new one (prevents JOINT->CART race)
    waitForPreviousExecution();
    m_moveGroupPtr->asyncExecute(*m_async_trajectory_ptr);
    // Store final positions for next waitForPreviousExecution
    const auto& jt = trajectory.joint_trajectory;
    if (!jt.points.empty() && !jt.joint_names.empty() &&
        jt.points.back().positions.size() == jt.joint_names.size()) {
        m_last_trajectory_joint_names_ = jt.joint_names;
        m_last_trajectory_final_positions_ = jt.points.back().positions;
    }
}

void m2SimpleIface::getArmState() 
{   
    moveit::core::RobotStatePtr fresh_state = m_moveGroupPtr->getCurrentState(0.1);
    if (!fresh_state) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to get current state");
        return;
    }
    
    fresh_state->update();
    Eigen::Isometry3d currentPose_ = fresh_state->getFrameTransform(EE_LINK_NAME);
    m_currPoseState = utils::convertIsometryToMsg(currentPose_);
    m_currPoseState.header.stamp = this->now();
    m_currPoseState.header.frame_id = PLANNING_FRAME;

    {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        m_robotStatePtr = fresh_state;
    }
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
    if (robotState != SERVO_CTL && servoEntered) {servoPtr->setPaused(true); servoEntered=false;} 

    if (robotState == JOINT_TRAJ_CTL)
    {
       if (recivCmd) {
           execMove(async);
           recivCmd = false;
       }
    }

    if (robotState == CART_TRAJ_CTL)
    {   
        // TODO: Beware if both are true at the same time, shouldn't occur, 
        if (recivCmd) {
            planExecCartesian(async);
            recivCmd = false;
        }

        if (recivTraj) {
            execCartesian(async);
            recivTraj = false;
        }
    }

    if (robotState == SERVO_CTL)
    {   
        if (!servoEntered)
        {   
            // Moveit servo status codes: https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/include/moveit_servo/utils/datatypes.hpp
            servoPtr->start(); 
            servoEntered = true; 
        }
    }

    return true;     
}



