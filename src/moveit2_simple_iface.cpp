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

m2SimpleIface::m2SimpleIface(const rclcpp::NodeOptions &options)
    : Node("moveit2_simple_iface", options), node_(std::make_shared<rclcpp::Node>("moveit2_simple_iface_node")), 
     executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()), gripper(node_) 
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
    max_vel_scaling_factor = config["robot"]["max_vel_scaling_factor"].as<float>();
    max_acc_scaling_factor = config["robot"]["max_acc_scaling_factor"].as<float>();
    
    // Currently not used :) [ns]
    ns_ = this->get_namespace(); 	
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
    pose_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + pose_state_name, 1); 
    auto current_robot_state_name = config["topic"]["pub"]["current_robot_state"]["name"].as<std::string>(); 
    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>(ns_ + current_robot_state_name, 1);
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized publishers!");
}

void m2SimpleIface::init_subscribers()
{
    auto pose_cmd_name = config["topic"]["sub"]["cmd_pose"]["name"].as<std::string>(); 
    auto cart_traj_cmd_name = config["topic"]["sub"]["cmd_traj"]["name"].as<std::string>(); 
    auto joint_states_name = config["topic"]["sub"]["joint_states"]["name"].as<std::string>();
    pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + pose_cmd_name, 1, std::bind(&m2SimpleIface::pose_cmd_cb, this, _1));
    ctraj_cmd_sub_ = this->create_subscription<arm_api2_msgs::msg::CartesianWaypoints>(ns_ + cart_traj_cmd_name, 1, std::bind(&m2SimpleIface::cart_poses_cb, this, _1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(ns_ + joint_states_name, 1, std::bind(&m2SimpleIface::joint_state_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!"); 
}

void m2SimpleIface::init_services()
{
    auto change_state_name = config["srv"]["change_robot_state"]["name"].as<std::string>(); 
    auto set_vel_acc_name  = config["srv"]["set_vel_acc"]["name"].as<std::string>();
    auto set_planner_name  = config["srv"]["set_planner"]["name"].as<std::string>();
    auto open_gripper_name = config["srv"]["open_gripper"]["name"].as<std::string>(); 
    auto close_gripper_name= config["srv"]["close_gripper"]["name"].as<std::string>();
    change_state_srv_ = this->create_service<arm_api2_msgs::srv::ChangeState>(ns_ + change_state_name, std::bind(&m2SimpleIface::change_state_cb, this, _1, _2)); 
    set_vel_acc_srv_  = this->create_service<arm_api2_msgs::srv::SetVelAcc>(ns_ + set_vel_acc_name, std::bind(&m2SimpleIface::set_vel_acc_cb, this, _1, _2));
    set_planner_srv_  = this->create_service<arm_api2_msgs::srv::SetStringParam>(ns_ + set_planner_name, std::bind(&m2SimpleIface::set_planner_cb, this, _1, _2));
    open_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(ns_ + open_gripper_name, std::bind(&m2SimpleIface::open_gripper_cb, this, _1, _2));
    close_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(ns_ + close_gripper_name, std::bind(&m2SimpleIface::close_gripper_cb, this, _1, _2));
    add_collision_object_srv_ = this->create_service<arm_api2_msgs::srv::AddCollisionObject>(ns_ + "add_collision_object", std::bind(&m2SimpleIface::add_collision_object_cb, this, _1, _2));
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
   
    // hardcode this to planning frame to check if it works like that? 
    m_currPoseCmd.header.frame_id = PLANNING_FRAME; 
    m_currPoseCmd.pose = msg->pose;
    if (!utils::comparePose(m_currPoseCmd, m_oldPoseCmd)) recivCmd = true;
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
    if(robotModelInit) {m_robotStatePtr->setVariablePositions(jointNames, jointPositions);}; 

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
    
    m_planningSceneInterface.addCollisionObjects(collision_objects);
    
    RCLCPP_INFO(this->get_logger(), "Added collision object '%s' to planning scene", req->id.c_str());
    res->success = true;
    res->message = "Collision object added successfully";
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

void m2SimpleIface::execPlan(bool async=false)
{
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (m_moveGroupPtr->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        if (async) {
            // Store plan as member variable to keep it alive during async execution
            m_async_plan_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(plan);
            // Make a shared_ptr copy of the trajectory to ensure it stays alive
            auto trajectory_copy = std::make_shared<moveit_msgs::msg::RobotTrajectory>(m_async_plan_ptr->trajectory);
            RCLCPP_INFO(this->get_logger(), "Starting async execution, plan at: %p, trajectory at: %p", 
                        static_cast<void*>(m_async_plan_ptr.get()), static_cast<void*>(trajectory_copy.get()));
            m_moveGroupPtr->asyncExecute(*trajectory_copy);
            // Keep trajectory_copy alive by storing it
            m_async_trajectory_ptr = trajectory_copy;
        }
        else {
            m_moveGroupPtr->execute(plan);
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
    if (async) {
        // Store trajectory as member variable to keep it alive during async execution
        m_async_trajectory_ptr = std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
        RCLCPP_INFO(this->get_logger(), "Starting async trajectory execution, storing at: %p", static_cast<void*>(m_async_trajectory_ptr.get()));
        m_moveGroupPtr->asyncExecute(*m_async_trajectory_ptr);
        RCLCPP_INFO_STREAM(this->get_logger(), "Executing trajectory asynchronously!");
    }
    else{
        m_moveGroupPtr->execute(trajectory);
    }
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
    if (robotState != SERVO_CTL && servoEntered) {servoPtr->setCollisionChecking(false); servoEntered=false;} // New API: no setPaused 

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
            servoPtr->setCollisionChecking(true); // New API: no start(), servo works on-demand 
            servoEntered = true; 
        }
    }

    return true;     
}



