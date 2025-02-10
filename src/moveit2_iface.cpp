/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Crobotic Solutions d.o.o.
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
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : The core robot manipulator and MoveIt2! ROS 2 interfacing header class.
 */

#include "arm_api2/moveit2_iface.hpp"
#include <future>
#include <vector>

m2Iface::m2Iface(const rclcpp::NodeOptions &options)
    : Node("moveit2_iface", options), node_(std::make_shared<rclcpp::Node>("moveit2_iface_node")), 
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
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&m2Iface::run, this));

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
    WITH_PLANNER        = config["robot"]["with_planner"].as<bool>();
    max_vel_scaling_factor = config["robot"]["max_vel_scaling_factor"].as<float>();
    max_acc_scaling_factor = config["robot"]["max_acc_scaling_factor"].as<float>();
    
    // Currently not used :) 
    ns_ = this->get_namespace(); 	
    init_publishers(); 
    init_subscribers(); 
    init_services(); 
    init_moveit(); 
    init_actionservers();
    if (enable_servo) {servoPtr = init_servo();}; 

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
    auto pose_state_name = config["topic"]["pub"]["current_pose"]["name"].as<std::string>();
    auto robot_state_name = config["topic"]["pub"]["current_robot_state"]["name"].as<std::string>(); 
    pose_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + pose_state_name, 1);
    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>(ns_ + robot_state_name, 1); 
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized publishers!");
}

void m2Iface::init_subscribers()
{
    auto joint_states_name = config["topic"]["sub"]["joint_states"]["name"].as<std::string>();
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(ns_ + joint_states_name, 1, std::bind(&m2Iface::joint_state_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!"); 
}

void m2Iface::init_services()
{
    auto change_state_name = config["srv"]["change_robot_state"]["name"].as<std::string>(); 
    auto set_vel_acc_name = config["srv"]["set_vel_acc"]["name"].as<std::string>();
    auto set_eelink_name = config["srv"]["set_eelink"]["name"].as<std::string>();
    change_state_srv_ = this->create_service<arm_api2_msgs::srv::ChangeState>(ns_ + change_state_name, std::bind(&m2Iface::change_state_cb, this, _1, _2)); 
    set_vel_acc_srv_ = this->create_service<arm_api2_msgs::srv::SetVelAcc>(ns_ + set_vel_acc_name, std::bind(&m2Iface::set_vel_acc_cb, this, _1, _2));
    set_eelink_srv_ = this->create_service<arm_api2_msgs::srv::SetStringParam>(ns_ + set_eelink_name, std::bind(&m2Iface::set_eelink_cb, this, _1, _2));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized services!"); 
}

void m2Iface::init_actionservers()
{
    auto move_to_pose_name = config["action"]["move_to_pose"]["name"].as<std::string>();
    auto move_to_joint_name = config["action"]["move_to_joint"]["name"].as<std::string>();
    auto move_to_pose_path_name = config["action"]["move_to_pose_path"]["name"].as<std::string>();
    auto gripper_control_name = config["action"]["gripper_control"]["name"].as<std::string>();
    move_to_pose_as_ = rclcpp_action::create_server<arm_api2_msgs::action::MoveCartesian>(this,
                                                                                        ns_ + move_to_pose_name,
                                                                                        std::bind(&m2Iface::move_to_pose_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::move_to_pose_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::move_to_pose_accepted_cb, this, _1));
    move_to_joint_as_ = rclcpp_action::create_server<arm_api2_msgs::action::MoveJoint>(this,
                                                                                        ns_ + move_to_joint_name,
                                                                                        std::bind(&m2Iface::move_to_joint_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::move_to_joint_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::move_to_joint_accepted_cb, this, _1));
    move_to_pose_path_as_ = rclcpp_action::create_server<arm_api2_msgs::action::MoveCartesianPath>(this,
                                                                                        ns_ + move_to_pose_path_name,
                                                                                        std::bind(&m2Iface::move_to_pose_path_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::move_to_pose_path_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::move_to_pose_path_accepted_cb, this, _1));
    gripper_control_as_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(this,
                                                                                        ns_ + gripper_control_name,
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
    // MoveIt related things!
    moveGroupInit       = setMoveGroup(node_, PLANNING_GROUP, MOVE_GROUP_NS); 
    pSceneMonitorInit   = setPlanningSceneMonitor(node_, ROBOT_DESC);
    robotModelInit      = setRobotModel(node_);
}

// TODO: Try to replace with auto
std::unique_ptr<moveit_servo::Servo> m2Iface::init_servo()
{   
    auto nodeParameters = node_->get_node_parameters_interface(); 
    auto servoParams = moveit_servo::ServoParameters::makeServoParameters(node_); 
    RCLCPP_INFO_STREAM(this->get_logger(), "ee_frame_name: " << servoParams->ee_frame_name);  
    servoParams->get("moveit_servo", nodeParameters);


    //auto servoParamsPtr = std::make_shared<moveit_servo::ServoParameters>(std::move(servoParams));
    //auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_); 
    // Servo parameters need to bee constSharedPtr
    auto servo = std::make_unique<moveit_servo::Servo>(node_, servoParams, m_pSceneMonitorPtr); 
    RCLCPP_INFO(this->get_logger(), "Servo initialized!"); 
    return servo;
}

void m2Iface::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{   
    std::vector<std::string> jointNames = msg->name;
    std::vector<double> jointPositions = msg->position;
    if(robotModelInit) {m_robotStatePtr->setVariablePositions(jointNames, jointPositions);}; 

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

void m2Iface::change_state_cb(const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req, 
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

bool m2Iface::setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs)
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

    // velocity scaling
    m_moveGroupPtr->setMaxVelocityScalingFactor(0.05);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(0.05);
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
    const bool success = planWithPlanner(plan);
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning to joint space goal: " << (success ? "SUCCEEDED" : "FAILED"));
    
    if (success) {
        //addTimestampsToTrajectory(plan.trajectory_);
        //printTimestamps(plan.trajectory_);

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
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning Cartesian path!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Current pose is: " << m_currPoseState.pose.position.x << " " << m_currPoseState.pose.position.y << " " << m_currPoseState.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Target pose is: " << goalPose.pose.position.x << " " << goalPose.pose.position.y << " " << goalPose.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Creating Cartesian waypoints!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Number of waypoints: " << NUM_CART_PTS);

    m_moveGroupPtr->setPoseTarget(goalPose);
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success = planWithPlanner(plan);
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning to pose goal: " << (success ? "SUCCEEDED" : "FAILED"));

    if(success){
        //addTimestampsToTrajectory(plan.trajectory_);
        //printTimestamps(plan.trajectory_);

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
            printTimestamps(plan.trajectory_);
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

    if(success){
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
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(m_moveGroupPtr->getCurrentState()->getRobotModel(), PLANNING_GROUP);
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*m_moveGroupPtr->getCurrentState(), trajectory);
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt, max_vel_scaling_factor, max_acc_scaling_factor);
    RCLCPP_INFO_STREAM(this->get_logger(), "Computed time stamp " << (success ? "SUCCEEDED" : "FAILED"));
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);
} 

bool m2Iface::planWithPlanner(moveit::planning_interface::MoveGroupInterface::Plan &plan){
    // Planning priority:
    // 1. cuMotion planner from cuMotion (if have)
    // 2. LIN planner from pilz_industrial_motion_planner
    // 3. EST planner from ompl
    // 4. PRM planner from ompl
    // for EST and PRM create three plans and choose the best one
    //-----------------------------------------------------------------------------------------------
    m_moveGroupPtr->setPlanningPipelineId("isaac_ros_cumotion");
    m_moveGroupPtr->setPlannerId("cuMotion");
    std::list<moveit::planning_interface::MoveGroupInterface::Plan> all_plans;
    moveit::planning_interface::MoveGroupInterface::Plan cuMotion_plan;
    bool cuMotion_success = (m_moveGroupPtr->plan(cuMotion_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (cuMotion_success) {
        plan = cuMotion_plan;
        RCLCPP_INFO(this->get_logger(), "cuMotion planner succeeded. Immediate return.");
        return true;
    }

    std::vector<std::pair<std::string, std::string>> planners = {
        {"pilz_industrial_motion_planner", "LIN"},
        {"ompl", "EST"},
        {"ompl", "PRM"}
    };
    // Vector to store futures for parallel planning
    std::vector<std::future<std::optional<moveit::planning_interface::MoveGroupInterface::Plan>>> futures;
    std::atomic<bool> plan_found{false};    
    // Launch planning tasks in parallel
    for (const auto &planner : planners) {
        futures.emplace_back(std::async(std::launch::async, [this, planner, &plan_found]() -> std::optional<moveit::planning_interface::MoveGroupInterface::Plan> {
            if (plan_found.load()) return std::nullopt;
            m_moveGroupPtr->setPlanningPipelineId(planner.first);
            m_moveGroupPtr->setPlannerId(planner.second);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = static_cast<bool>(m_moveGroupPtr->plan(plan));
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Planner %s succeeded with %d points.", planner.second.c_str(),
                            int(plan.trajectory_.joint_trajectory.points.size()));
                return plan;
            } else {
                RCLCPP_WARN(this->get_logger(), "Planner %s failed.", planner.second.c_str());
                return std::nullopt;
            }
        }));
    }
    std::list<moveit::planning_interface::MoveGroupInterface::Plan> valid_plans;
    // Collect all results
    for (auto &future : futures) {
        try {
            auto result = future.get();  // Wait for the task to finish
            if (result.has_value()) {
                valid_plans.push_back(result.value());
                
                if(valid_plans.size() >= 2){
                   plan_found.store(true);
                   break;
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in planner: %s", e.what());
        }
    }

    if (valid_plans.empty()) {
        RCLCPP_ERROR(this->get_logger(), "All planners failed!");
        return false;
    }
    // Find the best plan from the list of plans
    auto best_plan = std::min_element(valid_plans.begin(), valid_plans.end(), [](auto const &a, auto const &b) {
        return a.trajectory_.joint_trajectory.points.size() < b.trajectory_.joint_trajectory.points.size();
    });

    plan = *best_plan;
    RCLCPP_INFO(this->get_logger(), "Best plan selected with %d points.", int(best_plan->trajectory_.joint_trajectory.points.size()));
    return true;
}

void m2Iface::getArmState() 
{   
    const moveit::core::JointModelGroup* joint_model_group = m_robotStatePtr->getJointModelGroup(PLANNING_GROUP);
    m_robotStatePtr->copyJointGroupPositions(joint_model_group, m_currJointPosition);
    // get current ee pose
    m_currPoseState = m_moveGroupPtr->getCurrentPose(EE_LINK_NAME); 
    // current_state_monitor
    m_robotStatePtr = m_moveGroupPtr->getCurrentState();
    // by default timeout is 10 secs
    m_robotStatePtr->update();
    
    Eigen::Isometry3d currentPose_ = m_moveGroupPtr->getCurrentState()->getFrameTransform(EE_LINK_NAME);
    m_currPoseState = utils::convertIsometryToMsg(currentPose_);
    auto frame_id = m_moveGroupPtr->getPlanningFrame().c_str();
    m_currPoseState.header.frame_id = frame_id;
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
    if (robotState != SERVO_CTL && servoEntered) {servoPtr->setPaused(true); servoEntered=false;} 

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

    if (robotState == SERVO_CTL)
    {   
        if (!servoEntered)
        {   
            // Moveit servo status codes: https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/include/moveit_servo/utils/datatypes.hpp
            servoPtr->start(); 
            servoEntered = true; 
        }
    }

    if(recivGripperCmd){
        auto goal = m_gripperControlGoalHandle_->get_goal();
        float position = goal->command.position;
        float effort = goal->command.max_effort;

        auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
        bool success = gripper.send_gripper_command(position, effort);

        if(success){
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



