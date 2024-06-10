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

m2Iface::m2Iface(const rclcpp::NodeOptions &options)
    : Node("moveit2_iface", options), node_(std::make_shared<rclcpp::Node>("moveit2_iface_node")), 
     executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()) 
{   
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
    
    // Currently not used :) 
    ns_ = this->get_namespace(); 	
    init_publishers(); 
    init_subscribers(); 
    init_services(); 
    init_moveit(); 
    if (enable_servo) {servoPtr = init_servo();}; 

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node!"); 

    // Init anything for the old pose because it is non existent at the beggining
    m_oldPoseCmd.pose.position.x = 5.0; 
    nodeInit = true; 
}

YAML::Node m2Iface::init_config(std::string yaml_path)
{   
    return YAML::LoadFile(yaml_path);
}

void m2Iface::init_publishers()
{   
    auto pose_state_name = config["topic"]["pub"]["current_pose"]["name"].as<std::string>(); 
    pose_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + pose_state_name, 1); 
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized publishers!");
}

void m2Iface::init_subscribers()
{
    auto pose_cmd_name = config["topic"]["sub"]["cmd_pose"]["name"].as<std::string>(); 
    auto cart_traj_cmd_name = config["topic"]["sub"]["cmd_traj"]["name"].as<std::string>(); 
    pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + pose_cmd_name, 1, std::bind(&m2Iface::pose_cmd_cb, this, _1));
    ctraj_cmd_sub_ = this->create_subscription<arm_api2_msgs::msg::CartesianWaypoints>(ns_ + cart_traj_cmd_name, 1, std::bind(&m2Iface::cart_poses_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!"); 
}

void m2Iface::init_services()
{
    auto change_state_name = config["srv"]["change_robot_state"]["name"].as<std::string>(); 
    change_state_srv_ = this->create_service<arm_api2_msgs::srv::ChangeState>(ns_ + change_state_name, 
                                                                              std::bind(&m2Iface::change_state_cb, this, _1, _2)); 
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized services!"); 
}

void m2Iface::init_moveit()
{

    RCLCPP_INFO_STREAM(this->get_logger(), "robot_description: " << ROBOT_DESC); 
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_group: " << PLANNING_GROUP);
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

void m2Iface::pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    m_currPoseCmd.pose = msg->pose;
    // Compare with old command
    RCLCPP_INFO_STREAM(this->get_logger(), "Test1");
    if (!comparePose(m_currPoseCmd, m_oldPoseCmd)) recivCmd = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "recivCmd: " << recivCmd); 
    RCLCPP_INFO_STREAM(this->get_logger(), "Test2"); 
}

void m2Iface::cart_poses_cb(const arm_api2_msgs::msg::CartesianWaypoints::SharedPtr msg)
{
    m_cartesianWaypoints = msg->poses; 
    recivTraj = true; 
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

    // set move group stuff
    m_moveGroupPtr->setEndEffectorLink(EE_LINK_NAME); 
    m_moveGroupPtr->setPoseReferenceFrame(PLANNING_FRAME); 
    m_moveGroupPtr->startStateMonitor(); 
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

void m2Iface::execMove(bool async=false)
{   
    m_moveGroupPtr->setPoseTarget(m_currPoseCmd); 
    execPlan(async); 
    m_oldPoseCmd = m_currPoseCmd; 
    RCLCPP_INFO_STREAM(this->get_logger(), "Executing commanded path!"); 

}

void m2Iface::execPlan(bool async=false)
{

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (m_moveGroupPtr->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        if (async) {m_moveGroupPtr->asyncExecute(plan);}
        else {m_moveGroupPtr->execute(plan);};  
    }else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!"); 
    }
}

void m2Iface::planExecCartesian(bool async=false)
{   
    // TODO: Move this method to utils.cpp
    std::vector<geometry_msgs::msg::Pose> cartesianWaypoints = createCartesianWaypoints(m_currPoseState.pose, m_currPoseCmd.pose, NUM_CART_PTS); 
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

void m2Iface::execCartesian(bool async=false)
{   
     
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

void m2Iface::execTrajectory(moveit_msgs::msg::RobotTrajectory trajectory, bool async=false)
{
    if (async) {m_moveGroupPtr->asyncExecute(trajectory);}
    else{m_moveGroupPtr->execute(trajectory);}; 
}

void m2Iface::getArmState() 
{
    // get current ee pose
    m_currPoseState = m_moveGroupPtr->getCurrentPose(EE_LINK_NAME); 
    // current_state_monitor
    m_robotStatePtr = m_moveGroupPtr->getCurrentState();
    // by default timeout is 10 secs
}

// TODO: Move to utils
bool m2Iface::comparePosition(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2)
{   
    // Returns false if different positions
    bool x_cond = false;  bool y_cond = false;  bool z_cond = false; 
    double d = 0.01; 

    if (std::abs(p1.pose.position.x - p2.pose.position.x) < d) x_cond = true; 
    if (std::abs(p1.pose.position.y - p2.pose.position.y) < d) y_cond = true; 
    if (std::abs(p1.pose.position.z - p2.pose.position.z) < d) z_cond = true; 

    bool cond = x_cond && y_cond && z_cond;  

    return cond; 
}

// TODO: Move to utils
bool m2Iface::compareOrientation(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2)
{   
    // Returns false if different positions
    bool p_cond = false;  bool r_cond = false;  bool y_cond = false; 
    double d = 0.01; 

    geometry_msgs::msg::Quaternion q1c, q2c; 
    q1c.x = p1.pose.orientation.x; q2c.x = p2.pose.orientation.x; 
    q1c.y = p1.pose.orientation.y; q2c.y = p2.pose.orientation.y; 
    q1c.z = p1.pose.orientation.z; q2c.z = p2.pose.orientation.z; 
    q1c.w = p1.pose.orientation.w; q2c.w = p2.pose.orientation.w; 

    tf2::Quaternion q1(q1c.x, q1c.y, q1c.z, q1c.w); 
    tf2::Quaternion q2(q2c.x, q2c.y, q2c.z, q2c.w); 
    
    double r1, r2, pi1, pi2, y1, y2; 
    tf2::Matrix3x3(q1).getEulerYPR(y1, pi1, r1); 
    tf2::Matrix3x3(q2).getEulerYPR(y2, pi2, r2);

    d = 0.01; 
    if (std::abs(pi1 - pi2) < d) p_cond = true; 
    if (std::abs(r1 - r2) < d) r_cond = true; 
    if (std::abs(y1 - y2) < d) y_cond = true; 

    //TODO: Convert quat1 to roll & pitch & yaw 
    bool cond = p_cond && r_cond && y_cond; 

    return cond; 
}

bool m2Iface::comparePose(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2)
{
    bool position, orientation; 
    position = comparePosition(p1, p2); 
    orientation = compareOrientation(p1, p2); 
    return position && orientation; 
}

// TODO: move to utils
std::vector<geometry_msgs::msg::Pose> m2Iface::createCartesianWaypoints(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2, int n) 
{
    std::vector<geometry_msgs::msg::Pose> result;
    geometry_msgs::msg::Pose pose_;
    if (n <= 1) {
        result.push_back(p1);
        return result;
    }
    double dX = (p2.position.x - p1.position.x) / (n - 1);
    double dY = (p2.position.y - p1.position.y) / (n - 1); 
    double dZ = (p2.position.z - p1.position.z) / (n - 1); 
    // Set i == 1 because start point doesn't have to be included into waypoint list 
    // https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/
    for (int i = 1; i < n; ++i) {
        pose_.position.x = p1.position.x + i * dX; 
        pose_.position.y = p1.position.y + i * dY; 
        pose_.position.z = p1.position.z + i * dZ; 
        pose_.orientation.x = p1.orientation.x; 
        pose_.orientation.y = p1.orientation.y;
        pose_.orientation.z = p1.orientation.z;
        pose_.orientation.w = p1.orientation.w; 
        result.push_back(pose_);
    }

    return result;
}

bool m2Iface::run()
{
    if(!nodeInit)       {RCLCPP_ERROR(this->get_logger(), "Node not fully initialized!"); return false;} 
    if(!moveGroupInit)  {RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized!"); return false;} 

    getArmState(); 
    pose_state_pub_->publish(m_currPoseState);

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

        if (recivTraj){
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



