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
    
    // Currently not used :) 
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

    // TODO: Change gripper based on the config file and the gripper types
    gripper = RobotiqGripper();
}

YAML::Node m2Iface::init_config(std::string yaml_path)
{   
    RCLCPP_INFO_STREAM(this->get_logger(), "Config yaml path is: " << yaml_path); 
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
    auto joint_states_name = config["topic"]["sub"]["joint_states"]["name"].as<std::string>();
    pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + pose_cmd_name, 1, std::bind(&m2Iface::pose_cmd_cb, this, _1));
    ctraj_cmd_sub_ = this->create_subscription<arm_api2_msgs::msg::CartesianWaypoints>(ns_ + cart_traj_cmd_name, 1, std::bind(&m2Iface::cart_poses_cb, this, _1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(ns_ + joint_states_name, 1, std::bind(&m2Iface::joint_state_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!"); 
}

void m2Iface::init_services()
{
    auto change_state_name = config["srv"]["change_robot_state"]["name"].as<std::string>(); 
    auto open_gripper_name = config["srv"]["open_gripper"]["name"].as<std::string>(); 
    auto close_gripper_name= config["srv"]["close_gripper"]["name"].as<std::string>();
    change_state_srv_ = this->create_service<arm_api2_msgs::srv::ChangeState>(ns_ + change_state_name, std::bind(&m2Iface::change_state_cb, this, _1, _2)); 
    open_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(ns_ + open_gripper_name, std::bind(&m2Iface::open_gripper_cb, this, _1, _2));
    close_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(ns_ + close_gripper_name, std::bind(&m2Iface::close_gripper_cb, this, _1, _2));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized services!"); 
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

void m2Iface::pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // hardcode this to planning frame to check if it works like that? 
    m_currPoseCmd.header.frame_id = PLANNING_FRAME; 
    m_currPoseCmd.pose = msg->pose;
    if (!utils::comparePose(m_currPoseCmd, m_oldPoseCmd)) recivCmd = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "recivCmd: " << recivCmd); 
}

void m2Iface::cart_poses_cb(const arm_api2_msgs::msg::CartesianWaypoints::SharedPtr msg)
{
    m_cartesianWaypoints = msg->poses; 
    recivTraj = true; 
}

void m2Iface::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{   
    std::vector<std::string> jointNames = msg->name;
    std::vector<double> jointPositions = msg->position;
    if(robotModelInit) {m_robotStatePtr->setVariablePositions(jointNames, jointPositions);}; 

}

void m2Iface::open_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                              const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    gripper.open();
}

void m2Iface::close_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                               const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    gripper.close(); 
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

void m2Iface::execMove(bool async=false)
{   
    m_currPoseCmd = utils::normalizeOrientation(m_currPoseCmd); 
    m_moveGroupPtr->clearPoseTargets(); 
    m_moveGroupPtr->setPoseTarget(m_currPoseCmd.pose, EE_LINK_NAME); 
    geometry_msgs::msg::PoseStamped poseTarget; 
    poseTarget = m_moveGroupPtr->getPoseTarget(); 
    RCLCPP_INFO_STREAM(this->get_logger(), "poseTarget is: " << poseTarget.pose.position.x << " " << poseTarget.pose.position.y << " " << poseTarget.pose.position.z); 
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
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning Cartesian path!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Current pose is: " << m_currPoseState.pose.position.x << " " << m_currPoseState.pose.position.y << " " << m_currPoseState.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Target pose is: " << m_currPoseCmd.pose.position.x << " " << m_currPoseCmd.pose.position.y << " " << m_currPoseCmd.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Creating Cartesian waypoints!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Number of waypoints: " << NUM_CART_PTS);
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
    const moveit::core::JointModelGroup* joint_model_group = m_robotStatePtr->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string>& joint_names = m_robotStatePtr->getVariableNames();
    std::vector<double> joint_values;
    m_robotStatePtr->copyJointGroupPositions(joint_model_group, joint_values);
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



