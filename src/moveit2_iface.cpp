#include "arm_api2/moveit2_iface.hpp"

m2Iface::m2Iface(): Node("moveit2_iface") 
{   
    node_ = std::make_shared<rclcpp::Node>(this->get_name(), 
                                               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // TODO: Load config path from param
    this->declare_parameter<std::string>("config_path", "/root/ws_moveit2/src/arm_api2/config/franka_demo.yaml");
    this->get_parameter("config_path", config_path);
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config!");

    // TODO: Add as reconfigurable param 
    std::chrono::duration<double> SYSTEM_DT(0.05);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&m2Iface::run, this));

    // Load arm basically --> two important params
    // Manual param specification --> https://github.com/moveit/moveit2_tutorials/blob/8eaef05bfbabde3f35910ad054a819d79e70d3fc/doc/tutorials/quickstart_in_rviz/launch/demo.launch.py#L105
    config              = init_config(config_path);  
    PLANNING_GROUP      = config["robot"]["arm_name"].as<std::string>(); 
    EE_LINK_NAME        = config["robot"]["ee_link_name"].as<std::string>();
    ROBOT_DESC          = config["robot"]["robot_desc"].as<std::string>();  
    PLANNING_SCENE      = config["robot"]["planning_scene"].as<std::string>(); 
    MOVE_GROUP_NS       = config["robot"]["move_group_ns"].as<std::string>(); 
    
    // Currently not used :) 
    ns_ = this->get_namespace(); 	
    init_publishers(); 
    init_subscribers(); 
    init_moveit(); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node!"); 


    // Init anything for the old pose because it is non existent at the beggining
    oldPoseCmd.pose.position.x = 5.0; 
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
    pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + pose_cmd_name, 1, std::bind(&m2Iface::pose_cmd_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!"); 
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
void m2Iface::pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    
    newPoseCmd.pose = msg->pose; 
    recivCmd = true; 
}

bool m2Iface::setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs)
{
    // check if moveNs is empty
    if (moveNs == "null") moveNs=""; 

    m_moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(nodePtr, groupName); 
    // set mGroupIface 
    /*
    m_moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(nodePtr, 
        moveit::planning_interface::MoveGroupInterface::Options(
            groupName,
            moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION,
            moveNs));*/
    RCLCPP_INFO_STREAM(this->get_logger(), "Move group interface set up!"); 
    
    return true; 
}

/* This is not neccessary*/
bool m2Iface::setRobotModel(rclcpp::Node::SharedPtr nodePtr)
{
  
    robot_model_loader::RobotModelLoader robot_model_loader(nodePtr);
    kinematic_model = robot_model_loader.getModel(); 
    //m_planningScenePtr = new planning_scene::PlanningScene(kinematic_model);
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot model loaded!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot model frame is: " << kinematic_model->getModelFrame().c_str());
    return true;
}

bool m2Iface::setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name)
{
    // https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
    // https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/planning_scene/src/planning_scene_tutorial.cpp
    m_pSceneMonitorPtr = new planning_scene_monitor::PlanningSceneMonitor(nodePtr, name); 
    m_pSceneMonitorPtr->startSceneMonitor(PLANNING_SCENE); 
    //TODO: Check what's difference between planning_Scene and planning_scene_monitor
    RCLCPP_INFO_STREAM(this->get_logger(), "Created planning scene monitor!");
    return true; 
}

void m2Iface::executePlan(bool async=false)
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

void m2Iface::executeMove()
{
    if (comparePositions(newPoseCmd, oldPoseCmd))
        {   
            auto steady_clock = rclcpp::Clock();
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), steady_clock, 2000, "Same pose commanded!"); 
        }else{
            // this is blocking call I think!
            m_moveGroupPtr->setPoseTarget(newPoseCmd); 
            executePlan(true); 
            recivCmd = false; 
            oldPoseCmd = std::move(newPoseCmd); 
            RCLCPP_INFO_STREAM(this->get_logger(), "Executing path!"); 
    }
}

void m2Iface::getArmState() 
{
  currPose = m_moveGroupPtr->getCurrentPose(EE_LINK_NAME); 
  // current_state_monitor
  m_robotStatePtr = m_moveGroupPtr->getCurrentState();
}

// TODO: Move to utils
bool m2Iface::comparePositions(geometry_msgs::msg::PoseStamped pose1, geometry_msgs::msg::PoseStamped pose2)
{
    bool x_cond = false;  bool y_cond = false;  bool z_cond = false; 
    double dist = 0.01; 

    if (std::abs(pose1.pose.position.x - pose2.pose.position.x) < dist) x_cond = true; 
    if (std::abs(pose1.pose.position.y - pose2.pose.position.y) < dist) y_cond = true; 
    if (std::abs(pose1.pose.position.z - pose2.pose.position.z) < dist) z_cond = true; 

    bool cond = x_cond && y_cond && z_cond;  

    return cond; 
}

bool m2Iface::run()
{
    if(!nodeInit){RCLCPP_ERROR(this->get_logger(), "Node not fully initialized!"); return false;} 
    if(!moveGroupInit) {RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized!"); return false;} 

    // Time is ok, uses sim time! --> in sim case, not in demo case (demo case doesn't publish clock)
    RCLCPP_INFO_STREAM(this->get_logger(),  "Running: " << node_->now().seconds());   

    if (recivCmd){
        // TODO: Wrap it further
       executeMove(); 
    }

    // Clean execution of run method
    return true;     
}
