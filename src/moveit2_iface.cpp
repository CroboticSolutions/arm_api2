#include "arm_api2/moveit2_iface.hpp"

m2Iface::m2Iface(): Node("moveit2_iface")
{   
    // devel --> use demo from moveit2_tutorials

  
    auto node_ = std::make_shared<rclcpp::Node>(this->get_name(), 
                                               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Load config 
    // TODO: Load config path from param
    config = init_config("/root/ws_moveit2/src/arm_api2/config/franka.yaml"); 
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config!");

    // Load arm basically --> two important params
    PLANNING_GROUP = config["robot"]["arm_name"].as<std::string>(); 
    EE_LINK_NAME   = config["robot"]["ee_link_name"].as<std::string>(); 
    
    //robotDescLoaded = loadRobotDesc
    moveGroupInit = setMoveGroup(node_, PLANNING_GROUP); 

    // todo: fix this part
    /*robot_model_loader::RobotModelLoader robot_model_loader(node_);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());*/

    // robot Model Loader --> fix this part
    /* robot_model_loader::RobotModelLoader robot_model_loader(node_);    
    kinematic_model = new robot_model_loader.getModel();
    robot_state = new moveit::core::RobotState(kinematic_model);
    RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str()); */

    ns_ = this->get_namespace(); 	


    init_publishers(); 
    init_subscribers(); 

    // TODO: Add as reconfigurable param
    std::chrono::duration<double> SYSTEM_DT(0.05);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&m2Iface::run, this));

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

void m2Iface::pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    
    newPoseCmd.pose = msg->pose; 
    recivCmd = true; 
}

bool m2Iface::setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName)
{
    // set mGroupIface 
    m_moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(nodePtr, groupName);
    
    return true; 
}

bool m2Iface::setPlanningScene()
{

    return true; 
}

bool m2Iface::setRobotModel(rclcpp::Node::SharedPtr nodePtr)
{
    //robot_model_loader::RobotModelLoader robot_model_loader(&nodePtr); 
    //m_robotModelPtr = new robot_model_loader.getModel(); 
    return false; 
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
    if(!moveGroupInit){RCLCPP_ERROR(this->get_logger(), "Move group not initialized"); return false;} 

    if (recivCmd){

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

    // Clean execution of run method
    return true;     
}
