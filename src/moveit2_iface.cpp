#include "arm_api2/moveit2_iface.hpp"

m2Iface::m2Iface(): Node("moveit2_iface")
{   
    ns_ = this->get_namespace(); 	
    // TODO: Load yaml path from param
    config = init_config("/root/ws_moveit2/src/arm_api2/config/franka.yaml"); 
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config!");
    init_publishers(); 
    init_subscribers(); 
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
    geometry_msgs::msg::PoseStamped pose_cmd; 
    pose_cmd.pose = msg->pose; 
    std::cout << "Reciv msg" << pose_cmd << std::endl; 
}
