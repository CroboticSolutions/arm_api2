#include "arm_api2/grippers/robotiq_gripper.hpp"

RobotiqGripper::RobotiqGripper(rclcpp::Node::SharedPtr nodePtr)
    : node_ptr_(nodePtr)
{
    RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "RobotiqGripper initialized");
    is_open_ = false;
}

RobotiqGripper::~RobotiqGripper()
{
}

void RobotiqGripper::open()
{
    // Add implementation to open the gripper
    is_open_ = true;
    RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Gripper opened");
}

void RobotiqGripper::close()
{
    // Add implementation to close the gripper
    is_open_ = false;
    RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Gripper closed");
}

bool RobotiqGripper::isOpen() const
{
    return is_open_;
}