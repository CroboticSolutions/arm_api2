#ifndef ROBOTIQ_GRIPPER_H
#define ROBOTIQ_GRIPPER_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "gripper.hpp"

class RobotiqGripper: public Gripper {

public:
    // Constructor
    RobotiqGripper(){
        isOpen = false;
    };
    ~RobotiqGripper(){};

    // Method to open the gripper
    void open(){
        //TODO: Implement correct topic/action call for starters
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Gripper opened.");
        isOpen = true;
    }; 

    // Method to close the gripper
    void close(){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Gripper closed.");
        isOpen = false; 
    };

private:
    bool isOpen;
    // TODO: Add ROS2 action client or topic publisher which is going to be used
};

#endif // ROBOTIQ_GRIPPER_H