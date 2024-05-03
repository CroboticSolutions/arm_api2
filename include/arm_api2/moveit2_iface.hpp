#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <string>

#include <yaml-cpp/yaml.h>

// include ROS
#include <rclcpp/rclcpp.hpp>


namespace arm_api2
{

class Moveit2_iface
{
private: 

    rclcpp::Node m_ros_node; 
    YAML::Node m_robot_config; 


public: 

    Moveit2_iface(rclcpp::Node node, YAML::Node config)
        : m_ros_node { }
        , m_robot_config { config }
        {}

    //bool setPlanningScene() const {return m_planningScene}; 
    //bool setMoveGroup() const {return m_moveGroup}; 
    //bool setPlanningScene


}; 

} //namespace arm_api2

// TODO: 
// - [] Class init (constructor)
// - [] Class destrcutor 
// - [] private vs public variables 
// - [] Init robot model 
// - [] Init planning scene 
// - [] Init move_group 

#endif