#ifndef MOVEIT2_IFACE_HPP
#define MOVEIT2_IFACE_HPP
// Used instead of #pragma once because of the diff PC support

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <string>

//* yaml params
#include <yaml-cpp/yaml.h>

//* ros
#include <rclcpp/rclcpp.hpp>

//* moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

//* msgs
#include "geometry_msgs/msg/pose_stamped.hpp"

// *std placeholders
using namespace std::chrono_literals;  
using std::placeholders::_1;
using std::placeholders::_2; 

class m2Iface: public rclcpp::Node
{

    public:

        m2Iface();  
        //~Moveit2Iface();

        /* namespace param, maybe redundant */ 
        std::string ns_; 

    private: 

        // PLANNING_GROUP
        std::string PLANNING_GROUP;  

        // timers
        rclcpp::TimerBase::SharedPtr                                        timer_;

        /* flags*/
        bool moveGroupInit; 
        bool nodeInit; 
        
        YAML::Node config; 

        /*config*/
        YAML::Node init_config(std::string yaml_path);

        /* init methods */
        void init_subscribers();
        void init_publishers(); 

        /* subs */
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    pose_cmd_sub_;

        /* pubs */
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       pose_state_pub_;
        /* rclcpp::Publisher<arm_api2_msgs::msg::State>::SharedPtr             arm_state_pub_;*/ 

        /* callbacks */
        void pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg); 
        void run(); 

        /* setters */
        /* RobotModel */
        // const moveit::core::RobotModelPtr setRobotModel()
        bool setPlanningSceneIface();  
        bool setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName); 
        //bool setPlanningSceneMonitor() const {return m_planningSceneMonitor}; 

        // init kinematic model
        /*moveit::core::RobotModelPtr& kinematic_model; 
        moveit::core::RobotStatePtr robot_state; */
        moveit::planning_interface::MoveGroupInterface *m_moveGroupPtr; 
        


        /* getters */



}; 

#endif

// MoveIt2 ROS wrappers for cpp
// https://moveit.picknik.ai/main/doc/examples/examples.html#movegroup-ros-wrappers-in-c
// TODO: 
// - [x] Class init (constructor)
// - [] Class destrcutor -- 
// - [x] private vs public variables --> everything node related (private) 
// - [] MoveIt cpp iface https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html
// - [] Init robot model 
// - [] Init planning scene 
// - [] Init move_group 
