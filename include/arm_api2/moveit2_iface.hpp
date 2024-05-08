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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

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
        //~m2Iface();

        /* namespace param, maybe redundant */ 
        std::string ns_; 

    private: 

        /* arm_definition */
        std::string PLANNING_GROUP; 
        std::string EE_LINK_NAME;  
        std::string ROBOT_DESC; 
        std::string PLANNING_SCENE; 
        std::string MOVE_GROUP_NS; 

        /* timers */
        rclcpp::TimerBase::SharedPtr                                        timer_;
        
        /* config_file */
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

        /* callbacks */
        void pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        bool run(); 

        /* setters */
        bool setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs); 
        bool setRobotModel(rclcpp::Node::SharedPtr nodePtr); 
        bool setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name);

        /* getters */
        void getArmState();  

        /* utils */
        bool comparePositions(geometry_msgs::msg::PoseStamped pose1, geometry_msgs::msg::PoseStamped pose2); 

        /* funcs */
        void executePlan(bool async); 
        void executeMove();  

        moveit::planning_interface::MoveGroupInterface *m_moveGroupPtr; 
        //moveit::planning_interface::PlanningSceneInterface *m_planningSceneInterfacePtr; 
        planning_scene_monitor::PlanningSceneMonitor *m_pSceneMonitorPtr; 
        moveit::core::RobotStatePtr m_robotStatePtr;  

        /* flags*/
        bool moveGroupInit;
        bool robotModelInit;  
        bool pSceneMonitorInit;
        bool nodeInit; 
        bool recivCmd; 

        /* ros vars */
        geometry_msgs::msg::PoseStamped newPoseCmd; 
        geometry_msgs::msg::PoseStamped oldPoseCmd; 
        geometry_msgs::msg::PoseStamped currPose; 

}; 

#endif

// MoveIt2 ROS wrappers for cpp
// https://moveit.picknik.ai/main/doc/examples/examples.html#movegroup-ros-wrappers-in-c
// TODO: 
// - [x] Class init (constructor)
// - [ ] Class destrcutor -- 
// - [x] private vs public variables --> everything node related (private) 
// - [x] Init move_group 
// - [ ] Add planning scene monitor
// - [ ] Add all func from arm_api
// - [ ] MoveIt cpp iface https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html
// - [ ] Init robot model 
// - [ ] Init planning scene monitor 
