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

//* srvs
#include "arm_api2_msgs/srv/change_state.hpp"

#define stringify( name ) #name

// *std placeholders
using namespace std::chrono_literals;  
using std::placeholders::_1;
using std::placeholders::_2; 

class m2Iface: public rclcpp::Node
{

    public:

        m2Iface(const rclcpp::NodeOptions &options);  
        //~m2Iface();

        /* namespace param, maybe redundant */ 
        std::string ns_; 

    private: 

        rclcpp::Node::SharedPtr node_;
        rclcpp::Executor::SharedPtr executor_;
        std::thread executor_thread_;

        /* arm_definition */
        std::string PLANNING_GROUP; 
        std::string EE_LINK_NAME;  
        std::string ROBOT_DESC; 
        std::string PLANNING_SCENE; 
        std::string PLANNING_FRAME; 
        std::string MOVE_GROUP_NS; 

        /* timers */
        rclcpp::TimerBase::SharedPtr                                        timer_;

        /* parameters */
        std::string                                                         config_path; 
        
        /* config_file */
        YAML::Node config; 

        /*config*/
        YAML::Node init_config(std::string yaml_path);

        /* init methods */
        void init_subscribers();
        void init_publishers(); 
        void init_services(); 
        void init_moveit(); 

        /* subs */
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    pose_cmd_sub_;

        /* pubs */
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       pose_state_pub_;

        /* srvs */
        rclcpp::Service<arm_api2_msgs::srv::ChangeState>::SharedPtr              change_state_srv_; 

        /* callbacks */
        void pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void change_state_cb(const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req, 
                             const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res); 
        bool run(); 

        /* setters */
        bool setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs); 
        bool setRobotModel(rclcpp::Node::SharedPtr nodePtr); 
        bool setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name);

        /* getters */
        void getArmState();  

        /* utils */
        bool comparePositions(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2);  

        /* funcs */
        void executePlan(bool async); 
        void executeMove(bool async);  

        // Simple state machine 
        enum state{
            IDLE = 0, 
            JOINT_TRAJ_CTL = 1, 
            CART_TRAJ_CTL = 2, 
            SERVO_CTL = 3
        }; 
        
        // stateNames 
        const char* stateNames[4] = 
        {
            stringify (IDLE), 
            stringify (JOINT_TRAJ_CTL), 
            stringify (CART_TRAJ_CTL), 
            stringify (SERVO_CTL)
        }; 

        // robot state
        enum state robotState = IDLE; 



        /* flags*/
        bool moveGroupInit      = false;
        bool robotModelInit     = false;  
        bool pSceneMonitorInit  = false;
        bool nodeInit           = false; 
        bool recivCmd           = false; 

        /* ros vars */
        geometry_msgs::msg::PoseStamped m_currPoseCmd; 
        geometry_msgs::msg::PoseStamped m_pubCurrPoseCmd; 
        geometry_msgs::msg::PoseStamped m_oldPoseCmd; 
        geometry_msgs::msg::PoseStamped m_currPose; 

        moveit::planning_interface::MoveGroupInterfacePtr m_moveGroupPtr; 
        //moveit::planning_interface::PlanningSceneInterface *m_planningSceneInterfacePtr; 
        planning_scene_monitor::PlanningSceneMonitor *m_pSceneMonitorPtr; 
        moveit::core::RobotStatePtr m_robotStatePtr;  
        moveit::core::RobotModelPtr kinematic_model; 

}; 

#endif

// MoveIt2 ROS wrappers for cpp
// https://moveit.picknik.ai/main/doc/examples/examples.html#movegroup-ros-wrappers-in-c
// TODO: 
// - [x] Class init (constructor)
// - [ ] Class destrcutor -- 
// - [x] private vs public variables --> everything node related (private) 
// - [x] Init move_group 
// - [x] Add planning scene monitor
// - [ ] Add all func from arm_api
// - [x] MoveIt cpp iface https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html
// - [x] Init robot model 
// - [x] Init planning scene monitor 
// - [ ] Add arm modes 
// - [ ] Add Cartesian planning 
// - [ ] Test with a real robot arm
// - [ ] Add servoing 