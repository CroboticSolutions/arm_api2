
/*******************************************************************************
*
 * Copyright (c) 2024, Crobotic Solutions d.o.o. (www.crobotics.tech)
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

/*      Title       : moveit2_iface.hpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : The core robot manipulator and MoveIt2! ROS 2 interfacing header class.
 */

#ifndef MOVEIT2_IFACE_HPP
#define MOVEIT2_IFACE_HPP

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <string>

//* yaml params
#include <yaml-cpp/yaml.h>

//* ros
#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

//* moveit
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

//* msgs
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arm_api2_msgs/msg/cartesian_waypoints.hpp"

//* srvs
#include "arm_api2_msgs/srv/change_state.hpp"
#include "std_srvs/srv/trigger.hpp"

// utils
#include "arm_api2/utils.hpp"

// For starters just include robotiq_gripper
// TODO: Think of a way to include different gripper based on the gripper type
#include "arm_api2/grippers/gripper.hpp"
#include "arm_api2/grippers/robotiq_gripper.hpp"

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

        /* node related stuff */
        rclcpp::Node::SharedPtr node_;
        rclcpp::Executor::SharedPtr executor_;
        std::thread executor_thread_;

        /* gripper */
        RobotiqGripper gripper; 

        /* arm_definition */ 
        std::string PLANNING_GROUP; 
        std::string EE_LINK_NAME;  
        std::string ROBOT_DESC; 
        std::string PLANNING_SCENE; 
        std::string PLANNING_FRAME; 
        std::string MOVE_GROUP_NS; 
        std::string JOINT_STATES; 
        int NUM_CART_PTS; 
        bool ENABLE_SERVO; 

        /* timers */
        rclcpp::TimerBase::SharedPtr                                        timer_;

        /* parameters */
        std::string                                                         config_path; 
        bool                                                                enable_servo; 
        float                                                               dt; 
        
        /* config_file */
        YAML::Node config; 

        /*config*/
        YAML::Node init_config(std::string yaml_path);

        /* init methods */
        void init_subscribers();
        void init_publishers(); 
        void init_services(); 
        void init_moveit(); 
        std::unique_ptr<moveit_servo::Servo> init_servo(); 
        
        /* subs */
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr        pose_cmd_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr           joint_state_sub_;
        rclcpp::Subscription<arm_api2_msgs::msg::CartesianWaypoints>::SharedPtr ctraj_cmd_sub_; 

        /* pubs */
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       pose_state_pub_;

        /* srvs */
        rclcpp::Service<arm_api2_msgs::srv::ChangeState>::SharedPtr              change_state_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                       open_gripper_srv_; 
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                       close_gripper_srv_; 

        /* topic callbacks */
        void pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void cart_poses_cb(const arm_api2_msgs::msg::CartesianWaypoints::SharedPtr msg); 
        void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
        
        /* srv callbacks*/
        void change_state_cb(const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req, 
                             const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res);
        void open_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                             const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
        void close_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                             const std::shared_ptr<std_srvs::srv::Trigger::Response> res); 

        bool run(); 

        /* setters */
        bool setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs); 
        bool setRobotModel(rclcpp::Node::SharedPtr nodePtr); 
        bool setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name);

        /* getters */
        void getArmState();  

        /* funcs */
        void execPlan(bool async); 
        void execMove(bool async);  
        void execCartesian(bool async); 
        void planExecCartesian(bool async); 
        void execTrajectory(moveit_msgs::msg::RobotTrajectory trajectory, bool async); 

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
        bool gripperInit        = false; 
        bool nodeInit           = false; 
        bool recivCmd           = false; 
        bool recivTraj          = false; 
        bool servoEntered       = false; 
        bool async              = false; 

        /* ros vars */
        geometry_msgs::msg::PoseStamped m_currPoseCmd; 
        geometry_msgs::msg::PoseStamped m_pubCurrPoseCmd; 
        geometry_msgs::msg::PoseStamped m_oldPoseCmd; 
        geometry_msgs::msg::PoseStamped m_currPoseState;
        sensor_msgs::msg::JointState    m_currJointState;  
        std::vector<geometry_msgs::msg::Pose> m_cartesianWaypoints; 
        

        moveit::planning_interface::MoveGroupInterfacePtr m_moveGroupPtr; 
        moveit::core::RobotStatePtr m_robotStatePtr;  
        moveit::core::RobotModelPtr kinematic_model; 
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> m_pSceneMonitorPtr; 
        std::unique_ptr<moveit_servo::Servo> servoPtr; 

}; 

#endif
