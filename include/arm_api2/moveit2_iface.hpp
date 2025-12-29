
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

/*      Title       : arm2_action_iface.hpp
 *      Project     : arm_api2
 *      Created     : 08/06/2025
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
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

//* moveit
#include <moveit_servo/servo.hpp>
#include <moveit_servo/moveit_servo_lib_parameters.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <pluginlib/class_loader.hpp>

//* msgs
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arm_api2_msgs/msg/cartesian_waypoints.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/servo_status.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

//* srvs
#include "arm_api2_msgs/srv/change_state.hpp"
#include "arm_api2_msgs/srv/add_collision_object.hpp"
#include "arm_api2_msgs/srv/set_vel_acc.hpp"
#include "arm_api2_msgs/srv/set_string_param.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "arm_api2_msgs/action/move_cartesian.hpp"
#include "arm_api2_msgs/action/move_joint.hpp"
#include "arm_api2_msgs/action/move_cartesian_path.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

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
        bool WITH_PLANNER; 
        bool ENABLE_SERVO; 
        float INIT_VEL_SCALING; 
        float INIT_ACC_SCALING; 

        /* timers */
        rclcpp::TimerBase::SharedPtr                                        timer_;

        /* parameters */
        std::string                                                         config_path; 
        bool                                                                enable_servo; 
        bool                                                                eager_execution; 
        float                                                               dt;
        float                                                               max_vel_scaling_factor;
        float                                                               max_acc_scaling_factor;
        
        /* planner info */
        std::string current_planner_id_ = "pilz_industrial_motion_planner";
        std::string current_planner_type_ = "LIN";
        
        /* config_file */
        YAML::Node config;
        /*config*/
        YAML::Node init_config(std::string yaml_path);

        /* init methods */
        void init_subscribers();
        void init_publishers(); 
        void init_services();
        void init_actionservers();
        void init_moveit(); 
        std::unique_ptr<moveit_servo::Servo> init_servo(); 
        
        /* subs */
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr           joint_state_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr       servo_twist_sub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr     servo_trajectory_pub_;
        rclcpp::Publisher<moveit_msgs::msg::ServoStatus>::SharedPtr            servo_status_pub_;

        /* pubs */
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       pose_state_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 robot_state_pub_;

        /* srvs */
        rclcpp::Service<arm_api2_msgs::srv::ChangeState>::SharedPtr              change_state_srv_;
        rclcpp::Service<arm_api2_msgs::srv::SetVelAcc>::SharedPtr                set_vel_acc_srv_;
        rclcpp::Service<arm_api2_msgs::srv::SetStringParam>::SharedPtr           set_planner_srv_;
        rclcpp::Service<arm_api2_msgs::srv::SetStringParam>::SharedPtr           set_eelink_srv_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                      set_plan_only_srv_;
        rclcpp::Service<arm_api2_msgs::srv::AddCollisionObject>::SharedPtr      add_collision_object_srv_;

        /* actions */
        rclcpp_action::Server<arm_api2_msgs::action::MoveJoint>::SharedPtr              move_to_joint_as_;
        rclcpp_action::Server<arm_api2_msgs::action::MoveCartesian>::SharedPtr          move_to_pose_as_;
        rclcpp_action::Server<arm_api2_msgs::action::MoveCartesianPath>::SharedPtr      move_to_pose_path_as_;
        rclcpp_action::Server<control_msgs::action::GripperCommand>::SharedPtr          gripper_control_as_;

        /* topic callbacks */
        void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
        void servo_twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        void processServoCommand();
        
        /* srv callbacks*/
        void change_state_cb(const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req, 
                             const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res);
        void set_vel_acc_cb(const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Request> req,
                             const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Response> res);
        void set_planner_cb(const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req,
                            const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res);
        void set_eelink_cb(const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req,
                            const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res);
        void set_plan_only_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                               const std::shared_ptr<std_srvs::srv::SetBool::Response> res);
        void add_collision_object_cb(const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Request> req,
                                      const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Response> res);

        /* action callbacks */
        rclcpp_action::GoalResponse move_to_joint_goal_cb(
            const rclcpp_action::GoalUUID &uuid, 
            std::shared_ptr<const arm_api2_msgs::action::MoveJoint::Goal> goal);
        rclcpp_action::CancelResponse move_to_joint_cancel_cb(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>> goal_handle);
        void move_to_joint_accepted_cb(
            std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>> goal_handle);
        
        rclcpp_action::GoalResponse move_to_pose_goal_cb(
            const rclcpp_action::GoalUUID &uuid, 
            std::shared_ptr<const arm_api2_msgs::action::MoveCartesian::Goal> goal);
        rclcpp_action::CancelResponse move_to_pose_cancel_cb(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>> goal_handle);
        void move_to_pose_accepted_cb(
            std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>> goal_handle);
        
        rclcpp_action::GoalResponse move_to_pose_path_goal_cb(
            const rclcpp_action::GoalUUID &uuid, 
            std::shared_ptr<const arm_api2_msgs::action::MoveCartesianPath::Goal> goal);
        rclcpp_action::CancelResponse move_to_pose_path_cancel_cb(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>> goal_handle);
        void move_to_pose_path_accepted_cb(
            std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>> goal_handle);

        rclcpp_action::GoalResponse gripper_control_goal_cb(
            const rclcpp_action::GoalUUID &uuid, 
            std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal);
        rclcpp_action::CancelResponse gripper_control_cancel_cb(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void gripper_control_accepted_cb(
            std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);

        bool run(); 

        /* setters */
        bool setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs); 
        bool setRobotModel(rclcpp::Node::SharedPtr nodePtr); 
        bool setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name);

        /* getters */
        void getArmState();  

        /* funcs */
        void planAndExecJoint();
        void planAndExecPose();
        void planAndExecPosePath();
        void printTimestamps(const moveit_msgs::msg::RobotTrajectory &trajectory);
        void addTimestampsToTrajectory(moveit_msgs::msg::RobotTrajectory &trajectory);
        bool planWithPlanner(moveit::planning_interface::MoveGroupInterface::Plan &plan, bool eagerExecution = true);

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
        bool recivTraj          = false; 
        bool recivGripperCmd    = false;
        bool servoEntered       = false;
        rclcpp::Time servo_entered_time_;
        geometry_msgs::msg::TwistStamped latest_twist_cmd_;
        std::atomic<bool> new_twist_cmd_{false};
        moveit_servo::KinematicState last_servo_state_;
        bool async              = false;
        bool planOnly           = false; 

        /* goal handles */
        std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>> m_moveToJointGoalHandle_;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>> m_moveToPoseGoalHandle_;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>> m_moveToPosePathGoalHandle_;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> m_gripperControlGoalHandle_;
        
        /* ros vars */
        std::vector<double> m_currJointPosition;
        geometry_msgs::msg::PoseStamped m_currPoseState; 
        std::vector<geometry_msgs::msg::Pose> m_cartesianWaypoints; 
        

        moveit::planning_interface::MoveGroupInterfacePtr m_moveGroupPtr; 
        moveit::core::RobotStatePtr m_robotStatePtr;  
        moveit::core::RobotModelPtr kinematic_model; 
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> m_pSceneMonitorPtr; 
        std::shared_ptr<servo::ParamListener> servo_param_listener_;
        std::unique_ptr<moveit_servo::Servo> servoPtr; 
        moveit::planning_interface::PlanningSceneInterfacePtr m_planningSceneInterface; 

}; 

#endif
