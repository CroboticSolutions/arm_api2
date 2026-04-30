
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

/*      Title       : moveit2_simple_iface.hpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : The core robot manipulator and MoveIt2! ROS 2 interfacing header class.
 */

#ifndef MOVEIT2_SIMPLE_IFACE_HPP
#define MOVEIT2_SIMPLE_IFACE_HPP

#include <atomic>
#include <mutex>
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

//* moveit
#include <moveit_servo/servo.hpp>
#include <moveit_servo/moveit_servo_lib_parameters.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <pluginlib/class_loader.hpp>

//* msgs
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "arm_api2_msgs/msg/cartesian_waypoints.hpp"
#include "arm_api2_msgs/msg/plan_status.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/servo_status.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

//* srvs
#include "arm_api2_msgs/srv/change_state.hpp"
#include "arm_api2_msgs/srv/set_vel_acc.hpp"
#include "arm_api2_msgs/srv/set_string_param.hpp"
#include "arm_api2_msgs/srv/add_collision_object.hpp"
#include "arm_api2_msgs/srv/check_reachability.hpp"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
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

class m2SimpleIface: public rclcpp::Node
{

    public:

        m2SimpleIface(const rclcpp::NodeOptions &options);  
        //~m2SimpleIface();

        /* namespace param, maybe redundant */ 
        std::string ns_; 

    private: 

        /* node related stuff */
        rclcpp::Node::SharedPtr node_;
        rclcpp::Executor::SharedPtr executor_;
        std::thread executor_thread_;
        
        /* Thread safety */
        std::mutex pose_cmd_mutex_;

        /** Set while a trajectory is being executed (async: worker thread; sync: same thread). */
        std::atomic_bool trajectory_executing_{ false };

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
        std::string SERVO_TRAJECTORY_TOPIC;
        int NUM_CART_PTS; 
        bool ENABLE_SERVO; 

        /* timers */
        rclcpp::TimerBase::SharedPtr                                        timer_;

        /* parameters */
        std::string                                                         config_path; 
        bool                                                                enable_servo; 
        float                                                               dt; 
        float                                                               max_vel_scaling_factor;
        float                                                               max_acc_scaling_factor;
        
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
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr       servo_twist_sub_;

        /* pubs */
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       pose_state_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 robot_state_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 gripper_state_pub_;
        rclcpp::Publisher<arm_api2_msgs::msg::PlanStatus>::SharedPtr        plan_status_pub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr  servo_trajectory_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr       servo_forward_position_pub_;
        rclcpp::Publisher<moveit_msgs::msg::ServoStatus>::SharedPtr         servo_status_pub_;

        /* srvs */
        rclcpp::Service<arm_api2_msgs::srv::ChangeState>::SharedPtr              change_state_srv_;
        rclcpp::Service<arm_api2_msgs::srv::SetVelAcc>::SharedPtr                set_vel_acc_srv_;
        rclcpp::Service<arm_api2_msgs::srv::SetStringParam>::SharedPtr           set_planner_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                       open_gripper_srv_; 
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                       close_gripper_srv_;
        rclcpp::Service<arm_api2_msgs::srv::AddCollisionObject>::SharedPtr       add_collision_object_srv_;
        rclcpp::Service<arm_api2_msgs::srv::CheckReachability>::SharedPtr        check_reachability_srv_;
        rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client_;
        rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr   load_controller_client_;
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
        /* topic callbacks */
        void pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void cart_poses_cb(const arm_api2_msgs::msg::CartesianWaypoints::SharedPtr msg); 
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
        void open_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                             const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
        void close_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                             const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
        void add_collision_object_cb(const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Request> req,
                                     const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Response> res);
        void check_reachability_cb(const std::shared_ptr<arm_api2_msgs::srv::CheckReachability::Request> req,
                                   const std::shared_ptr<arm_api2_msgs::srv::CheckReachability::Response> res);
        bool run();

        /* Publishes a PlanStatus snapshot for the most recent plan attempt. */
        void publishPlanStatus(bool success,
                               const std::string &error_code,
                               const std::string &reason,
                               const geometry_msgs::msg::PoseStamped &requested_pose,
                               double plan_time_sec,
                               const std::string &mode);

        /* setters */
        bool setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs); 
        bool setRobotModel(rclcpp::Node::SharedPtr nodePtr); 
        bool setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name);

        /* getters */
        void getArmState();  
        bool loadController(const std::string& controller_name);
        bool configureController(const std::string& controller_name);
        bool switchControllers(const std::vector<std::string>& activate,
                               const std::vector<std::string>& deactivate);
        bool enterServoControllerMode();
        bool leaveServoControllerMode();

        /* funcs */
        /** @return true if a new execution was started or completed (sync); false if busy or plan failed. */
        bool execPlan(bool async);
        /** Execute an already-planned trajectory (joint-space Plan). Lets the
         *  caller (execMove) own planning so it can publish PlanStatus with
         *  meaningful timing/error metadata. */
        bool execPlan_with_plan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async);
        bool execMove(bool async);
        bool execCartesian(bool async);
        bool planExecCartesian(bool async);
        bool execTrajectory(moveit_msgs::msg::RobotTrajectory trajectory, bool async);
        /** Stop active trajectory before planning a new one. Overlap is also prevented by trajectory_executing_. */
        void stopBeforeAsyncExecute();

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
        bool forward_position_controller_active_ = false;
        rclcpp::Time servo_entered_time_;
        geometry_msgs::msg::TwistStamped latest_twist_cmd_;
        std::atomic<bool> new_twist_cmd_{false};
        moveit_servo::KinematicState last_servo_state_;
        bool async              = true; 

        /* planner info */
        std::string current_planner_id_ = "pilz_industrial_motion_planner";
        std::string current_planner_type_ = "LIN";

        /* planning budget for OMPL pose targets (seconds). 5 s is the MoveIt
         * default, but for an interactive GUI 1.5 s is plenty: a healthy goal
         * solves in << 100 ms, and an infeasible goal doesn't get more honest
         * with more time. Reduces user-visible latency on bad clicks 3×. */
        double pose_plan_time_sec_ = 1.5;

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
        moveit::planning_interface::PlanningSceneInterface m_planningSceneInterface;
        std::shared_ptr<servo::ParamListener> servo_param_listener_;
        std::unique_ptr<moveit_servo::Servo> servoPtr;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

}; 

#endif
