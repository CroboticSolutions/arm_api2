/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Crobotic Solutions d.o.o.
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

/*      Title       : arm_joy.cpp
 *      Project     : arm_api2
 *      Created     : 14/11/2024
 *      Author      : Guanqi Chen
 *
 *      Description : Joystick control code for Servoing
 */

#include "arm_api2/arm_joy.hpp"
#define X_I 4
#define Y_I 3
#define YAW_I 6
#define Z_I 7

// Some constants used in the Servo Teleop demo
const std::string TWIST_TOPIC = "/moveit2_iface_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/moveit2_iface_node/delta_joint_cmds";
const std::string JOINT_STATE_TOPIC = "/joint_states";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EEF_FRAME_ID = "tool0";
const std::string BASE_FRAME_ID = "base_link";

JoyCtl::JoyCtl(): Node("joy_ctl")
{
    init();

<<<<<<< HEAD
    setScaleFactor(1); 
    this->set_parameter(rclcpp::Parameter("use_sim_time", false));
=======
    setScaleFactor(0.1); 
>>>>>>> 47c7bb5 (integrate sucessfully the keyboard, for joy still have some problems)

    enableJoy_ = false; 


}

void JoyCtl::init()
{

    // publishers
    cmdVelPub_ 		    = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE); 
    joint_pub_ 		    = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
    // subscribers
    joySub_ 		    = this->create_subscription<sensor_msgs::msg::Joy>("/joy", ROS_QUEUE_SIZE, std::bind(&JoyCtl::joy_callback, this, _1)); 
    joint_state_sub_    = this->create_subscription<sensor_msgs::msg::JointState>(
        JOINT_STATE_TOPIC, ROS_QUEUE_SIZE,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            (void)msg;
            joint_states_received_ = true;
            // save joint names
            joint_names_ = msg->name;
        });

    RCLCPP_INFO(this->get_logger(), "Initialized joy_ctl");
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
    RCLCPP_INFO(this->get_logger(), "Joint %d: %s", int(i + 1), joint_names_[i].c_str());
    }

    puts("Reading from Joystick");
    puts("---------------------------");
    puts("Use Xbox button to turn on/off the joystick control.");
    puts("Use back and start buttons to accelerate and decelerate the motion.");
    puts("Use 'LT' and 'RT' for x-direction forward/backward.");
    puts("Use 'right stick - left/right' for y-direction left/right.");
    puts("Use 'right stick - up/down' for z-direction up/dwon.");
    puts("Use 'left stick - left/right' for roll rotation.");
    puts("Use 'left stick - up/down' for pitch rotation.");
    puts("Use 'LB' and 'RB' for -/+ yaw rotation.");
    puts("Cross axes left|up|right|down for joint 1|2|3|4 jog, Button X|Y|B|A for joint 5|6|7|8 jog.");
    puts("Use 'LS' to reverse the direction of jogging.");
 
}

void JoyCtl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{   
<<<<<<< HEAD
	float x_dir, y_dir, z_dir, yaw;  
	std::vector<float> axes_ = msg->axes; 
	
<<<<<<< HEAD
    x_dir = axes_.at(X_I); 
	y_dir = axes_.at(Y_I); 
    z_dir = axes_.at(Z_I); 
	yaw = axes_.at(YAW_I);
=======
    x_dir = axes_.at(1); 
	y_dir = axes_.at(0); 
	yaw = axes_.at(3);
>>>>>>> 2439fb3 (fixed bugs about jerking, added outputs for keyboard, integrate xbox joy ctl)
=======
    // while(!joint_states_received_)
    // {
    // RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
    // rclcpp::sleep_for(std::chrono::seconds(1));
    // }

    // print joint names with index (starting from 1)
	float x_dir, y_dir, z_dir, yaw, pitch, roll;

    // Create teleop msg
    auto teleop_msg 	    = geometry_msgs::msg::TwistStamped();
    auto joint_msg          = control_msgs::msg::JointJog();
>>>>>>> 47c7bb5 (integrate sucessfully the keyboard, for joy still have some problems)

    // Enabling joystick functionality
    // xbox button pressed --> on/off switch
    int LOG_JOY_STATE_T = 5000; 
<<<<<<< HEAD
    if (msg->buttons.at(5) == 1)
=======
    if (msg->axes.at(2) == -1)
>>>>>>> 2439fb3 (fixed bugs about jerking, added outputs for keyboard, integrate xbox joy ctl)
    { 
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, LOG_JOY_STATE_T, "ON"); 
        setEnableJoy(true);
    }
<<<<<<< HEAD

    // R2 released --> joy off
<<<<<<< HEAD
    if (msg->buttons.at(5) == 0)
=======
    if (msg->axes.at(2) == 1)
>>>>>>> 2439fb3 (fixed bugs about jerking, added outputs for keyboard, integrate xbox joy ctl)
=======
    else if (msg->axes.at(2) == 1)
>>>>>>> 47c7bb5 (integrate sucessfully the keyboard, for joy still have some problems)
    {
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, LOG_JOY_STATE_T, "OFF"); 
        setEnableJoy(false);
    }

    enableJoy_ = getEnableJoy();

    float sF = getScaleFactor();
    // https://www.quantstart.com/articles/Passing-By-Reference-To-Const-in-C/ 

    // Velocity control
    if (msg->buttons.at(7) == 1){
        
        if (sF > 0 && sF < 10)
        {
          sF += 0.1; 
          RCLCPP_INFO_STREAM(this->get_logger(), "Increasing scale factor: " << sF); 
        }
        else{sF = 0.1;}
    }
    
    if (msg->buttons.at(6) == 1){
       if (sF > 0 && sF < 10) 
       {
        sF -= 0.1; 
        RCLCPP_INFO_STREAM(this->get_logger(), "Decreasing scale factor: " << sF); 
       }
       else{sF = 0.1;}
    }

<<<<<<< HEAD
    /*if (msg->buttons.at(4) == 1) {
       RCLCPP_INFO_STREAM(this->get_logger(), "Calling jingle bells!"); 
       auto req_ = std::make_shared<std_srvs::srv::Trigger::Request>();
       jingleBellsClient_->async_send_request(req_); 
    }*/
    // Test scale fact (ADD C const to prevent large cmd)
    setScaleFactor(sF); 

	
    // Create teleop msg
    auto teleop_msg 	    = geometry_msgs::msg::TwistStamped(); 
    teleop_msg.header.stamp = this->get_clock()->now(); 
    teleop_msg.header.frame_id = "link6"; 

    float C = 0.01; 
    if (enableJoy_){
        // Currently modified for the PIPER
        teleop_msg.twist.linear.z = x_dir  * sF * C; 
        teleop_msg.twist.linear.y  = y_dir  * sF * C;
        teleop_msg.twist.linear.x = - z_dir * sF * C;  
        teleop_msg.twist.angular.z 	= yaw  * sF * C; 
        cmdVelPub_->publish(teleop_msg); 
=======
    // Motion control
    if (msg->axes.at(3) != 0)
    {
        y_dir = msg->axes.at(3);
    }

    if (msg->axes.at(4) != 0)
    {
        z_dir = msg->axes.at(4);
    }

    if (msg->buttons.at(8) == 1)
    {
        x_dir = 1.0;
    }

    if (msg->axes.at(5) <= 0)
    {
        x_dir = msg->axes.at(5);
    }

    if (msg->axes.at(1) != 0)
    {
        pitch = msg->axes.at(0);
    }

    if (msg->axes.at(0) != 0)
    {
        roll = msg->axes.at(1);
    }

    if (msg->buttons.at(4) == 1)
    {
        yaw = -1;
    }

    if (msg->buttons.at(5) == 1)
    {
        yaw = 1;
    }

    // Joint control
    if (msg->axes.at(6) == 1)
    {
        joint_msg.joint_names.push_back(joint_names_[0]);
        joint_msg.velocities.push_back(joint_vel_cmd_ * sF);
        RCLCPP_INFO(this->get_logger(), "Now is controlling: %s", joint_msg.joint_names[0].c_str());
    } // Cross axes left

    if(msg->axes.at(7) == 1)
    {
        joint_msg.joint_names.push_back(joint_names_[1]);
        joint_msg.velocities.push_back(joint_vel_cmd_ * sF);
        RCLCPP_INFO(this->get_logger(), "Now is controlling: %s", joint_msg.joint_names[0].c_str());
    } // Cross axes up

    if(msg->axes.at(6) == -1)
    {
        joint_msg.joint_names.push_back(joint_names_[2]);
        joint_msg.velocities.push_back(joint_vel_cmd_ * sF);
        RCLCPP_INFO(this->get_logger(), "Now is controlling: %s", joint_msg.joint_names[0].c_str());
    } // Cross axes right

    if(msg->axes.at(7) == -1)
    {
        joint_msg.joint_names.push_back(joint_names_[3]);
        joint_msg.velocities.push_back(joint_vel_cmd_ * sF);
        RCLCPP_INFO(this->get_logger(), "Now is controlling: %s", joint_msg.joint_names[0].c_str());
    } // Cross axes down

    if(msg->buttons.at(2) == 1)
    {
        joint_msg.joint_names.push_back(joint_names_[4]);
        joint_msg.velocities.push_back(joint_vel_cmd_ * sF);
        RCLCPP_INFO(this->get_logger(), "Now is controlling: %s", joint_msg.joint_names[0].c_str());
    } // Button X

    if(msg->buttons.at(3) == 1)
    {
        joint_msg.joint_names.push_back(joint_names_[5]);
        joint_msg.velocities.push_back(joint_vel_cmd_ * sF);
        RCLCPP_INFO(this->get_logger(), "Now is controlling: %s", joint_msg.joint_names[0].c_str());
    } // Button Y

    if(msg->buttons.at(1) == 1)
    {
        if(joint_names_.size() < 7)
        {
            RCLCPP_WARN(this->get_logger(), "Not enough joints");
        }else{
            joint_msg.joint_names.push_back(joint_names_[6]);
            joint_msg.velocities.push_back(joint_vel_cmd_ * sF);
            RCLCPP_INFO(this->get_logger(), "Now is controlling: %s", joint_msg.joint_names[0].c_str());
        }
    } // Button B

    if(msg->buttons.at(0) == 1)
    {   
        if(joint_names_.size() < 8)
        {
            RCLCPP_WARN(this->get_logger(), "Not enough joints");
        }else{
            joint_msg.joint_names.push_back(joint_names_[7]);
            joint_msg.velocities.push_back(joint_vel_cmd_ * sF);
            RCLCPP_INFO(this->get_logger(), "Now is controlling: %s", joint_msg.joint_names[0].c_str());
        }
    } // Button A

    if (msg->buttons.at(9) == 1)
    {
        joint_vel_cmd_ *= -1;
    } // Left stick button


    setScaleFactor(sF); 

    // Modify the message
    teleop_msg.header.stamp = this->get_clock()->now();
    teleop_msg.header.frame_id = frame_to_publish_; 
    teleop_msg.twist.linear.x	= x_dir  * sF; 
    teleop_msg.twist.linear.y 	= y_dir   * sF;
    teleop_msg.twist.linear.z 	= z_dir   * sF; 
    teleop_msg.twist.angular.z = yaw    * sF;
    teleop_msg.twist.angular.y = pitch * sF;
    teleop_msg.twist.angular.x = roll  * sF; 
    
    if (enableJoy_){
        cmdVelPub_->publish(teleop_msg);
        joint_pub_->publish(joint_msg); 
>>>>>>> 47c7bb5 (integrate sucessfully the keyboard, for joy still have some problems)
    }
    else{
        teleop_msg.twist.linear.x = 0;
        teleop_msg.twist.linear.y = 0;
<<<<<<< HEAD
        teleop_msg.twist.linear.z = 0; 
=======
        teleop_msg.twist.linear.z = 0;
>>>>>>> 47c7bb5 (integrate sucessfully the keyboard, for joy still have some problems)
        teleop_msg.twist.angular.z = 0;
        teleop_msg.twist.angular.y = 0;
        teleop_msg.twist.angular.x = 0;
	    cmdVelPub_->publish(teleop_msg);

        joint_msg.joint_names.clear();
        joint_msg.velocities.push_back(0.0);
        joint_pub_->publish(joint_msg); 
    }


}

// Methods that set scale factor 
void JoyCtl::setScaleFactor(float value)
{
    scale_factor = value; 
}

float JoyCtl::getScaleFactor() const
{
    return scale_factor; 
}

void JoyCtl::setEnableJoy(bool val) 
{
    enableJoy_ = val; 
}

bool JoyCtl::getEnableJoy() const
{
    return enableJoy_; 
}