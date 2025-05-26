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
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : Joystick control code.
 */

#include "arm_api2/arm_joy.hpp"
#define X_I 4
#define Y_I 3
#define YAW_I 6
#define Z_I 7

JoyCtl::JoyCtl(): Node("joy_ctl")
{
    init();

    setScaleFactor(1); 
    this->set_parameter(rclcpp::Parameter("use_sim_time", false));

    enableJoy_ = true; 


}

void JoyCtl::init()
{

    // publishers
    cmdVelPub_ 		    = this->create_publisher<geometry_msgs::msg::TwistStamped>("/moveit2_iface_node/delta_twist_cmds", 1); 

    // subscribers
    joySub_ 		    = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyCtl::joy_callback, this, _1)); 

    RCLCPP_INFO(this->get_logger(), "Initialized joy_ctl"); 
}

void JoyCtl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{   
	float x_dir, y_dir, z_dir, yaw;  
	std::vector<float> axes_ = msg->axes; 
	
    x_dir = axes_.at(X_I); 
	y_dir = axes_.at(Y_I); 
    z_dir = axes_.at(Z_I); 
	yaw = axes_.at(YAW_I);

    // Enabling joystick functionality
    // R2 pressed --> joy on
    int LOG_JOY_STATE_T = 5000; 
    if (msg->buttons.at(5) == 1)
    { 
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, LOG_JOY_STATE_T, "ON");    
        setEnableJoy(true); 
    }

    // R2 released --> joy off
    if (msg->buttons.at(5) == 0)
    {
        
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, LOG_JOY_STATE_T, "OFF"); 
        setEnableJoy(false); 
    }

    enableJoy_ = getEnableJoy(); 

    float sF = getScaleFactor();
    // https://www.quantstart.com/articles/Passing-By-Reference-To-Const-in-C/ 
    if (msg->axes.at(1) == 1){
        
        if (sF > 0 && sF < 100)
        {
          sF += 1; 
          RCLCPP_INFO_STREAM(this->get_logger(), "Increasing scale factor: " << sF); 
        }
        else{sF = 1;}
    }
    
    if (msg->axes.at(1) == -1){
       if (sF > 0 && sF < 100) 
       {
        sF -= 1; 
        RCLCPP_INFO_STREAM(this->get_logger(), "Decreasing scale factor: " << sF); 
       }
       else{sF = 1;}
    }

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

    float C = 0.1; 
    if (enableJoy_){
        teleop_msg.twist.linear.x = x_dir  * sF * C; 
        teleop_msg.twist.linear.y  = y_dir  * sF * C;
        teleop_msg.twist.linear.z = z_dir * sF * C;  
        teleop_msg.twist.angular.z 	= yaw  * sF * C; 
        cmdVelPub_->publish(teleop_msg); 
    }
    else{
        teleop_msg.twist.linear.x = 0;
        teleop_msg.twist.linear.y = 0;
        teleop_msg.twist.linear.z = 0; 
        teleop_msg.twist.angular.z = 0;
	cmdVelPub_->publish(teleop_msg); 
    }


}

// Methods that set scale factor 
void JoyCtl::setScaleFactor(int value)
{
    scale_factor = value; 
}

int JoyCtl::getScaleFactor() const
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