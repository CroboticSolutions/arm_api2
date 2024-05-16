#include "arm_api2/arm_joy.hpp"

JoyCtl::JoyCtl(): Node("joy_ctl")
{
    init();

    setScaleFactor(1); 

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
	float x_dir, y_dir, yaw;  
	std::vector<float> axes_ = msg->axes; 
	
    x_dir = axes_.at(3); 
	y_dir = axes_.at(2); 
	yaw = axes_.at(4);

    // Enabling joystick functionality
    // R2 pressed --> joy on
    int LOG_JOY_STATE_T = 5000; 
    if (msg->buttons.at(7) == 1)
    { 
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, LOG_JOY_STATE_T, "ON");    
        setEnableJoy(true); 
    }

    // R2 released --> joy off
    if (msg->buttons.at(7) == 0)
    {
        
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, LOG_JOY_STATE_T, "OFF"); 
        setEnableJoy(false); 
    }

    enableJoy_ = getEnableJoy(); 

    int sF = getScaleFactor();
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
    setScaleFactor(sF); 


	
    // Create teleop msg
    auto teleop_msg 	    = geometry_msgs::msg::TwistStamped(); 
    teleop_msg.header.stamp = this->get_clock()->now(); 
    teleop_msg.twist.linear.x	    = x_dir  * sF; 
    teleop_msg.twist.linear.y 	= y_dir   * sF; 
    teleop_msg.twist.angular.z 	= yaw    * sF; 
    
    if (enableJoy_){
        cmdVelPub_->publish(teleop_msg); 
    }
    else{
        teleop_msg.twist.linear.x = 0;
        teleop_msg.twist.linear.y = 0;
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