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

/*      Title       : utils.hpp
 *      Project     : arm_api2
 *      Created     : 06/21/2024
 *      Author      : Filip Zoric
 *
 *      Description : Implementation of the gripper class for the Robotiq gripper
 */

#ifndef ROBOTIQ_GRIPPER_H
#define ROBOTIQ_GRIPPER_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "gripper.hpp"

class RobotiqGripper: public Gripper {

public:
    // Constructor
    RobotiqGripper(){
        isOpen = false;
    };
    ~RobotiqGripper(){};

    // Method to open the gripper
    void open(){
        //TODO: Implement correct topic/action call for starters
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Gripper opened.");
        isOpen = true;
    }; 

    // Method to close the gripper
    void close(){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Gripper closed.");
        isOpen = false; 
    };

private:
    bool isOpen;
    // TODO: Add ROS2 action client or topic publisher which is going to be used
};

#endif // ROBOTIQ_GRIPPER_H