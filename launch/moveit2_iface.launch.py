######################################################################
 # BSD 3-Clause License
 #
 # Copyright (c) 2024, Crobotic Solutions d.o.o.
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #
 # * Redistributions of source code must retain the above copyright notice, this
 #   list of conditions and the following disclaimer.
 #
 # * Redistributions in binary form must reproduce the above copyright notice,
 #   this list of conditions and the following disclaimer in the documentation
 #   and/or other materials provided with the distribution.
 #
 # * Neither the name of the copyright holder nor the names of its
 #   contributors may be used to endorse or promote products derived from
 #   this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 # ARE
 # DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 # FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 # DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 # SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 # OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
######################################################################

 #      Title       : m
 #      Project     : arm_api2
 #      Created     : 05/10/2024
 #      Author      : Filip Zoric
 #
 #      Description : Launch file for moveit2_iface
 #

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from launch.substitutions import LaunchConfiguration

import os

# https://roboticsbackend.com/ros2-yaml-params/
# https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
# TODO: Move all of this as launch arguments 
robot = "ur"
yaml = "{0}/{1}_sim.yaml".format(robot, robot)
servo_yaml = "{0}/{1}_servo_sim.yaml".format(robot, robot)
use_sim_time = True
enable_servo = True
joy = True
dt = 0.1

#TODO: Convert robot name to the part of the launch argument to be able to dynamically change the robot type
#https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/
def generate_launch_description(): 

    ld = LaunchDescription()
    # TODO: How to add robot type as default name for the argument?
    # Get config params for the robot 
    config_path = os.path.join(
         get_package_share_directory('arm_api2'), 
        "config", 
        yaml
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("arm_api2") 
        .yaml(f"config/{servo_yaml}")
        .to_dict()
        # moveit_servo.use_gazebo
    }

    node = Node(
        package ='arm_api2', 
        name ='moveit2_iface_node', 
        executable ='moveit2_iface', 
        parameters = [{"use_sim_time": use_sim_time}, 
                      {"config_path": config_path}, 
                      {"enable_servo": enable_servo},
                      {"dt": dt},  
                      servo_params]
        # Stupid naming conventions 
    )

    if joy: 

        # https://index.ros.org/p/joy/ --> joy node as joystick (Create subscriber that takes cmd_vel)
        # Example of demo joint_jog
        joy_node = Node(
            package='joy', 
            executable="joy_node", 
            output="screen", 
            arguments={'device_name':'js0'}.items()
        )

        joy_ctl_node = Node(
            package="arm_api2", 
            executable="joy_ctl", 
            output="screen", 
            parameters = [{"use_sim_time": use_sim_time}]
        )

        ld.add_action(joy_node)
        ld.add_action(joy_ctl_node)

    ld.add_action(node)
    return ld
