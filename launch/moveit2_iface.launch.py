# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

import os

# https://roboticsbackend.com/ros2-yaml-params/
# https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
yaml = "franka_sim.yaml"
use_sim_time = True

def generate_launch_description(): 

    ld = LaunchDescription()

    config_path_ = os.path.join(
         get_package_share_directory('arm_api2'), 
        "config", 
        yaml
    )

    node = Node(
        package ='arm_api2', 
        name ='movei2_iface_node', 
        executable ='moveit2_iface', 
        parameters = [{"use_sim_time": use_sim_time}, 
                      {"config_path": config_path_}]
        # Stupid naming conventions 
    )

    ld.add_action(node)
    return ld