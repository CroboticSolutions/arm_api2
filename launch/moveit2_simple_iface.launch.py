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

 #      Title       : moveit2_simple_iface.launch.py
 #      Project     : arm_api2
 #      Created     : 05/10/2024
 #      Author      : Filip Zoric
 #
 #      Description : Launch file for moveit2_simple_iface
 #

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from launch.substitutions import LaunchConfiguration

import yaml
import os

# TODO: Make this changeable without ERROR for wrong param type
use_sim_time = True
use_servo = True
dt = 0.1

def get_moveit_configs(robot_name):
    """Load MoveIt configs for supported robots."""
    if robot_name == "so_arm100":
        from so_arm100_description.launch_utils import MoveItConfigs
        return MoveItConfigs().to_dict()
    elif robot_name == "piper":
        from moveit_configs_utils import MoveItConfigsBuilder
        moveit_config = MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()
        return moveit_config.to_dict()
    return {}

def launch_setup(context, *args, **kwargs):

    launch_nodes_ = []
    arg_robot_name      = context.perform_substitution(LaunchConfiguration('robot_name'))
    arg_launch_joy      = context.perform_substitution(LaunchConfiguration('launch_joy', default=True))
    arg_use_gdb         = context.perform_substitution(LaunchConfiguration('use_gdb', default=False))
    arg_use_sim_time    = context.perform_substitution(LaunchConfiguration('use_sim_time', default='false'))


    # TODO: Swap between sim and real arg depending on the robot type
    robot_yaml = "{0}/{1}_sim.yaml".format(arg_robot_name, arg_robot_name)
    servo_yaml = "{0}/{1}_servo_sim.yaml".format(arg_robot_name, arg_robot_name)
    kinematics_yaml = "config/{0}/{1}_kinematics.yaml".format(arg_robot_name, arg_robot_name)
    
    # Arm params (ctl, servo) --> sent just as path
    # 3 different ways of loading and using yaml files, DISGUSTING [FIX ASAP]
    config_path = os.path.join(
        get_package_share_directory('arm_api2'),
        "config",
        robot_yaml
    )

    # Servo params created with the help of ParameterBuilder
    servo_params = {
        "moveit_servo": ParameterBuilder("arm_api2") 
        .yaml(f"config/{servo_yaml}")
        .to_dict()
    }
    
    # Load kinematic params
    kinematic_params = load_yaml("arm_api2", kinematics_yaml)

    # Load MoveIt configs for the robot (robot_description, robot_description_semantic, etc.)
    moveit_configs = get_moveit_configs(arg_robot_name)

    print(moveit_configs)

    # Build parameter list
    node_params = [
        {"use_sim_time": arg_use_sim_time.lower() == 'true'},
        {"enable_servo": use_servo},
        {"dt": dt},
        {"config_path": config_path},
    ]
    
    # Add MoveIt configs if available
    if moveit_configs:
        node_params.append(moveit_configs)
    
    # Add kinematic params if available
    if kinematic_params:
        node_params.append({"robot_description_kinematics": kinematic_params})
    
    # Add servo params if available
    if servo_params:
        node_params.append(servo_params)


    # Configure GDB prefix if debugging is enabled
    prefix_cmd = []
    if arg_use_gdb.lower() == 'true':
        prefix_cmd =['xterm -e gdb -ex run --args']

    launch_arm_api2 = Node(
        package='arm_api2',
        executable='moveit2_simple_iface',
        parameters=node_params,
        prefix=prefix_cmd,
        output='screen'
    )

    launch_nodes_.append(launch_arm_api2)

    if arg_launch_joy: 

        # https://index.ros.org/p/joy/ --> joy node as joystick (Create subscriber that takes cmd_vel)
        # Example of demo joint_jog
        joy_node = Node(
            package='joy', 
            executable="joy_node", 
            output="screen", 
            arguments={'device_name':'js0'}.items()
        )
        launch_nodes_.append(joy_node)

        joy_ctl_node = Node(
            package="arm_api2", 
            executable="joy_ctl", 
            output="screen", 
            parameters = [{"use_sim_time": use_sim_time}]
        )

        launch_nodes_.append(joy_ctl_node)

    return launch_nodes_

#https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/
def generate_launch_description(): 

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(name='robot_name',
                              default_value='kinova',
                              description='robot name')
    )
    # TODO: THIS IS NOT CONVERTED TO FALSE WHEN SETUP! FIX IT!
    declared_arguments.append(
        DeclareLaunchArgument(name='launch_joy', 
                              default_value='true', 
                              description='launch joystick')
    )

    declared_arguments.append(
        DeclareLaunchArgument(name='dt', 
                              default_value='0.1', 
                              description='time step')
    )

    declared_arguments.append(
        DeclareLaunchArgument(name='use_sim_time', 
                              default_value='false', 
                              description='use simulation time')
    )

    declared_arguments.append(
        DeclareLaunchArgument(name='use_gdb',
                              default_value='false',
                              description='Run node with GDB debugger')
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def load_yaml(package_name: str, file_path: str):
    """
    Load yaml configuration based on package name and file path relative to its share.
    """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)

def parse_yaml(absolute_file_path: str):
    """
    Parse yaml from file, given its absolute file path.
    """

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None