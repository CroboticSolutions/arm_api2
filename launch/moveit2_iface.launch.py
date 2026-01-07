######################################################################
 # BSD 3-Clause License
 #
 # Copyright (c) 2024, Crobotic Solutions d.o.o.
 # All rights reserved.
######################################################################

 #      Title       : moveit2_iface.launch.py
 #      Project     : arm_api2
 #      Created     : 06/08/2025
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
    arg_launch_servo_watchdog = context.perform_substitution(LaunchConfiguration('launch_servo_watchdog', default=True))
    arg_use_sim_time    = context.perform_substitution(LaunchConfiguration('use_sim_time', default='false'))
    print("arg_launch_joy: ", arg_launch_joy)   

    # TODO: Swap between sim and real arg depending on the robot type
    robot_yaml = "{0}/{1}_sim.yaml".format(arg_robot_name, arg_robot_name)
    servo_yaml = "{0}/{1}_servo_sim.yaml".format(arg_robot_name, arg_robot_name)
    kinematics_yaml = "config/{0}/{1}_kinematics.yaml".format(arg_robot_name, arg_robot_name)
    
    # Arm params (ctl, servo) --> sent just as path
    config_path = os.path.join(
        get_package_share_directory('arm_api2'),
        "config",
        robot_yaml
    )

    # Servo params created with the help of ParameterBuilder
    servo_params = {}
    try:
        servo_params = {
            "moveit_servo": ParameterBuilder("arm_api2") 
            .yaml(f"config/{servo_yaml}")
            .to_dict()
        }
    except Exception as e:
        print(f"Warning: Could not load servo params: {e}")
    
    # Load kinematic params
    kinematic_params = load_yaml("arm_api2", kinematics_yaml) or {}

    # Load MoveIt configs for the robot (robot_description, robot_description_semantic, etc.)
    moveit_configs = get_moveit_configs(arg_robot_name)

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

    launch_move_group = Node(
        package='arm_api2',
        executable='moveit2_iface',
        output='screen',
        parameters=node_params
    )

    launch_nodes_.append(launch_move_group)
    
    if str(arg_launch_joy).lower() == "true":
        joy_node = Node(
            package='joy', 
            executable="joy_node", 
            output="screen", 
            arguments={'device_name':"js0"}.items()
        )
        launch_nodes_.append(joy_node)
    
    if str(arg_launch_servo_watchdog).lower() == "true":
        launch_servo_watchdog = Node(
            package='arm_api2',
            executable="servo_watchdog.py",
            output='screen'
        )
        launch_nodes_.append(launch_servo_watchdog)

    return launch_nodes_

def generate_launch_description(): 

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(name='robot_name',
                              default_value='kinova',
                              description='robot name')
    )
    declared_arguments.append(
        DeclareLaunchArgument(name='launch_joy', 
                              default_value='false', 
                              description='launch joystick')
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(name='launch_servo_watchdog', 
                              default_value='false', 
                              description='launch servo_watchdog node')
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def load_yaml(package_name: str, file_path: str):
    """Load yaml configuration based on package name and file path relative to its share."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)

def parse_yaml(absolute_file_path: str):
    """Parse yaml from file, given its absolute file path."""
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
