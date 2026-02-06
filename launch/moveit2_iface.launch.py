######################################################################
# BSD 3-Clause License
#
# Copyright (c) 2024, Crobotic Solutions d.o.o.
# All rights reserved.
######################################################################
#
#      Title       : moveit2_iface.launch.py
#      Project     : arm_api2
#      Created     : 06/08/2025
#      Author      : Filip Zoric
#
#      Description : Launch file for moveit2_iface.
#                    One launch = one robot. Use robot_namespaces:=ur1 or ur2 per terminal.
#

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from launch.substitutions import LaunchConfiguration

import yaml
import os

use_sim_time = True
dt = 0.1


def _create_moveit2_iface_node(config_path, servo_params, kinematic_params, use_servo, node_name=None, namespace=None):
    """Create a moveit2_iface Node with given config."""
    node_params = [
        {"use_sim_time": use_sim_time},
        {"enable_servo": use_servo},
        {"dt": dt},
        {"config_path": config_path},
        servo_params,
    ]
    if kinematic_params:
        node_params.insert(-1, {"robot_description_kinematics": kinematic_params})

    node_kwargs = {
        "package": "arm_api2",
        "executable": "moveit2_iface",
        "name": node_name,
        "output": "screen",
        "parameters": node_params,
    }
    if namespace:
        node_kwargs["namespace"] = namespace
    return Node(**node_kwargs)


def launch_setup(context, *args, **kwargs):
    launch_nodes_ = []
    arg_robot_name = context.perform_substitution(LaunchConfiguration("robot_name"))
    arg_robot_namespaces = context.perform_substitution(
        LaunchConfiguration("robot_namespaces", default="")
    )
    arg_launch_joy = context.perform_substitution(
        LaunchConfiguration("launch_joy", default="true")
    )
    arg_enable_servo = context.perform_substitution(
        LaunchConfiguration("enable_servo", default="true")
    )
    arg_launch_servo_watchdog = context.perform_substitution(
        LaunchConfiguration("launch_servo_watchdog", default="true")
    )

    use_servo = str(arg_enable_servo).lower() == "true"
    pkg_share = get_package_share_directory("arm_api2")


    # robot_namespaces: "ur1" or "ur2" - one per terminal. Empty = single-robot mode (robot_name).
    ns = arg_robot_namespaces.strip().split(",")[0].strip() if arg_robot_namespaces else ""
    if ns:
        instance_name = ns
        config_subdir = "ur"
        kinematics = load_yaml("arm_api2", "config/ur/ur_kinematics.yaml")
    else:
        instance_name = arg_robot_name
        config_subdir = arg_robot_name
        ns = ""
        kinematics = load_yaml("arm_api2", f"config/{arg_robot_name}/{arg_robot_name}_kinematics.yaml")

    config_path = os.path.join(pkg_share, "config", config_subdir, f"{instance_name}_sim.yaml")
    servo_params = {
        "moveit_servo": ParameterBuilder("arm_api2")
        .yaml(f"config/{config_subdir}/{instance_name}_servo_sim.yaml")
        .to_dict()
    }
    node_name = f"moveit2_iface_{ns}" if ns else "moveit2_iface"
    node = _create_moveit2_iface_node(
        config_path, servo_params, kinematics, use_servo,
        node_name=node_name,
        namespace=ns if ns else None,
    )
    launch_nodes_.append(node)

    if str(arg_launch_joy).lower() == "true":
        joy_node = Node(
            package="joy",
            executable="joy_node",
            output="screen",
            arguments={"device_name": "js0"}.items(),
        )
        launch_nodes_.append(joy_node)

    if str(arg_launch_servo_watchdog).lower() == "true":
        launch_servo_watchdog = Node(
            package="arm_api2",
            executable="servo_watchdog.py",
            output="screen",
        )
        launch_nodes_.append(launch_servo_watchdog)

    return launch_nodes_


# https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/
def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            name="robot_name",
            default_value="kinova",
            description="Robot name for single-robot mode (config subdir, e.g. ur, kinova).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="robot_namespaces",
            default_value="",
            description='Namespace for this instance (e.g. ur1, ur2). Empty = single robot mode.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(name='launch_joy',
                             default_value='true',
                             description='launch joystick')
    )

    declared_arguments.append(
        DeclareLaunchArgument(name='launch_servo_watchdog',
                             default_value='true',
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

    declared_arguments.append(
        DeclareLaunchArgument(name='enable_servo',
                             default_value='true',
                             description='enable MoveIt Servo (set false to debug action server init)')
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
