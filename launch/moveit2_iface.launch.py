# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2024-2026 Crobotic Solutions d.o.o.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from launch.substitutions import LaunchConfiguration

import os
import re
import yaml

use_servo = True

ROS_NAME_PATTERN = re.compile(r"^[a-zA-Z][a-zA-Z0-9_]*$")


def _validate_ros_namespace(ns: str) -> str:
    if not ns or not ROS_NAME_PATTERN.match(ns):
        raise ValueError(f"robot namespace must match [a-zA-Z][a-zA-Z0-9_]*, got: {ns!r}")
    return ns


def get_moveit_configs(robot_name):
    """Load MoveIt configs for supported robots."""
    if robot_name == "so_arm100":
        from so_arm100_description.launch_utils import MoveItConfigs

        return MoveItConfigs().to_dict()
    return {}


def launch_setup(context, *args, **kwargs):

    launch_nodes_ = []
    arg_robot_name = context.perform_substitution(LaunchConfiguration("robot_name"))
    arg_robot_ns = context.perform_substitution(LaunchConfiguration("robot_ns", default=""))
    arg_robot_namespaces = context.perform_substitution(
        LaunchConfiguration("robot_namespaces", default="")
    )
    arg_launch_joy = context.perform_substitution(LaunchConfiguration("launch_joy", default=True))
    arg_launch_servo_watchdog = context.perform_substitution(
        LaunchConfiguration("launch_servo_watchdog", default=True)
    )
    arg_use_sim_time = context.perform_substitution(
        LaunchConfiguration("use_sim_time", default="false")
    )
    arg_config_profile = context.perform_substitution(
        LaunchConfiguration("config_profile", default="sim")
    )
    arg_mode = context.perform_substitution(LaunchConfiguration("mode", default="advanced"))
    # Empty means "derived from mode" inside the node; non-empty overrides it.
    arg_enable_topics = context.perform_substitution(
        LaunchConfiguration("enable_topics", default="")
    )
    arg_enable_actions = context.perform_substitution(
        LaunchConfiguration("enable_actions", default="")
    )
    arg_use_gdb = context.perform_substitution(LaunchConfiguration("use_gdb", default="false"))
    arg_dt = float(context.perform_substitution(LaunchConfiguration("dt")))
    robot_yaml = "{0}/{1}_{2}.yaml".format(arg_robot_name, arg_robot_name, arg_config_profile)
    servo_yaml = "{0}/{1}_servo_{2}.yaml".format(
        arg_robot_name, arg_robot_name, arg_config_profile
    )
    kinematics_yaml = "config/{0}/{1}_kinematics.yaml".format(arg_robot_name, arg_robot_name)

    # Arm params (ctl, servo) --> sent just as path
    config_path = os.path.join(get_package_share_directory("arm_api2"), "config", robot_yaml)
    if not os.path.isfile(config_path):
        fallback_yaml = "{0}/{1}_sim.yaml".format(arg_robot_name, arg_robot_name)
        config_path = os.path.join(
            get_package_share_directory("arm_api2"), "config", fallback_yaml
        )
        robot_yaml = fallback_yaml

    servo_path = os.path.join(get_package_share_directory("arm_api2"), "config", servo_yaml)
    if not os.path.isfile(servo_path):
        servo_yaml = "{0}/{1}_servo_sim.yaml".format(arg_robot_name, arg_robot_name)

    # Servo params created with the help of ParameterBuilder
    servo_params = {}
    try:
        servo_params = {
            "moveit_servo": ParameterBuilder("arm_api2").yaml(f"config/{servo_yaml}").to_dict()
        }
    except Exception as e:
        print(f"Warning: Could not load servo params: {e}")

    # Load kinematic params
    kinematic_params = load_yaml("arm_api2", kinematics_yaml) or {}

    # Load MoveIt configs for the robot (robot_description, robot_description_semantic, etc.)
    moveit_configs = get_moveit_configs(arg_robot_name)

    # Build parameter list
    node_params = [
        {"use_sim_time": arg_use_sim_time.lower() == "true"},
        {"enable_servo": use_servo},
        {"mode": arg_mode},
        {"dt": arg_dt},
        {"config_path": config_path},
    ]
    if arg_enable_topics:
        node_params.append({"enable_topics": arg_enable_topics.lower() == "true"})
    if arg_enable_actions:
        node_params.append({"enable_actions": arg_enable_actions.lower() == "true"})

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
    if arg_use_gdb.lower() == "true":
        prefix_cmd = ["xterm -e gdb -ex run --args"]

    # Multi-robot: robot_namespaces (e.g. "ur1;ur2") spawns one iface per
    # namespace and takes precedence over robot_ns.
    multi_robot = bool(arg_robot_namespaces.strip())
    if multi_robot:
        namespaces = [p.strip() for p in arg_robot_namespaces.split(";") if p.strip()]
        for ns in namespaces:
            _validate_ros_namespace(ns)
    else:
        namespaces = [arg_robot_ns]

    for idx, robot_ns in enumerate(namespaces):
        # Empty robot_ns means root; remapping /tf -> //tf is invalid for rcl.
        tf_remappings = []
        if robot_ns:
            tf_remappings = [
                ("/tf", f"/{robot_ns}/tf"),
                ("/tf_static", f"/{robot_ns}/tf_static"),
            ]

        launch_move_group = Node(
            package="arm_api2",
            executable="moveit2_iface",
            output="screen",
            namespace=robot_ns,
            parameters=node_params,
            remappings=tf_remappings,
            prefix=prefix_cmd,
        )
        launch_nodes_.append(launch_move_group)

        if str(arg_launch_servo_watchdog).lower() == "true":
            launch_servo_watchdog = Node(
                package="arm_api2",
                executable="servo_watchdog.py",
                output="screen",
                namespace=robot_ns,
            )
            launch_nodes_.append(launch_servo_watchdog)

        # One shared joy_node; in multi-robot mode joy_ctl drives the first robot.
        if str(arg_launch_joy).lower() == "true" and idx == 0:
            joy_node = Node(
                package="joy",
                executable="joy_node",
                output="screen",
                arguments={"device_name": "js0"}.items(),
            )
            launch_nodes_.append(joy_node)
            if multi_robot:
                joy_ctl_node = Node(
                    package="arm_api2",
                    executable="joy_ctl",
                    namespace=robot_ns,
                    output="screen",
                    parameters=[{"use_sim_time": arg_use_sim_time.lower() == "true"}],
                )
                launch_nodes_.append(joy_ctl_node)

    return launch_nodes_


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(name="robot_name", default_value="kinova", description="robot name")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="robot_ns",
            default_value="",
            description="ROS namespace for arm_api2 instance (e.g. ur1, ur2). Empty keeps root namespace.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="robot_namespaces",
            default_value="",
            description=(
                "Semicolon-separated namespaces for multi-robot setups (e.g. ur1;ur2). "
                "Spawns one arm_api2 instance per namespace; overrides robot_ns."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="launch_joy", default_value="false", description="launch joystick"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="launch_servo_watchdog",
            default_value="false",
            description="launch servo_watchdog node",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(name="dt", default_value="0.01", description="time step")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_sim_time", default_value="false", description="use simulation time"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="config_profile",
            default_value="sim",
            description="arm_api2 config profile suffix: sim or real",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="mode",
            default_value="advanced",
            description=(
                "Interface profile: 'simple' exposes just the topic interface, "
                "'advanced' adds the action servers"
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="enable_topics",
            default_value="",
            description="Override topic interface on/off; empty derives it from mode",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="enable_actions",
            default_value="",
            description="Override action interface on/off; empty derives it from mode",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_gdb",
            default_value="false",
            description="Run the node under gdb in an xterm window",
        )
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
