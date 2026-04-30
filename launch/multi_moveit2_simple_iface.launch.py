# BSD 3-Clause License — same as moveit2_simple_iface.launch.py (arm_api2)
#
# Launches one moveit2_simple_iface (+ optional joy) per ROS namespace, e.g. ur1;ur2
# for multi-robot simulation. Only one joy_node is started globally; joy_ctl runs
# in the first namespace only when launch_joy is true.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
        raise ValueError(
            f"robot namespace must match [a-zA-Z][a-zA-Z0-9_]*, got: {ns!r}"
        )
    return ns


def get_moveit_configs(robot_name):
    if robot_name == "so_arm100":
        from so_arm100_description.launch_utils import MoveItConfigs

        return MoveItConfigs().to_dict()
    return {}


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def launch_setup(context: LaunchContext, *args, **kwargs):
    robot_namespaces_raw = context.perform_substitution(LaunchConfiguration("robot_namespaces"))
    parts = [p.strip() for p in robot_namespaces_raw.split(";") if p.strip()]
    if not parts:
        raise ValueError(
            "robot_namespaces must contain at least one namespace, e.g. ur1;ur2"
        )

    arg_robot_name = context.perform_substitution(LaunchConfiguration("robot_name"))
    arg_launch_joy = context.perform_substitution(LaunchConfiguration("launch_joy", default="false"))
    arg_use_gdb = context.perform_substitution(LaunchConfiguration("use_gdb", default="false"))
    arg_use_sim_time = context.perform_substitution(LaunchConfiguration("use_sim_time", default="false"))
    arg_dt = float(context.perform_substitution(LaunchConfiguration("dt")))

    robot_yaml = "{0}/{1}_sim.yaml".format(arg_robot_name, arg_robot_name)
    servo_yaml = "{0}/{1}_servo_sim.yaml".format(arg_robot_name, arg_robot_name)
    kinematics_yaml = "config/{0}/{1}_kinematics.yaml".format(arg_robot_name, arg_robot_name)

    config_path = os.path.join(
        get_package_share_directory("arm_api2"),
        "config",
        robot_yaml,
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("arm_api2")
        .yaml(f"config/{servo_yaml}")
        .to_dict()
    }

    kinematic_params = load_yaml("arm_api2", kinematics_yaml)
    moveit_configs = get_moveit_configs(arg_robot_name)

    launch_nodes = []
    joy_node_added = False
    use_sim_flag = arg_use_sim_time.lower() == "true"

    for idx, arg_robot_ns in enumerate(parts):
        _validate_ros_namespace(arg_robot_ns)

        node_params = [
            {"use_sim_time": use_sim_flag},
            {"enable_servo": use_servo},
            {"dt": arg_dt},
            {"config_path": config_path},
        ]

        if moveit_configs:
            node_params.append(moveit_configs)

        if kinematic_params:
            node_params.append({"robot_description_kinematics": kinematic_params})

        if servo_params:
            node_params.append(servo_params)

        prefix_cmd = []
        if arg_use_gdb.lower() == "true":
            prefix_cmd = ["xterm -e gdb -ex run --args"]

        launch_arm_api2 = Node(
            package="arm_api2",
            executable="moveit2_simple_iface",
            namespace=arg_robot_ns,
            parameters=node_params,
            remappings=[
                ("/tf", f"/{arg_robot_ns}/tf"),
                ("/tf_static", f"/{arg_robot_ns}/tf_static"),
            ],
            prefix=prefix_cmd,
            output="screen",
        )
        launch_nodes.append(launch_arm_api2)

        if arg_launch_joy.lower() == "true":
            if not joy_node_added:
                joy_node = Node(
                    package="joy",
                    executable="joy_node",
                    output="screen",
                    arguments={"device_name": "js0"}.items(),
                )
                launch_nodes.append(joy_node)
                joy_node_added = True
            if idx == 0:
                joy_ctl_node = Node(
                    package="arm_api2",
                    executable="joy_ctl",
                    namespace=arg_robot_ns,
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_flag}],
                )
                launch_nodes.append(joy_ctl_node)

    return launch_nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_namespaces",
                default_value="ur1;ur2",
                description="Semicolon-separated ROS namespaces (e.g. ur1;ur2;ur3).",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="ur",
                description="arm_api2 config folder name (e.g. ur for UR robots).",
            ),
            DeclareLaunchArgument(
                "launch_joy",
                default_value="false",
                description="If true, one joy_node and joy_ctl only for the first namespace.",
            ),
            DeclareLaunchArgument(
                "dt",
                default_value="0.01",
                description="time step",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="use simulation time",
            ),
            DeclareLaunchArgument(
                "use_gdb",
                default_value="false",
                description="Run nodes with GDB debugger",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
