#!/usr/bin/env python3
# Copyright: wrapper only; camera stack is depthai_ros_driver.
"""OAK-D SR RGB-D + colored point cloud for arm_api2 / Piper workflows.

Delegates to ``depthai_ros_driver`` ``sr_rgbd_pcl.launch.py`` so documented
commands stay::

    ros2 launch arm_api2 oak_depthai_sr_rgbd.launch.py
    ros2 launch arm_api2 oak_depthai_sr_rgbd.launch.py parent_frame:=link6 ...

HaMeR / ``mp_config.yaml`` expects ``/oak/right/image_rect``, so this wrapper
enables RGB rectification by default.

See also: ``handeye_depthai_mount_args`` output for ``cam_pos_*`` /
``cam_roll`` / ``cam_pitch`` / ``cam_yaw``.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arm_api2 = get_package_share_directory("arm_api2")
    depthai = get_package_share_directory("depthai_ros_driver")
    default_params = os.path.join(
        arm_api2, "config", "oak_d_sr_035_1m_quality.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="DepthAI params tuned for OAK-D SR point clouds at 0.35-1.0 m.",
            ),
            DeclareLaunchArgument(
                "rectify_rgb",
                default_value="true",
                description="Publish /oak/right/image_rect for HaMeR and RGBD consumers.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(depthai, "launch", "sr_rgbd_pcl.launch.py")
                ),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "rectify_rgb": LaunchConfiguration("rectify_rgb"),
                }.items(),
            )
        ]
    )
