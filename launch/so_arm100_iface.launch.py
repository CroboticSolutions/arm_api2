from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from so_arm100_description.launch_utils import MoveItConfigs

def generate_launch_description():
    
    # Load MoveIt configs (includes URDF, SRDF, kinematics)
    moveit_configs = MoveItConfigs()
    
    # arm_api2 config path
    import os
    from ament_index_python.packages import get_package_share_directory
    config_path = os.path.join(
        get_package_share_directory('arm_api2'),
        'config',
        'so_arm100',
        'so_arm100_sim.yaml'
    )
    
    # Load kinematics yaml
    kinematics_yaml = moveit_configs.robot_description_kinematics
    
    moveit2_iface_node = Node(
        package='arm_api2',
        executable='moveit2_iface',
        name='moveit2_iface',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'enable_servo': False},
            {'dt': 0.1},
            {'config_path': config_path},
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            kinematics_yaml,
        ]
    )
    
    return LaunchDescription([
        moveit2_iface_node,
    ])
