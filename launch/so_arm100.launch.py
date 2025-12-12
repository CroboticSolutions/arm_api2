from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from so_arm100_description.launch_utils import MoveItConfigs

def generate_launch_description():
    moveit_configs = MoveItConfigs()
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        Node(
            package='arm_api2',
            executable='moveit2_iface',
            name='arm_api2',
            parameters=[
                moveit_configs.to_dict(),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'enable_servo': False},
                {'dt': 0.1},
                {'config_path': '/root/so_arm_ws/install/arm_api2/share/arm_api2/config/so_arm100/so_arm100_sim.yaml'},
            ],
            output='screen',
        ),
    ])
