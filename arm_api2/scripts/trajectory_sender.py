#!/usr/bin/env python3

import rclpy
import csv
import os
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CreateAndPublishTrajectory(Node):

    def __init__(self):

        # Create QoS profile 
        qp = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        super().__init__('create_publish_trajectory')
        timer_period        = 1.0  # seconds

        # Create subscribers
        self.curr_p_sub     = self.create_subscription(Pose, "/arm/state/current_pose", self.curr_p_cb, 1)
        #self.curr_p_sub.register_callback(self.curr_p_cb)
        self.trigger_sub    = self.create_subscription(Bool, "/traj_trigger", self.trig_cb, 1)
        #self.trigger_sub.register_callback(self.trig_cb)

        # Create publishers
        self.traj_pub       = self.create_publisher(JointTrajectory, 'joint_trajectory', 10)
        
        # Create timer
        self.timer          = self.create_timer(timer_period, self.run)

        #pkg_pth             = get_package_share_directory("arm_api2")
        #c_csv_pth           = os.path.join(pkg_pth, 'utils', 'CS_C.csv')
        #s_csv_pth           = os.path.join(pkg_pth, 'utils', 'CS_S.csv')
        c_csv_pth = "/root/ws_moveit2/src/arm_api2/utils/CS_C.csv"
        s_csv_pth = "/root/ws_moveit2/src/arm_api2/utils/CS_S.csv"

        self.get_logger().info(f"C trajectory path is: {c_csv_pth}")
        self.get_logger().info(f"S trajectory path is: {s_csv_pth}")
        # Load positions from YAML file
        #self.load_positions(self.pth_to_csv)    
        
        # Flags
        self.reciv_trig = False
        self.reciv_p = False
    
    def load_positions(self):
        csv_file_path = './csv/positions.csv'  # Path to the CSV file
        self.positions_data = []
        with open(csv_file_path, 'r') as file:
            reader = csv.DictReader(file)
            # TODO: Use this 
            for row in reader:
                 self.positions_data.append({
                     'time_from_start': float(row['time_from_start']),
                     'positions': [float(row['joint1']), float(row['joint2']), float(row['joint3'])]
                })
    
    def trig_cb(self, msg): 
        self.reciv_trig = True

    def curr_p_cb(self, msg): 
        self.reciv_p = True
        self.curr_p = Pose()
        self.curr_p.position = msg.position
        self.curr_p.orientation = msg.orientation

    def create_trajectory(self): 
        self.get_logger().info("Create trajectory!")
        pass

    def publish_trajectory(self): 
        self.get_logger().info("Publish trajectory!")

    def run(self):
        if self.reciv_trig: 
            self.create_trajectory()
            self.publish_trajectory()
        
        else: 
            self.get_logger().info("Waiting for the trigger to publish trajectory!")

def main(args=None):
    rclpy.init(args=args)
    CPT = CreateAndPublishTrajectory()
    rclpy.spin(CPT)
    CPT.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
