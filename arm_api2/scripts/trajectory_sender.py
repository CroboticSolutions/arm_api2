#!/usr/bin/env python3

import rclpy
import csv
import os
import numpy as np
import rclpy.duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from scipy.spatial.transform import Rotation as R 
from builtin_interfaces.msg import Duration
from arm_api2_msgs.msg import CartesianWaypoints

# NOTE: Requirement transformations 
# Used: transformations-2024.5.24
# installed with: pip install transformations

class CreateAndPublishTrajectory(Node):

    def __init__(self):

        # Create QoS profile --> not used currently
        #qp = QoSProfile(
        #    reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #    history=QoSHistoryPolicy.KEEP_LAST,
        #    depth=1
        #)
        super().__init__('create_publish_trajectory')
        timer_period        = 1.0  # seconds

        # Create subscribers
        self.curr_p_sub     = self.create_subscription(PoseStamped, "/arm/state/current_pose", self.curr_p_cb, 1)
        #self.curr_p_sub.register_callback(self.curr_p_cb)
        self.trigger_sub    = self.create_subscription(Bool, "/traj_trigger", self.trig_cb, 1)
        #self.trigger_sub.register_callback(self.trig_cb)

        # Create publishers
        self.traj_pub       = self.create_publisher(CartesianWaypoints, '/arm/cmd/traj', 1)
        
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
        
        self.c_data = self.load_positions(c_csv_pth)    
        self.s_data = self.load_positions(s_csv_pth)

        # Flags
        self.reciv_trig = False
        self.reciv_p = False
    
    def load_positions(self, csv_pth):
        p_data = []
        with open(csv_pth, 'r') as file:
            reader = csv.DictReader(file)
            # TODO: Use this 
            for row in reader:
                x = row['x']; y = row['y']; z = row['z']
                p_data.append(np.array([float(x), float(y), float(z), 1]).T)
        return p_data
                
    
    def trig_cb(self, msg): 
        self.reciv_trig = True

    def curr_p_cb(self, msg): 
        self.reciv_p = True
        self.curr_p = PoseStamped()
        self.curr_p.header = msg.header
        self.curr_p.pose.position = msg.pose.position
        self.curr_p.pose.orientation = msg.pose.orientation
        p, q = msg.pose.position, msg.pose.orientation
        x, y, z = p.x, p.y, p.z 
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        self.get_logger().info(f"x: {x}\t {y}\t {z}\t")
        # x, y, z, w
        R_ = R.from_quat([qx, qy, qz, qw])
        r = R_.as_matrix()
        p_ = np.array([x, y, z]).T.reshape(3, 1)
        self.get_logger().debug(f"R shape is: {r.shape}")
        self.get_logger().debug(f"p shape is: {p_.shape}")
        T_ = np.hstack((r, p_))
        self.T = np.vstack((T_, np.array([0, 0, 0, 1])))
        self.get_logger().debug(f"Reciv T matrix is: {self.T}")

    def create_trajectory(self): 
        self.get_logger().info("Create trajectory!")
        ct = CartesianWaypoints()
        # TODO: remove c_data as hardcoding 
        for i, p in enumerate(self.c_data):
            dt = 0.05
            self.get_logger().info(f"p is: {p}")
            self.get_logger().info(f"R is: {self.T}")
            p_ = np.matmul(self.T, p)
            r_ = R.from_matrix(self.T[:3, :3]); quat = r_.as_quat()
            rosp = Pose()
            rosp.position.x = p_[0]; rosp.position.y = p_[1]; rosp.position.z = p_[2]
            rosp.orientation.x = quat[0]; rosp.orientation.y = quat[1]; rosp.orientation.z = quat[2]; rosp.orientation.w = quat[3]
            ct.poses.append(rosp)
        return ct

    def publish_trajectory(self, msg):
        self.traj_pub.publish(msg) 
        self.get_logger().info("Publish trajectory!")

    def run(self):
        if self.reciv_trig and self.reciv_p:
            self.get_logger().info("Sending trajectory!") 
            ct = self.create_trajectory()
            self.publish_trajectory(ct)
            self.reciv_trig = False
        
        else: 
            if self.reciv_p: 
                self.get_logger().info("Recieved pose!")
            if self.reciv_trig: 
                self.get_logger().info("Recieved trigger!")
                
            self.get_logger().info("Waiting for the trigger to publish trajectory!")

def main(args=None): 
    rclpy.init(args=args)
    CPT = CreateAndPublishTrajectory()
    rclpy.spin(CPT)
    CPT.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
