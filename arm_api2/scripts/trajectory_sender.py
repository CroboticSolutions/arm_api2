#!/usr/bin/env python3

import rclpy
import csv
import numpy as np
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R 
from arm_api2_msgs.msg import CartesianWaypoints

# NOTE: Requirement transformations 
# Used: transformations-2024.5.24
# installed with: pip install transformations

class CreateAndPublishTrajectory(Node):

    def __init__(self):
        super().__init__('create_publish_trajectory')

        # Create subscribers
        self.curr_p_sub     = self.create_subscription(PoseStamped, "/arm/state/current_pose", self.curr_p_cb, 1)
        self.trigger_sub    = self.create_subscription(Bool, "/traj_trigger", self.trig_cb, 1)

        # Create publishers
        self.traj_pub       = self.create_publisher(CartesianWaypoints, '/arm/cmd/traj', 1)
        
        # Create timer
        timer_period        = 1.0  # seconds
        self.timer          = self.create_timer(timer_period, self.run)
        c_csv_pth = "/root/kortex2_ws/src/arm_api2/utils/CS_C.csv"
        s_csv_pth = "/root/kortex2_ws/src/arm_api2/utils/CS_S.csv"

        self.get_logger().info(f"C trajectory path is: {c_csv_pth}")
        self.get_logger().info(f"S trajectory path is: {s_csv_pth}")
        # Load positions from YAML file
        
        self.c_data = self.load_positions(c_csv_pth)    
        self.s_data = self.load_positions(s_csv_pth)

        # Flags
        self.reciv_trig = False
        self.reciv_p = False
        self.i = 0
    
    def load_positions(self, csv_pth):
        p_data = []
        with open(csv_pth, 'r') as file:
            reader = csv.DictReader(file)
            # TODO: Use this 
            for row in reader:
                print(row)
                x = row['x']; y = row['y']; z = row['z']
                p_data.append(np.array([float(x), float(y), float(z), 1]).T)
        return p_data
    
    def trig_cb(self, msg): 
        self.reciv_trig = True
        self.i+=1

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

    def create_trajectory(self, letter): 
        self.get_logger().info("Create trajectory!")

        if letter == 'S': data = self.s_data
        if letter == 'C': data = self.c_data
        ct = CartesianWaypoints()
        # TODO: remove c_data as hardcoding 
        for i, p in enumerate(data):
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
            if self.i == 0: 
                traj = self.create_trajectory('C')
            if self.i == 1: 
                traj = self.create_trajectory('S')
            self.publish_trajectory(traj)
            self.reciv_trig = False
            self.i = 0
        
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
