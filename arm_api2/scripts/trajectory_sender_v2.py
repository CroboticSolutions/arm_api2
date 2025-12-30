#!/usr/bin/env python3

import rclpy
import numpy as np
import pandas as pd
import rclpy.duration
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R 

from arm_api2_msgs.action import MoveCartesian

class CreateAndPublishTrajectory(Node):

    def __init__(self):
        super().__init__('create_publish_trajectory')

        # 1. Subscribers
        self.curr_p_sub = self.create_subscription(PoseStamped, "/arm/state/current_pose", self.curr_p_cb, 1)
        self.trigger_sub = self.create_subscription(Bool, "/traj_trigger", self.trig_cb, 1)

        # 2. Action Client
        self._action_client = ActionClient(self, MoveCartesian, '/arm/move_to_pose')
        
        # 3. Timer (samo za provjeru stanja)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.run_check)
        
        # 4. Ucitavanje CSV-a
        c_csv_pth = "/root/hpe_ws/src/mp_ros_wrapper/mp_wrapper_ros/desni_kaziprst2d_offset.csv"
        
        self.get_logger().info(f"Trajectory path is: {c_csv_pth}")
        self.c_data = self.load_positions(c_csv_pth)    

        # Flags i Queue
        self.reciv_trig = False
        self.reciv_p = False
        self.action_active = False 
        
        # Ovdje cemo spremiti listu tocaka koje treba izvrsiti jednu po jednu
        self.execution_queue = [] 

    def load_positions(self, csv_pth):
        try:
            df = pd.read_csv(csv_pth)
            df_relative = df - df.iloc[0]
            p_data = []
            for index, row in df_relative.iterrows():
                x = row['r_index_x'] * 0.00025
                y = row['r_index_y'] * 0.00025
                z = 0.0
                p_data.append(np.array([float(x), float(y), float(z), 1.0]))
            self.get_logger().info(f"Loaded {len(p_data)} positions.")
            return p_data
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV: {e}")
            return []
    
    def trig_cb(self, msg): 
        if msg.data:
            self.reciv_trig = True
            self.get_logger().info("Trigger received!")

    def curr_p_cb(self, msg): 
        self.reciv_p = True
        p = msg.pose.position
        q = msg.pose.orientation
        
        R_ = R.from_quat([q.x, q.y, q.z, q.w])
        r_matrix = R_.as_matrix()
        p_vector = np.array([p.x, p.y, p.z]).reshape(3, 1)
        
        T_top = np.hstack((r_matrix, p_vector))
        self.T = np.vstack((T_top, np.array([0, 0, 0, 1])))
        
        # Spremi frame_id da znamo u kojem frameu saljemo (npr. "base_link" ili "world")
        self.current_frame_id = msg.header.frame_id

    def calculate_trajectory_queue(self): 
        """Racuna sve tocke i puni self.execution_queue"""
        self.get_logger().info("Calculating full trajectory path...")
        self.execution_queue = []
        
        current_quat = R.from_matrix(self.T[:3, :3]).as_quat()

        for p_local in self.c_data:
            p_global = np.matmul(self.T, p_local)

            # Kreiramo Pose objekt
            pose = Pose()
            pose.position.x = p_global[0]
            pose.position.y = p_global[1]
            pose.position.z = p_global[2]
            pose.orientation.x = current_quat[0]
            pose.orientation.y = current_quat[1]
            pose.orientation.z = current_quat[2]
            pose.orientation.w = current_quat[3]
            
            self.execution_queue.append(pose)
            
        self.get_logger().info(f"Prepared {len(self.execution_queue)} waypoints for execution.")

    def start_execution(self):
        """Pocetak izvrsavanja - salje prvu tocku"""
        self.calculate_trajectory_queue()
        self.send_next_goal()

    def send_next_goal(self):
        """Uzima sljedecu tocku iz liste i salje action serveru"""
        
        if not self.execution_queue:
            self.get_logger().info("TRAJECTORY FINISHED! All points reached.")
            self.action_active = False
            return

        # 1. Uzmi sljedecu tocku (i makni je iz liste)
        target_pose = self.execution_queue.pop(0)

        # 2. Provjera servera
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available...')
            return

        # 3. Priprema poruke (PoseStamped)
        # Tvoj interface kaze: geometry_msgs/PoseStamped goal
        goal_msg = MoveCartesian.Goal()
        
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.header.frame_id = self.current_frame_id # Koristimo isti frame kao robot
        goal_msg.goal.pose = target_pose

        self.get_logger().info(f"Moving to point... ({len(self.execution_queue)} remaining)")
        self.action_active = True
        
        # 4. Slanje
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Point rejected by server.')
            self.action_active = False
            return

        # self.get_logger().info('Moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # self.get_logger().info(f'Point reached.')
        
        # OVDJE JE KLJUC: Cim zavrsi jedna tocka, odmah saljemo sljedecu
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        pass

    def run_check(self):
        # Ova funkcija samo provjerava treba li zapoceti proces
        if self.reciv_trig and self.reciv_p and not self.action_active:
            self.reciv_trig = False # Reset triggera
            self.start_execution() # Pokreni lanac
        
        elif not self.action_active:
             # Samo logiranje statusa dok cekamo
             pass

def main(args=None): 
    rclpy.init(args=args)
    CPT = CreateAndPublishTrajectory()
    rclpy.spin(CPT)
    CPT.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()