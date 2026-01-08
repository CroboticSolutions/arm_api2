#!/usr/bin/env python3

import copy
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from arm_api2_msgs.srv import ChangeState, SetVelAcc, SetStringParam, AddGraspedObject, AddGraspedObject
import std_srvs
from std_srvs.srv import Empty
from std_msgs.msg import String

class ArmCommander(Node):

    def __init__(self):
        super().__init__('arm_commander')

        # Initialize state variables
        self.current_arm_pose = None
        self.previous_arm_pose = None
        # State of the state machine 
        self.state_index = 0
        self.state = "INIT"
        # State of the robot arm controller
        self.arm_state = "IDLE"
        self.pose_commanded = False
        self.action_executed = False  # Track if action for current state was executed
        self.run_ready = False
        self.state_change_requested = False

        # Stall detection configuration
        self.max_stall_steps = 10  # Stall is 10 seconds
        self.stall_counter = 0
        self.pose_change_threshold = 0.001  # Minimum change in position to consider movement

        # Define pick and place sequence - easy to modify and extend
        self.pick_place_sequence = self._define_sequence()
        
        # Initialize ROS components
        self._init_publishers()
        self._init_subscribers()
        self._init_service_clients()
        
        # Create timer for state machine loop (100ms = 10Hz)
        self.timer = self.create_timer(1, self.timer_callback)
        
        self.get_logger().info('Arm Commander initialized')
        self.get_logger().info(f'Loaded sequence with {len(self.pick_place_sequence)} states')

    def _define_sequence(self):
        """Define the pick and place sequence as a list of state dictionaries"""


        # Stud rack positions
        pregrasp0 = Point(x=-0.0486634344967895, y=2.0899579307096596, z=1.8285165453258871)
        orientation0 = Quaternion(x=0.1976338642990086, y=-0.18551426943529198, z=0.6733863658426806, w=0.6878052877198012)
        
        grasp0 = Point(x=-0.041755877664185204, y=2.2799795008662858, z=1.517542817586289)
        orientation0_grasp = Quaternion(x=0.19755672095410443, y=-0.1855423803022532, z=0.6733951365670418, w=0.6878112801964587)
        
        lift0 = Point(x=pregrasp0.x, y=pregrasp0.y, z=pregrasp0.z)  # Same as pregrasp0
        predrop0 = Point(x=lift0.x, y=0.0, z=lift0.z)
        drop0 = Point(x=predrop0.x, y=-0.05, z=grasp0.z)  # Drop at y=-0.05
                                  

        # Base positions for first board
        pregrasp1 = Point(x=2.35, y=1.4313275801597196, z=1.50)
        grasp1 = Point(x=pregrasp1.x, y=pregrasp1.y, z=pregrasp1.z - 0.11)
        lift1 = Point(x=grasp1.x, y=grasp1.y, z=grasp1.z + 0.5)
        predrop1 = Point(x=lift1.x, y=0.0, z=lift1.z)
        drop1 = Point(x=predrop1.x, y=predrop1.y, z=grasp1.z)
        
        # Base positions for second board (-0.15 on y-axis from first board)
        pregrasp2 = Point(x=2.35, y=1.2813275801597196, z=1.50)
        grasp2 = Point(x=pregrasp2.x, y=pregrasp2.y, z=pregrasp2.z - 0.11)
        lift2 = Point(x=grasp2.x, y=grasp2.y, z=grasp2.z + 0.5)
        predrop2 = Point(x=lift2.x, y=-0.1, z=lift2.z)
        drop2 = Point(x=predrop2.x, y=predrop2.y, z=grasp2.z)
        
        orientation = Quaternion(x=0.0, y=-0.0, z=0.0, w=1.0)

        sequence = [
            # Stud rack pickup
            {
                'name': 'PREGRASP0',
                'type': 'move',
                'position': pregrasp0,
                'orientation': orientation0,
                'control_mode': 'JOINT_TRAJ_CTL',
                'planner': None,
                'vel_acc': None,
                'description': 'Move to pregrasp position for stud from rack'
            },
            {
                'name': 'GRASP0',
                'type': 'move_and_grasp',
                'position': grasp0,
                'orientation': orientation0_grasp,
                'control_mode': 'CART_TRAJ_CTL',
                'gripper_action': 'close',
                'wait_after': 10.0,
                'vel_acc': None,
                'description': 'Grasp stud from rack'
            },
            {
                'name': 'LIFT0',
                'type': 'move',
                'position': lift0,
                'orientation': orientation0,
                'control_mode': None,
                'vel_acc': (0.05, 0.05),  # Slow for lifting
                'description': 'Lift stud'
            },
            {
                'name': 'PREDROP0',
                'type': 'move',
                'position': predrop0,
                'orientation': orientation0,
                'control_mode': None,
                'planner': 'pilz_LIN',  # Linear motion for safety
                'wait_before': 0.5,
                'description': 'Move to drop location for stud'
            },
            {
                'name': 'DROP0',
                'type': 'move',
                'position': drop0,
                'orientation': orientation,
                'control_mode': None,
                'vel_acc': (0.05, 0.05),  # Slow for lowering (same as lift)
                'description': 'Lower stud to drop height'
            },
            {
                'name': 'RELEASE0',
                'type': 'gripper',
                'gripper_action': 'open',
                'wait_after': 3.0,
                'vel_acc_after': (0.5, 0.5),  # Reset to normal speed after release
                'description': 'Release stud'
            },
            {
                'name': 'POSTDROP0',
                'type': 'move',
                'position': predrop0,  # Lift back up to predrop height
                'orientation': orientation0,
                'control_mode': None,
                'description': 'Lift after releasing stud'
            },
            # First board pickup
            {
                'name': 'PREGRASP1',
                'type': 'move',
                'position': pregrasp1,
                'orientation': orientation,
                'control_mode': 'JOINT_TRAJ_CTL',
                'planner': None,
                'vel_acc': None,
                'description': 'Move to pregrasp position for board 1'
            },
            {
                'name': 'GRASP1',
                'type': 'move_and_grasp',
                'position': grasp1,
                'orientation': orientation,
                'control_mode': 'CART_TRAJ_CTL',
                'gripper_action': 'close',
                'wait_after': 7.0,
                'vel_acc': None,
                'description': 'Grasp board 1'
            },
            {
                'name': 'LIFT1',
                'type': 'move',
                'position': lift1,
                'orientation': orientation,
                'control_mode': None,
                'vel_acc': (0.05, 0.05),  # Slow for lifting
                'description': 'Lift board 1'
            },
            {
                'name': 'PREDROP1',
                'type': 'move',
                'position': predrop1,
                'orientation': orientation,
                'control_mode': None,
                'planner': 'pilz_LIN',  # Linear motion for safety
                'wait_before': 0.5,
                'description': 'Move to drop location (y=0)'
            },
            {
                'name': 'DROP1',
                'type': 'move',
                'position': drop1,
                'orientation': orientation,
                'control_mode': None,
                'vel_acc': (0.05, 0.05),  # Slow for lowering (same as lift)
                'description': 'Lower to drop height for board 1'
            },
            {
                'name': 'RELEASE1',
                'type': 'gripper',
                'gripper_action': 'open',
                'wait_after': 3.0,
                'vel_acc_after': (0.5, 0.5),  # Reset to normal speed after release
                'description': 'Release board 1'
            },
            {
                'name': 'POSTDROP1',
                'type': 'move',
                'position': predrop1,  # Lift back up to predrop height
                'orientation': orientation,
                'control_mode': None,
                'description': 'Lift after releasing board 1'
            },
            # Second board pickup
            {
                'name': 'PREGRASP2',
                'type': 'move',
                'position': pregrasp2,
                'orientation': orientation,
                'control_mode': None,
                'description': 'Move to pregrasp position for board 2'
            },
            {
                'name': 'GRASP2',
                'type': 'move_and_grasp',
                'position': grasp2,
                'orientation': orientation,
                'control_mode': None,
                'gripper_action': 'close',
                'wait_after': 5.0,
                'description': 'Grasp board 2'
            },
            {
                'name': 'LIFT2',
                'type': 'move',
                'position': lift2,
                'orientation': orientation,
                'control_mode': None,
                'vel_acc': (0.05, 0.05),
                'description': 'Lift board 2'
            },
            {
                'name': 'PREDROP2',
                'type': 'move',
                'position': predrop2,
                'orientation': orientation,
                'control_mode': None,
                'planner': 'pilz_LIN',
                'wait_before': 0.5,
                'description': 'Move to drop location for board 2'
            },
            {
                'name': 'DROP2',
                'type': 'move',
                'position': drop2,
                'orientation': orientation,
                'control_mode': None,
                'vel_acc': (0.05, 0.05),  # Slow for lowering (same as lift)
                'description': 'Lower to drop height for board 2'
            },
            {
                'name': 'RELEASE2',
                'type': 'gripper',
                'gripper_action': 'open',
                'wait_after': 3.0,
                'vel_acc_after': (0.5, 0.5),  # Reset to normal speed after release
                'description': 'Release board 2'
            },
            {
                'name': 'POSTDROP2',
                'type': 'move',
                'position': predrop2,  # Lift back up to predrop height
                'orientation': orientation,
                'control_mode': None,
                'description': 'Lift after releasing board 2'
            },
            {
                'name': 'COMPLETE',
                'type': 'end',
                'description': 'Sequence complete'
            }
        ]
        
        return sequence

    def _init_publishers(self):
        """Initialize all publishers"""
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/arm/cmd/pose', 10
        )

    def _init_subscribers(self):
        """Initialize all subscribers"""
        self.current_pose_subscriber = self.create_subscription(
            PoseStamped, '/arm/state/current_pose', self.current_pose_callback, 1
        )
        self.state_sub = self.create_subscription(
            String, 
            '/arm/state/ctl_state', 
            self.robot_state_callback, 
            10
        )

    def _init_service_clients(self):
        """Initialize all service clients"""
        self.arm_change_state_client = self.create_client(ChangeState, '/arm/change_state')
        self.set_vel_acc_client = self.create_client(SetVelAcc, '/arm/set_vel_acc')
        self.set_planner_client = self.create_client(SetStringParam, '/arm/set_planner')
        self.open_gripper_client = self.create_client(Empty, '/gripper/open')
        self.close_gripper_client = self.create_client(Empty, '/gripper/close')
        self.add_grasped_object_client = self.create_client(AddGraspedObject, '/arm/add_grasped_object')
        
    def current_pose_callback(self, msg):
        """Callback to receive current arm pose"""
        self.previous_arm_pose = copy.deepcopy(self.current_arm_pose)
        self.current_arm_pose = msg.pose


    def robot_state_callback(self, msg):
        self.arm_state = msg.data
        self.get_logger().info(f'Robot state: {msg.data}')
    # msg.data will be one of: "IDLE", "JOINT_TRAJ_CTL", "CART_TRAJ_CTL", "SERVO_CTL"

    def _get_elapsed_time(self, start_time):
        """Get elapsed time in seconds from a start time
        
        Args:
            start_time: rclpy.time.Time object
            
        Returns:
            float: Elapsed time in seconds
        """
        return (self.get_clock().now() - start_time).nanoseconds / 1e9
    
    def is_pose_changing(self):
        """Check if the robot pose is changing (not stalled)
        
        Returns:
            bool: True if pose is changing, False if stalled
        """
        if self.current_arm_pose is None or self.previous_arm_pose is None:
            return True  # Can't determine, assume moving
        
        # Calculate position difference between current and previous pose
        pos_diff = ((self.current_arm_pose.position.x - self.previous_arm_pose.position.x)**2 + 
                   (self.current_arm_pose.position.y - self.previous_arm_pose.position.y)**2 + 
                   (self.current_arm_pose.position.z - self.previous_arm_pose.position.z)**2)**0.5
        
        return pos_diff > self.pose_change_threshold
    
    def check_for_stall(self, state_name):
        """Check if robot has stalled and handle failure
        Args:
            state_name: Name of the current state for error reporting
        Returns:
            bool: True if stalled and should fail, False if still moving
        """
        if not self.is_pose_changing():
            self.stall_counter += 1
            if self.stall_counter >= self.max_stall_steps:
                self.get_logger().error(f'>>> Robot stalled at {state_name} (no movement for {self.stall_counter} steps). Exiting.')
                self.state = f'FAILED_AT_{state_name}'
                return True
        else:
            # Reset counter if pose is changing
            self.stall_counter = 0
        
        return False
    
    def poses_are_close(self, pose1, pose2, position_tolerance=0.01):
        """Check if two poses are close within tolerance"""
        if pose1 is None or pose2 is None:
            return False
            
        # Check position difference
        pos_diff = ((pose1.x - pose2.x)**2 + 
                   (pose1.y - pose2.y)**2 + 
                   (pose1.z - pose2.z)**2)**0.5

        self.get_logger().info(f"Checking pose closeness: {pos_diff:.3f}")

        if pos_diff < position_tolerance: 
            return True
        else: 
            return False

    
    def move_to_pose(self, position, orientation, description="target"):
        """Move robot to target pose by publishing to /arm/cmd/pose topic"""
        self.get_logger().info(f'Commanding {description}: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}')
        
        # Create PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position = position
        pose_stamped.pose.orientation = orientation
        
        # Publish the pose command
        self.pose_publisher.publish(pose_stamped)
    
    def change_scaling_factors(self, vel_scaling, acc_scaling):
        """Change the velocity and acceleration scaling factors of the arm controller"""
        self.get_logger().info(f'Changing scaling factors: velocity={vel_scaling}, acceleration={acc_scaling}')
        
        if not self.set_vel_acc_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service /arm/set_vel_acc not available')
            return False

        request = SetVelAcc.Request()
        request.max_vel = vel_scaling
        request.max_acc = acc_scaling
        result = self.set_vel_acc_client.call_async(request)
        return result

    def set_planner(self, planner_name):
        """Set the motion planner (e.g., 'pilz_LIN', 'ompl_RRT')"""
        self.get_logger().info(f'Setting planner to: {planner_name}')
        
        if not self.set_planner_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service /arm/set_planner not available')
            return False

        request = SetStringParam.Request()
        request.value = planner_name
        result = self.set_planner_client.call_async(request)
        return result

    def change_arm_state(self, state):
        """Change arm controller state to wanted state"""
        if not self.arm_change_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service /arm/change_state not available')
            return False
        
        request = ChangeState.Request()
        request.state = state
        # Simple and dummy service call under the assumption that everything goes well
        result = self.arm_change_state_client.call_async(request)
        return result

    def open_gripper(self):
        """Open the gripper"""
        self.get_logger().info('Opening gripper...')

        request = Empty.Request()
        result = self.open_gripper_client.call_async(request)

        return result

    def close_gripper(self):
        """Close the gripper"""
        self.get_logger().info('Closing gripper...')

        request = Empty.Request()
        result = self.close_gripper_client.call_async(request)

        return result

    def timer_callback(self):
        """Generalized timer callback for state machine - runs at 1Hz"""
        self.get_logger().info(f'State: {self.state_index}/{len(self.pick_place_sequence)-1} - {self.state}')
        
        # Step 1: Initialize - Change to JOINT_TRAJ_CTL mode
        if self.arm_state == 'IDLE':
            self.change_arm_state('JOINT_TRAJ_CTL')
            return
        
        # Check if sequence is complete
        if self.state_index >= len(self.pick_place_sequence):
            self.get_logger().info('>>> SEQUENCE COMPLETE <<<')
            return
        
        # Get current state configuration
        current_state = self.pick_place_sequence[self.state_index]
        self.state = current_state['name']
        
        # Execute state based on type
        if current_state['type'] == 'move':
            self._execute_move_state(current_state)
        elif current_state['type'] == 'move_and_grasp':
            self._execute_move_and_grasp_state(current_state)
        elif current_state['type'] == 'gripper':
            self._execute_gripper_state(current_state)
        elif current_state['type'] == 'end':
            self.get_logger().info(f">>> {current_state['description']} <<<")
            self.state_index += 1
    
    def _execute_move_state(self, state_config):
        """Execute a move state"""
        # Execute once per state
        if not self.action_executed:
            self.stall_counter = 0
            
            # Change control mode if specified
            if state_config.get('control_mode') and self.arm_state != state_config['control_mode']:
                self.change_arm_state(state_config['control_mode'])
                return
            
            # Wait before if specified
            if state_config.get('wait_before'):
                time.sleep(state_config['wait_before'])
            
            # Change planner if specified
            if state_config.get('planner'):
                self.set_planner(state_config['planner'])
                time.sleep(0.5)
            
            # Change velocity/acceleration if specified
            if state_config.get('vel_acc'):
                vel, acc = state_config['vel_acc']
                self.change_scaling_factors(vel, acc)
            
            # Command the movement
            self.move_to_pose(state_config['position'], state_config['orientation'], 
                            description=state_config['description'])
            self.action_executed = True
            self.pose_commanded = True
        
        # Check if position reached
        position_reached = self.poses_are_close(state_config['position'], self.current_arm_pose.position)
        if position_reached:
            self.get_logger().info(f">>> Position reached: {state_config['name']}")
            self.pose_commanded = False
            self.action_executed = False
            self.state_index += 1
        elif self.check_for_stall(state_config['name']):
            exit()
    
    def _execute_move_and_grasp_state(self, state_config):
        """Execute a move state followed by gripper action"""
        # Execute once per state
        if not self.action_executed:
            self.stall_counter = 0
            
            # Change control mode if specified
            if state_config.get('control_mode') and self.arm_state != state_config['control_mode']:
                self.change_arm_state(state_config['control_mode'])
                return
            
            # Command the movement
            self.move_to_pose(state_config['position'], state_config['orientation'],
                            description=state_config['description'])
            self.action_executed = True
            self.pose_commanded = True
        
        # Check if position reached
        position_reached = self.poses_are_close(state_config['position'], self.current_arm_pose.position)
        if position_reached:
            self.get_logger().info(f">>> Position reached: {state_config['name']}")
            
            # Execute gripper action
            if state_config.get('gripper_action') == 'close':
                self.get_logger().info('>>> Closing gripper...')
                self.close_gripper()
                # Add collision object representing the grasped stud (approximately 8mm diameter cylinder)
                self.add_stud_collision_object()
            elif state_config.get('gripper_action') == 'open':
                self.get_logger().info('>>> Opening gripper...')
                self.open_gripper()
            
            # Wait if specified
            if state_config.get('wait_after'):
                time.sleep(state_config['wait_after'])
            
            # Change velocity for next moves if specified
            if state_config.get('vel_acc_after'):
                vel, acc = state_config['vel_acc_after']
                self.change_scaling_factors(vel, acc)
            elif state_config.get('gripper_action') == 'close':
                # Default: slow down after grasping
                self.change_scaling_factors(0.05, 0.05)
            
            self.pose_commanded = False
            self.action_executed = False
            self.state_index += 1
        elif self.check_for_stall(state_config['name']):
            exit()
    
    def _execute_gripper_state(self, state_config):
        """Execute a gripper-only state"""
        if not self.action_executed:
            self.get_logger().info(f">>> {state_config['description']}")
            
            if state_config.get('gripper_action') == 'close':
                self.close_gripper()
            elif state_config.get('gripper_action') == 'open':
                self.open_gripper()
            
            # Wait if specified
            if state_config.get('wait_after'):
                time.sleep(state_config['wait_after'])
            
            # Change velocity/acceleration after action if specified
            if state_config.get('vel_acc_after'):
                vel, acc = state_config['vel_acc_after']
                self.change_scaling_factors(vel, acc)
                self.get_logger().info(f"Reset velocity/acceleration to {vel}/{acc}")
            
            self.action_executed = False
            self.state_index += 1

    def add_stud_collision_object(self):
        """Add a collision object representing the grasped wooden stud (2.00m x 0.15m x 0.03m box)"""
        try:
            request = AddGraspedObject.Request()
            
            # Create collision object for the stud
            collision_object = CollisionObject()
            collision_object.id = "grasped_stud"
            collision_object.header.frame_id = "upper_right_finger"
            collision_object.operation = CollisionObject.ADD
            
            # Define stud as a box with dimensions from Gazebo model: 2.00 x 0.15 x 0.03
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [2.00, 0.15, 0.03]  # [x, y, z] dimensions in meters
            
            # Position the stud centered on the finger
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)
            
            # Create attached collision object
            attached_object = AttachedCollisionObject()
            attached_object.link_name = "upper_right_finger"
            attached_object.object = collision_object
            
            request.grasped_object = collision_object
            request.attach_object = attached_object
            
            # Send request
            future = self.add_grasped_object_client.call_async(request)
            
            # Note: We don't wait for response here to avoid blocking
            self.get_logger().info('>>> Wooden stud collision object added to upper_right_finger (2.00m x 0.15m x 0.03m)')
            
        except Exception as e:
            self.get_logger().error(f'Failed to add stud collision object: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    commander = ArmCommander()

    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
