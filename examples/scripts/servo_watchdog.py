#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import String
from rclpy.time import Time

class ServoWatchdog(Node):
    def __init__(self):
        super().__init__('servo_watchdog_node')

        # Create a publisher for the twist topic with zero velocity
        self.pub_twist = self.create_publisher(TwistStamped, '/moveit2_iface_node/delta_twist_cmds', 10)
        self.pub_jog = self.create_publisher(JointJog, '/moveit2_iface_node/delta_joint_cmds', 10)

        # Create a subscriber for the twist topic to check if messages are being published
        self.sub_twist = self.create_subscription(
            TwistStamped,
            '/moveit2_iface_node/delta_twist_cmds',
            self.twist_callback,
            10
        )
        
        self.sub_jog = self.create_subscription(
            JointJog,
            '/moveit2_iface_node/delta_joint_cmds',
            self.jog_callback,
            10
        )
        self.sub_arm_api2_state = self.create_subscription(
            String,
            '/arm/state/current_state',
            self.arm_api2_state_callback,
            10
        )

        # Set a timer to publish a zero velocity twist message every second
        self.timer = self.create_timer(1.0, self.publish_zero_twist)

        # Flag to track whether we received a message
        self.received_message = False
        self.joint_names = []
        self.watchdog_active = False
        
        self.get_logger().info('Servo watchdog node started')

    def twist_callback(self, msg):
        """Callback that gets triggered when a new Twist message is received."""
        #self.get_logger().info('Received Twist message!')
        self.received_message = True  # Message received, reset the flag
    
    def jog_callback(self, msg):
        """Callback that gets triggered when a new Twist message is received."""
        #self.get_logger().info('Received Twist message!')
        print("Joints: ", msg.joint_names)
        self.joint_names = msg.joint_names
        self.received_message = True  # Message received, reset the flag
    
    def arm_api2_state_callback(self, msg):
        """Callback that gets triggered when a new Twist message is received."""
        #self.get_logger().info('Received Twist message!')
        if msg.data == "SERVO_CTL":
            if self.watchdog_active == False:
                self.get_logger().info('arm_api2 is in SERVO_CTL mode, starting watchdog')
            self.watchdog_active = True
        else:
            if self.watchdog_active == True:
                self.get_logger().info('arm_api2 is not in SERVO_CTL mode anymore, stopping watchdog')
            self.watchdog_active = False

    def publish_zero_twist(self):
        """Publish a zero velocity twist message if no message was received in the last second."""
        if not self.received_message and self.watchdog_active:
            self.get_logger().info('No Twist/JointJog message received, publishing zero velocity message.')
            zero_twist = TwistStamped()
            zero_twist.header.stamp = Time().to_msg()
            zero_twist.header.frame_id = 'world'
            zero_twist.twist.linear.x = 0.0
            zero_twist.twist.linear.y = 0.0
            zero_twist.twist.linear.z = 0.0
            zero_twist.twist.angular.x = 0.0
            zero_twist.twist.angular.y = 0.0    
            zero_twist.twist.angular.z = 0.0
            self.pub_twist.publish(zero_twist)  # Publish zero velocity
            
            zero_jog = JointJog()
            zero_jog.header.stamp = Time().to_msg()
            zero_jog.joint_names = self.joint_names
            zero_jog.velocities = [0.0] * len(self.joint_names)
            self.pub_jog.publish(zero_jog)
        else:
            # Reset the flag after publishing zero twist
            self.received_message = False


def main(args=None):
    rclpy.init(args=args)
    node = ServoWatchdog()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
