#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FR3JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('fr3_joint_state_subscriber')
        
        # List of FR3 joint names to track
        self.fr3_joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
            'fr3_finger_joint1', 'fr3_finger_joint2'
        ]
        
        # Create a subscriber for joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        
        # Initialize dictionaries to store the latest joint data
        self.joint_positions = {joint: 0.0 for joint in self.fr3_joint_names}
        self.joint_velocities = {joint: 0.0 for joint in self.fr3_joint_names}
        self.joint_efforts = {joint: 0.0 for joint in self.fr3_joint_names}
        
        self.get_logger().info('FR3 Joint State Subscriber initialized')
    
    def joint_states_callback(self, msg):
        """Process incoming joint state messages"""
        # Update the joint data dictionaries
        for i, name in enumerate(msg.name):
            if name in self.fr3_joint_names:
                self.joint_positions[name] = msg.position[i]
                self.joint_velocities[name] = msg.velocity[i]
                self.joint_efforts[name] = msg.effort[i]
        
        # Log the arm joint positions (excluding finger joints)
        arm_joints = [joint for joint in self.fr3_joint_names if 'finger' not in joint]
        self.get_logger().info('FR3 Arm Joint Positions:')
        for joint in arm_joints:
            self.get_logger().info(f'  {joint}: {self.joint_positions[joint]:.4f} rad')
        
        # Log the gripper joint positions
        finger_joints = [joint for joint in self.fr3_joint_names if 'finger' in joint]
        self.get_logger().info('FR3 Finger Joint Positions:')
        for joint in finger_joints:
            self.get_logger().info(f'  {joint}: {self.joint_positions[joint]:.4f} rad')


def main(args=None):
    rclpy.init(args=args)
    
    fr3_subscriber = FR3JointStateSubscriber()
    
    try:
        rclpy.spin(fr3_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        fr3_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()