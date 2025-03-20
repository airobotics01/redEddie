#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import numpy as np
import threading
import time


class FR3HomePositionService(Node):
    def __init__(self):
        super().__init__("fr3_home_position_service")

        # Define home position for the 7 arm joints
        self.home_position = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        # Define finger positions
        self.finger_position = [
            0.4,
            0.4,
        ]  # Matching the default_joints in the working code

        # Full joint positions (arm + fingers)
        self.full_home_position = self.home_position + self.finger_position

        # Joint names - must include all 9 joints as in the working code
        self.joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
            "fr3_finger_joint1",
            "fr3_finger_joint2",
        ]

        # Tolerance for considering a joint at home position (in radians)
        self.position_tolerance = 0.05

        # Current joint positions
        self.current_positions = [0.0] * len(self.joint_names)

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        # Publisher for joint commands - using same topic as working code
        self.publisher = self.create_publisher(JointState, "joint_command", 10)

        # Create the service
        self.service = self.create_service(Trigger, "go_home", self.go_home_callback)

        # Flag to track if we're currently in home mode
        self.going_home = False

        # Timer for continuous publishing when going home
        self.timer = None

        self.get_logger().info("FR3 Home Position Service initialized")

    def joint_states_callback(self, msg):
        """Process incoming joint states"""
        # Update current positions
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_positions[idx] = msg.position[i]

    def is_at_home(self):
        """Check if robot arm joints are at home position within tolerance"""
        for i in range(len(self.home_position)):
            if (
                abs(self.current_positions[i] - self.home_position[i])
                > self.position_tolerance
            ):
                return False
        return True

    def publish_home_command(self):
        """Publish home position command"""
        # Create a JointState message
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = self.joint_names
        cmd_msg.position = self.full_home_position

        # Publish the message
        self.publisher.publish(cmd_msg)

        # Check if we've reached home position
        if self.is_at_home():
            self.get_logger().info("Robot has reached home position")
            self.going_home = False
            self.timer.cancel()
            self.timer = None

    def go_home_callback(self, request, response):
        """Service callback to send robot to home position"""
        if self.is_at_home():
            response.success = True
            response.message = "Robot is already at home position"
            self.get_logger().info("Robot is already at home position")
        elif self.going_home:
            response.success = False
            response.message = "Robot is already in the process of moving home"
            self.get_logger().info("Already moving home, ignoring duplicate request")
        else:
            # Start continuous publishing to home position
            self.going_home = True

            # Use a timer to continuously publish at 20Hz (like the working example)
            timer_period = 0.05  # seconds (20Hz)
            self.timer = self.create_timer(timer_period, self.publish_home_command)

            response.success = True
            response.message = "Robot is moving to home position"
            self.get_logger().info("Started movement to home position")

        return response


def main(args=None):
    rclpy.init(args=args)

    service = FR3HomePositionService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
