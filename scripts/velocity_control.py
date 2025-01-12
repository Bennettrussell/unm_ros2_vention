#!/usr/bin/env python3

"""
ROS 2 version of a velocity controller node for a single MachineMotion gantry.
Subscribes to 'cmd_carriage_velocity' (Float32) and commands the axis velocity.
Publishes 'tracked_carriage_position' and 'tracked_carriage_velocity' (both Float32).
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

# MachineMotion import (assuming it's installed in Python environment)
from MachineMotion import MachineMotionV2OneDrive


class VentionVelocityController(Node):
    def __init__(self):
        super().__init__('velocity_vention_controller')

        # Subscribers
        self.vel_sub = self.create_subscription(
            Float32, 'cmd_carriage_velocity', self.cmd_vel_callback, 10
        )

        # Publishers
        self.tracked_pos_pub = self.create_publisher(Float32, 'tracked_carriage_position', 10)
        self.tracked_vel_pub = self.create_publisher(Float32, 'tracked_carriage_velocity', 10)

        # MachineMotion instance (single gantry)
        self.mm = MachineMotionV2OneDrive(machineIp='192.168.0.10')
        self.get_logger().info('MachineMotion object created for single gantry')

        # Internal variables
        self.axis = 1
        self.tracked_position = 0.0
        self.tracked_velocity = 0.0

        # Flag for shutting down
        self.shutdown_flag = False

        # Log initial state
        self.get_logger().info('VentionVelocityController Node Initialized')

    def cmd_vel_callback(self, msg: Float32):
        """
        Callback for velocity commands.
        Move the gantry at the specified velocity (mm/s).
        """
        if self.shutdown_flag:
            return

        cmd_vel = msg.data
        self.get_logger().info(f'(Heard Velocity {cmd_vel})')

        # Command the single axis motion
        self.mm.moveContinuous(self.axis, cmd_vel)

        # Update tracking (you could do this in a timer, but here we do it on every command)
        self.tracked_velocity = self.mm.getActualSpeeds(self.axis)
        self.tracked_position = self.mm.getActualPositions(self.axis)
        self.publish_tracking_info()

    def publish_tracking_info(self):
        """
        Publish the current position & velocity of the carriage.
        """
        self.get_logger().info(
            f"Publishing position: {self.tracked_position}  velocity: {self.tracked_velocity}"
        )
        pos_msg = Float32()
        vel_msg = Float32()
        pos_msg.data = self.tracked_position
        vel_msg.data = self.tracked_velocity
        self.tracked_pos_pub.publish(pos_msg)
        self.tracked_vel_pub.publish(vel_msg)

    def shutdown(self):
        """
        Stop the axis on shutdown.
        """
        self.shutdown_flag = True
        self.get_logger().info('Stopping carriage and shutting down')
        self.mm.stopMoveContinuous(self.axis)


def main(args=None):
    """
    Entry point: initialize ROS 2, create the node, and spin.
    We also handle KeyboardInterrupt for graceful shutdown.
    """
    rclpy.init(args=args)
    node = VentionVelocityController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down node.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
