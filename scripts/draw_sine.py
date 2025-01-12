#!/usr/bin/env python3

"""
ROS 2 version of a node that publishes a sine wave velocity command
on 'cmd_carriage_velocity' (Float32) at 50 Hz.
"""

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float32


class SineVelocityPublisher(Node):
    def __init__(self):
        super().__init__('command_sine_vel')

        # Params for sine wave
        self.amplitude = 100.0  # mm/s
        self.freq = 0.06        # Hz

        # Create publisher
        self.pub = self.create_publisher(Float32, 'cmd_carriage_velocity', 10)

        # Create timer at 50 Hz
        self.timer_frequency = 50.0
        self.timer_period = 1.0 / self.timer_frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Store the start time
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.get_logger().info('SineVelocityPublisher node has started.')

    def timer_callback(self):
        """
        Runs at 50 Hz, publishes the next sine wave velocity command.
        """
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        cmd_vel = self.amplitude * np.sin(2.0 * np.pi * self.freq * elapsed_time)

        vel_msg = Float32()
        vel_msg.data = cmd_vel

        self.get_logger().info(f'Published velocity: {cmd_vel}')
        self.pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SineVelocityPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down sine publisher.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
