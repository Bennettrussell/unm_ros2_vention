#!/usr/bin/env python3

"""
ROS 2 version of a node that publishes a sequence of carriage positions (Float32)
to the 'cmd_carriage_position' topic.
It generates positions between start_pos and end_pos, forward and back.
"""

import rclpy
from rclpy.node import Node

import numpy as np
from std_msgs.msg import Float32


def generate_positions(start_pos, end_pos, duration, rate_hz):
    """
    Generate a forward/backward array of positions.
    Forward: from start_pos to end_pos over 'duration' seconds.
    Then backward: from end_pos back to start_pos.
    """
    step_count = int(duration * rate_hz)
    forward = np.linspace(start_pos, end_pos, step_count)
    backward = forward[::-1]  # reversed
    return np.concatenate([forward, backward])


class DefinePositionPublisher(Node):
    def __init__(self):
        super().__init__('command_position')

        # Params
        self.rate_hz = 100.0
        self.start_pos = -50.0
        self.end_pos = -300.0
        self.duration = 10.0  # seconds (for forward leg)

        # Create publisher
        self.pub = self.create_publisher(Float32, 'cmd_carriage_position', 10)

        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz or any pace
        self.get_logger().info('DefinePositionPublisher node started.')

        # Pre-generate the positions we’ll publish
        self.positions = generate_positions(
            self.start_pos, self.end_pos, self.duration, self.rate_hz
        )
        self.index = 0

    def timer_callback(self):
        """
        Periodically publish the next position in the pre-generated list.
        Loops when done.
        """
        if self.index >= len(self.positions):
            # Reset index to loop again (or you could stop the timer if you prefer)
            self.index = 0

        current_pos = self.positions[self.index]
        self.index += 1

        msg = Float32()
        msg.data = float(current_pos)
        self.get_logger().info(f'Published position: {current_pos}')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DefinePositionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
