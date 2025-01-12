#!/usr/bin/env python3

"""
ROS 2 version of a position controller node for a single MachineMotion gantry.
Subscribes to 'cmd_carriage_position' (Float32) and moves the axis to that position.
Publishes 'tracked_carriage_position' and 'tracked_carriage_velocity' (Float32).
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from MachineMotion import MachineMotionV2OneDrive


class VentionPositionController(Node):
    def __init__(self):
        super().__init__('position_vention_controller')

        # Subscribers
        self.pos_sub = self.create_subscription(
            Float32, 'cmd_carriage_position', self.cmd_pos_callback, 10
        )

        # Publishers
        self.tracked_pos_pub = self.create_publisher(Float32, 'tracked_carriage_position', 10)
        self.tracked_vel_pub = self.create_publisher(Float32, 'tracked_carriage_velocity', 10)

        # MachineMotion instance (single axis)
        self.mm = MachineMotionV2OneDrive()
        self.get_logger().info('MachineMotion object created for single gantry')

        self.axis = 1
        self.tracked_position = 0.0
        self.tracked_velocity = 0.0
        self.shutdown_flag = False

        self.get_logger().info('VentionPositionController Node Initialized')

    def cmd_pos_callback(self, msg: Float32):
        """
        Callback for position commands.
        Move the gantry to the specified position (mm).
        """
        if self.shutdown_flag:
            return

        position_cmd = msg.data
        self.get_logger().info(f'(Heard Position {position_cmd})')

        # Move the axis to the requested position
        self.mm.moveToPosition(self.axis, position_cmd)

        # Track and publish
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
        Stop motion on shutdown if needed.
        """
        self.shutdown_flag = True
        self.get_logger().info('Stopping carriage and shutting down')
        self.mm.stopMoveContinuous(self.axis)


def main(args=None):
    rclpy.init(args=args)
    node = VentionPositionController()

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
