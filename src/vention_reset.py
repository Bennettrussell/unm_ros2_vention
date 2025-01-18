#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Adjust the import path as needed to find VentionController
from vention_api_wrapper import VentionController, DEFAULT_IP_ADDRESS

class VentionResetNode(Node):
    """
    Ephemeral node that attempts to:
      1) Release e-stop (if triggered)
      2) Reset system (if needed)
    Then shuts itself down.
    """

    def __init__(self):
        super().__init__('vention_reset_node')
        
        # 1. Declare and read a parameter for the MachineMotion IP
        self.declare_parameter('vention_ip', DEFAULT_IP_ADDRESS.usb_windows)
        vention_ip = self.get_parameter('vention_ip').value

        # 2. Instantiate a VentionController (this will connect to MQTT, etc.)
        self.vc = VentionController(ip_address=vention_ip)
        self.get_logger().info(f"Attempting to release e-stop & reset system at IP={vention_ip}")

        # 3. Try releasing e-stop
        try:
            self.get_logger().info("--> Removing software stop")
            success = self.vc.release_estop()
            self.get_logger().info(f"releaseEstop => {success}")
        except Exception as e:
            self.get_logger().error(f"Error while releasing e-stop: {e}")

        # 4. Try resetting system
        try:
            self.get_logger().info("--> Resetting system")
            success = self.vc.reset_system()
            self.get_logger().info(f"resetSystem => {success}")
        except Exception as e:
            self.get_logger().error(f"Error while resetting system: {e}")

        # 5. Since we’re done, we can schedule a shutdown
        self.get_logger().info("Reset node done; shutting down.")
        # We do this in a small timer so the node has time to log
        self.create_timer(1.0, self._shutdown_soon)

    def _shutdown_soon(self):
        self.get_logger().info("Exiting vention_reset_node now.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = VentionResetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt => stopping reset node.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
