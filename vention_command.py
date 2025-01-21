#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class VentionControlLoop(Node):
    """
    A ROS 2 node that performs a simple closed-loop control on Drive #1:
      - Subscribes to 'drive_1_position' to get the current position (mm).
      - Publishes velocities to 'drive_1_vel_cmd' to move toward a target position.

    This demonstrates how to do real-time control of a single axis using rclpy.
    """

    def __init__(self):
        super().__init__('vention_control_loop')
        
        # Declare a parameter for the target position
        self.declare_parameter('target_position', 100.0)  # default 100 mm
        # A simple P-control gain
        self.declare_parameter('p_gain', 0.5)            # mm/s per mm of error
        # Maximum velocity limit
        self.declare_parameter('max_vel', 100.0)         # mm/s
        # Tolerance for stopping
        self.declare_parameter('tolerance', 2.0)         # mm
        # Rate for the control loop
        self.declare_parameter('control_rate', 5.0)      # Hz (updates per second)

        self.target_position = self.get_parameter('target_position').value
        self.p_gain          = self.get_parameter('p_gain').value
        self.max_vel         = self.get_parameter('max_vel').value
        self.tolerance       = self.get_parameter('tolerance').value
        self.control_rate    = self.get_parameter('control_rate').value

        # Current measured position
        self.current_position = 0.0
        # Are we close enough to target to stop?
        self.reached_target = False

        # Create subscriber to read the drive_1_position topic
        # (published by vention_node)
        self.sub_position = self.create_subscription(
            Float32,
            'drive_1_position',
            self.position_callback,
            10
        )
        
        # Create publisher to send velocity commands to drive_1_vel_cmd
        self.pub_velocity = self.create_publisher(Float32, 'drive_1_vel_cmd', 10)
        
        # Create a control loop timer
        timer_period_s = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period_s, self.control_loop)

        self.get_logger().info(
            f"VentionControlLoop started. target={self.target_position} mm, "
            f"p_gain={self.p_gain}, max_vel={self.max_vel}, tol={self.tolerance}, "
            f"rate={self.control_rate} Hz"
        )

    def position_callback(self, msg: Float32):
        # Save the latest position reading
        self.current_position = msg.data

    def control_loop(self):
        """
        Runs at 'control_rate' Hz. Simple P-control: velocity = p_gain * (target - current).
        Clamps velocity to +/- max_vel. Stops if within tolerance.
        """
        error = self.target_position - self.current_position

        if abs(error) <= self.tolerance:
            # If we are close enough, stop motion
            self.stop_drive()
            if not self.reached_target:
                self.reached_target = True
                self.get_logger().info(
                    f"[Drive1] Reached target {self.target_position} mm (error={error:.1f} mm)."
                )
            # Could optionally shut down the node here if you want to end the script:
            # self.shutdown_and_exit()
            return

        # We haven't reached target, so compute velocity
        velocity = self.p_gain * error
        # Clamp to max velocity
        if velocity > 0:
            velocity = min(velocity, self.max_vel)
        else:
            velocity = max(velocity, -self.max_vel)

        # Publish velocity command
        vel_msg = Float32()
        vel_msg.data = float(velocity)
        self.pub_velocity.publish(vel_msg)

        self.get_logger().debug(
            f"[Control] error={error:.1f}, vel={velocity:.1f}, pos={self.current_position:.1f}"
        )

    def stop_drive(self):
        """Publishes a 0.0 velocity to stop drive #1."""
        vel_msg = Float32()
        vel_msg.data = 0.0
        self.pub_velocity.publish(vel_msg)

    def shutdown_and_exit(self):
        """
        If you want to end the node once the target is reached, you can call this
        method from control_loop. Then the node will stop drive motion and exit.
        """
        self.stop_drive()
        self.get_logger().info("Target reached -> shutting down control loop node.")
        # The next line ensures we schedule a normal node shutdown
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VentionControlLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt -> Stopping drive #1.")
        node.stop_drive()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
