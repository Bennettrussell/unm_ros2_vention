#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String, Int32MultiArray, Bool

# Import your completed wrapper
from vention_api_wrapper import (
    VentionController,
    MECH_GAIN,
    DIRECTION,
    MICRO_STEPS,
    POWER_SWITCH,
    PUSH_BUTTON,
    BRAKE_STATES
)

class VentionNode(Node):
    """
    A comprehensive ROS 2 node that uses the VentionController class
    to interface with a MachineMotion v2 or v2OneDrive system.

    Features:
      1. Parameter-based IP configuration (vention_ip, is_onedrive, etc.)
      2. Subscriptions for:
         - Velocity commands
         - Position commands
         - E-stop commands
         - Brake commands
         - Power-switch commands
         - Homing commands (home one axis or all)
         - Speed/Acceleration set commands (global)
         - Set absolute or relative axis positions
         - Digital IO write commands
      3. Periodic position publishing
      4. Timer-based “continuous velocity” re-application
      5. Cleanup on shutdown
    """

    def __init__(self):
        super().__init__('vention_node')

        # ------------------------------------------------------
        # 1. Declare and Retrieve Parameters
        # ------------------------------------------------------
        self.declare_parameter('vention_ip', '192.168.7.2')
        self.declare_parameter('is_onedrive', False)
        self.declare_parameter('publish_position', True)
        self.declare_parameter('timer_period_s', 0.2)
        self.declare_parameter('use_drive_2', True)
        self.declare_parameter('use_drive_3', True)

        vention_ip = self.get_parameter('vention_ip').value
        is_onedrive = self.get_parameter('is_onedrive').value
        self.publish_position = self.get_parameter('publish_position').value
        self.timer_period_s = float(self.get_parameter('timer_period_s').value)
        self.use_drive_2 = bool(self.get_parameter('use_drive_2').value)
        self.use_drive_3 = bool(self.get_parameter('use_drive_3').value)

        # ------------------------------------------------------
        # 2. Initialize VentionController
        # ------------------------------------------------------
        self.vention = VentionController(
            ip_address=vention_ip,
            is_one_drive=is_onedrive
        )
        self.get_logger().info(f"Initialized VentionController at IP={vention_ip}, is_onedrive={is_onedrive}")

        # ------------------------------------------------------
        # 3. Internal State for Continuous Velocity
        # ------------------------------------------------------
        self.vel_drive_2 = 0.0
        self.vel_drive_3 = 0.0

        # ------------------------------------------------------
        # 4. Publishers
        #    - Carriage or drive positions
        # ------------------------------------------------------
        self.pub_drive_2_pos = self.create_publisher(Float32, 'drive_2_position', 10)
        self.pub_drive_3_pos = self.create_publisher(Float32, 'drive_3_position', 10)

        # ------------------------------------------------------
        # 5. Subscribers
        # ------------------------------------------------------

        # 5a) Velocity commands (mm/s)
        self.sub_drive_2_vel_cmd = self.create_subscription(
            Float32,
            'drive_2_vel_cmd',
            self.cb_drive_2_vel_cmd,
            10
        )
        self.sub_drive_3_vel_cmd = self.create_subscription(
            Float32,
            'drive_3_vel_cmd',
            self.cb_drive_3_vel_cmd,
            10
        )

        # 5b) Position commands (mm)
        self.sub_drive_2_pos_cmd = self.create_subscription(
            Float32,
            'drive_2_pos_cmd',
            self.cb_drive_2_pos_cmd,
            10
        )
        self.sub_drive_3_pos_cmd = self.create_subscription(
            Float32,
            'drive_3_pos_cmd',
            self.cb_drive_3_pos_cmd,
            10
        )

        # 5c) E-stop commands (TRIGGER, RELEASE, RESET)
        self.sub_estop_cmd = self.create_subscription(
            String,
            'estop_cmd',
            self.cb_estop_cmd,
            10
        )

        # 5d) Brake commands (LOCK_2, UNLOCK_2, etc.)
        self.sub_brake_cmd = self.create_subscription(
            String,
            'brake_cmd',
            self.cb_brake_cmd,
            10
        )

        # 5e) Power switch commands (SWITCH_ON_1, SWITCH_OFF_1, etc.)
        self.sub_power_switch_cmd = self.create_subscription(
            String,
            'power_switch_cmd',
            self.cb_power_switch_cmd,
            10
        )

        # 5f) Homing commands (HOME_2, HOME_3, HOME_ALL)
        self.sub_homing_cmd = self.create_subscription(
            String,
            'homing_cmd',
            self.cb_homing_cmd,
            10
        )

        # 5g) Speed/Acceleration config commands (for global moves)
        #     Example usage: "SET_SPEED 200.0" => sets global speed to 200 mm/s
        self.sub_speed_cmd = self.create_subscription(
            String,
            'speed_cmd',
            self.cb_speed_cmd,
            10
        )
        # same approach for acceleration
        self.sub_accel_cmd = self.create_subscription(
            String,
            'accel_cmd',
            self.cb_accel_cmd,
            10
        )

        # 5h) Tare position commands (like "SET_POS_2 100.0" => sets axis #2's position to 100 mm)
        self.sub_set_position_cmd = self.create_subscription(
            String,
            'set_position_cmd',
            self.cb_set_position_cmd,
            10
        )

        # 5i) Digital IO write commands
        #     Expect a message like "2,0,1" => means device=2, pin=0, value=1
        self.sub_digital_write_cmd = self.create_subscription(
            String,
            'digital_write_cmd',
            self.cb_digital_write_cmd,
            10
        )

        # ------------------------------------------------------
        # 6. Timer: Re-apply velocities & publish positions
        # ------------------------------------------------------
        self.timer = self.create_timer(self.timer_period_s, self.timer_callback)

        self.get_logger().info("vention_node started. Listening for commands...")

    # ------------------------------------------------------
    # Velocity Callbacks
    # ------------------------------------------------------
    def cb_drive_2_vel_cmd(self, msg: Float32):
        if not self.use_drive_2:
            return
        self.vel_drive_2 = msg.data
        self.get_logger().debug(f"[Drive2] velocity => {self.vel_drive_2} mm/s")

    def cb_drive_3_vel_cmd(self, msg: Float32):
        if not self.use_drive_3:
            return
        self.vel_drive_3 = msg.data
        self.get_logger().debug(f"[Drive3] velocity => {self.vel_drive_3} mm/s")

    # ------------------------------------------------------
    # Position Callbacks
    # ------------------------------------------------------
    def cb_drive_2_pos_cmd(self, msg: Float32):
        if not self.use_drive_2:
            return
        position = msg.data
        self.get_logger().info(f"[Drive2] position cmd => {position} mm")
        try:
            self.vention.move_to_position(axis=2, position=position)
        except Exception as e:
            self.get_logger().error(f"[Drive2] position move error: {e}")

    def cb_drive_3_pos_cmd(self, msg: Float32):
        if not self.use_drive_3:
            return
        position = msg.data
        self.get_logger().info(f"[Drive3] position cmd => {position} mm")
        try:
            self.vention.move_to_position(axis=3, position=position)
        except Exception as e:
            self.get_logger().error(f"[Drive3] position move error: {e}")

    # ------------------------------------------------------
    # E-Stop Callback
    # ------------------------------------------------------
    def cb_estop_cmd(self, msg: String):
        """
        Possible messages:
          - "TRIGGER"
          - "RELEASE"
          - "RESET"
        """
        cmd = msg.data.upper().strip()
        self.get_logger().info(f"[E-Stop CMD] => {cmd}")
        if cmd == "TRIGGER":
            try:
                success = self.vention.trigger_estop()
                self.get_logger().warn(f"E-stop triggered => {success}")
            except Exception as e:
                self.get_logger().error(f"Error triggering e-stop: {e}")
        elif cmd == "RELEASE":
            try:
                success = self.vention.release_estop()
                self.get_logger().warn(f"E-stop released => {success}")
            except Exception as e:
                self.get_logger().error(f"Error releasing e-stop: {e}")
        elif cmd == "RESET":
            try:
                success = self.vention.reset_system()
                self.get_logger().warn(f"System reset => {success}")
            except Exception as e:
                self.get_logger().error(f"Error resetting system: {e}")
        else:
            self.get_logger().warn(f"Unknown e-stop command: {cmd}")

    # ------------------------------------------------------
    # Brake Callback
    # ------------------------------------------------------
    def cb_brake_cmd(self, msg: String):
        """
        Example messages:
          - "LOCK_2" => lock brake on AUX port #2
          - "UNLOCK_2" => unlock brake on AUX port #2
        """
        cmd = msg.data.upper().strip()
        self.get_logger().info(f"[Brake CMD] => {cmd}")
        if cmd.startswith("LOCK_"):
            try:
                aux_str = cmd.split("_")[1]
                aux_port = int(aux_str)
                self.vention.lock_brake(aux_port)
                self.get_logger().info(f"Locked brake on AUX port {aux_port}")
            except Exception as e:
                self.get_logger().error(f"Error locking brake: {e}")
        elif cmd.startswith("UNLOCK_"):
            try:
                aux_str = cmd.split("_")[1]
                aux_port = int(aux_str)
                self.vention.unlock_brake(aux_port)
                self.get_logger().info(f"Unlocked brake on AUX port {aux_port}")
            except Exception as e:
                self.get_logger().error(f"Error unlocking brake: {e}")
        else:
            self.get_logger().warn(f"Unknown brake command: {cmd}")

    # ------------------------------------------------------
    # Power Switch Callback
    # ------------------------------------------------------
    def cb_power_switch_cmd(self, msg: String):
        """
        Example usage:
          - "SWITCH_ON_1"  => set power switch module #1 => ON
          - "SWITCH_OFF_1" => set power switch module #1 => OFF
        """
        cmd = msg.data.upper().strip()
        self.get_logger().info(f"[Power Switch CMD] => {cmd}")
        if cmd.startswith("SWITCH_ON_"):
            try:
                dev_id_str = cmd.split("_")[2]
                dev_id = int(dev_id_str)
                self.vention.set_power_switch(dev_id, POWER_SWITCH.ON)
                self.get_logger().info(f"Power switch {dev_id} => ON")
            except Exception as e:
                self.get_logger().error(f"Error switching on device: {e}")
        elif cmd.startswith("SWITCH_OFF_"):
            try:
                dev_id_str = cmd.split("_")[2]
                dev_id = int(dev_id_str)
                self.vention.set_power_switch(dev_id, POWER_SWITCH.OFF)
                self.get_logger().info(f"Power switch {dev_id} => OFF")
            except Exception as e:
                self.get_logger().error(f"Error switching off device: {e}")
        else:
            self.get_logger().warn(f"Unknown power switch command: {cmd}")

    # ------------------------------------------------------
    # Homing Callback
    # ------------------------------------------------------
    def cb_homing_cmd(self, msg: String):
        """
        Example usage:
          - "HOME_2" => home drive #2
          - "HOME_3" => home drive #3
          - "HOME_ALL" => home all
        """
        cmd = msg.data.upper().strip()
        self.get_logger().info(f"[Homing CMD] => {cmd}")
        if cmd == "HOME_ALL":
            try:
                self.vention.home_all_axes()
                self.vention.wait_for_motion_completion()
                self.get_logger().info("All axes homed.")
            except Exception as e:
                self.get_logger().error(f"Error homing all: {e}")
        elif cmd.startswith("HOME_"):
            axis_str = cmd.split("_")[1]
            try:
                axis = int(axis_str)
                self.vention.home_axis(axis)
                self.vention.wait_for_motion_completion()
                self.get_logger().info(f"Axis {axis} homed.")
            except Exception as e:
                self.get_logger().error(f"Error homing axis {axis_str}: {e}")
        else:
            self.get_logger().warn(f"Unknown homing command: {cmd}")

    # ------------------------------------------------------
    # Speed & Acceleration Callbacks
    # ------------------------------------------------------
    def cb_speed_cmd(self, msg: String):
        """
        Example usage: "SET_SPEED 200.0"
        => sets global move speed to 200 mm/s
        """
        data = msg.data.upper().strip()
        if not data.startswith("SET_SPEED"):
            self.get_logger().warn(f"Unknown speed command: {data}")
            return
        try:
            parts = data.split()
            if len(parts) < 2:
                self.get_logger().warn(f"Invalid speed command: {data}")
                return
            speed_val = float(parts[1])
            self.vention.set_speed(speed_val)
            self.get_logger().info(f"Global speed set to {speed_val} mm/s")
        except Exception as e:
            self.get_logger().error(f"Error setting speed: {e}")

    def cb_accel_cmd(self, msg: String):
        """
        Example usage: "SET_ACCEL 300.0"
        => sets global move accel to 300 mm/s^2
        """
        data = msg.data.upper().strip()
        if not data.startswith("SET_ACCEL"):
            self.get_logger().warn(f"Unknown accel command: {data}")
            return
        try:
            parts = data.split()
            if len(parts) < 2:
                self.get_logger().warn(f"Invalid accel command: {data}")
                return
            accel_val = float(parts[1])
            self.vention.set_acceleration(accel_val)
            self.get_logger().info(f"Global acceleration set to {accel_val} mm/s^2")
        except Exception as e:
            self.get_logger().error(f"Error setting acceleration: {e}")

    # ------------------------------------------------------
    # Tare/Set Position Callback
    # ------------------------------------------------------
    def cb_set_position_cmd(self, msg: String):
        """
        Example usage: "SET_POS_2 100.0" => sets axis #2's position to 100 mm
        or "SET_POS_3 0.0" => sets axis #3's position to 0 mm
        """
        data = msg.data.upper().strip()
        if not data.startswith("SET_POS_"):
            self.get_logger().warn(f"Unknown set_position command: {data}")
            return
        try:
            parts = data.split()
            # e.g.: "SET_POS_2", "100.0"
            if len(parts) < 2:
                self.get_logger().warn(f"Invalid set_position command: {data}")
                return

            axis_part = parts[0]
            pos_val = float(parts[1])
            # axis_part might be "SET_POS_2"
            axis_str = axis_part.split("_")[2]  # "2"
            axis = int(axis_str)

            self.vention.set_position(axis, pos_val)
            self.get_logger().info(f"Axis {axis} position set to {pos_val} mm")
        except Exception as e:
            self.get_logger().error(f"Error setting position: {e}")

    # ------------------------------------------------------
    # Digital IO Write Callback
    # ------------------------------------------------------
    def cb_digital_write_cmd(self, msg: String):
        """
        Expects a message in the form "device,pin,value" 
        For example: "2,0,1" => digitalWrite(device=2, pin=0, value=1)
        """
        data = msg.data.strip()
        try:
            parts = data.split(",")
            if len(parts) != 3:
                self.get_logger().warn(f"Invalid digital_write_cmd: {data}")
                return
            dev_id = int(parts[0])
            pin = int(parts[1])
            val = int(parts[2])
            self.vention.digital_write(dev_id, pin, val)
            self.get_logger().info(f"digitalWrite({dev_id}, {pin}, {val}) => done.")
        except Exception as e:
            self.get_logger().error(f"Error in digital_write_cmd: {e}")

    # ------------------------------------------------------
    # Timer Callback
    # ------------------------------------------------------
    def timer_callback(self):
        """
        Periodically re-apply velocity commands for drives #2 & #3
        and publish positions (if enabled).
        """
        # 1. Drive #2 velocity
        if self.use_drive_2:
            try:
                if abs(self.vel_drive_2) > 1e-6:
                    self.vention.move_continuous(axis=2, speed_mm_s=self.vel_drive_2, accel_mm_s2=100.0)
                else:
                    self.vention.stop_move_continuous(axis=2, accel_mm_s2=100.0)
            except Exception as e:
                self.get_logger().error(f"[Drive2] velocity update error: {e}")

        # 2. Drive #3 velocity
        if self.use_drive_3:
            try:
                if abs(self.vel_drive_3) > 1e-6:
                    self.vention.move_continuous(axis=3, speed_mm_s=self.vel_drive_3, accel_mm_s2=100.0)
                else:
                    self.vention.stop_move_continuous(axis=3, accel_mm_s2=100.0)
            except Exception as e:
                self.get_logger().error(f"[Drive3] velocity update error: {e}")

        # 3. Publish positions (if enabled)
        if self.publish_position:
            # Drive #2 pos
            if self.use_drive_2:
                try:
                    pos_2 = self.vention.get_position(2)
                    msg_2 = Float32()
                    msg_2.data = float(pos_2)
                    self.pub_drive_2_pos.publish(msg_2)
                except Exception as e:
                    self.get_logger().error(f"[Drive2] get_position error: {e}")
            # Drive #3 pos
            if self.use_drive_3:
                try:
                    pos_3 = self.vention.get_position(3)
                    msg_3 = Float32()
                    msg_3.data = float(pos_3)
                    self.pub_drive_3_pos.publish(msg_3)
                except Exception as e:
                    self.get_logger().error(f"[Drive3] get_position error: {e}")

    # ------------------------------------------------------
    # Node Shutdown
    # ------------------------------------------------------
    def destroy_node(self):
        """
        On node shutdown, do final cleanup: stop all motion.
        """
        self.get_logger().info("Node shutting down -> stopping all motion.")
        try:
            self.vention.stop_all_motion()
        except Exception as e:
            self.get_logger().warn(f"Error stopping motion: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VentionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt -> shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
