#!/usr/bin/env python3
# vention_api_wrapper.py
#
# A high-level wrapper around the MachineMotion v2 or v2OneDrive code (from MachineMotion.py).
# This excludes all MMv1 features and ensures smooth usage for a dual-gantry or other v2 setups.

import sys
import time
import json
import threading

from MachineMotion import *

class VentionController:
    """
    A wrapper for controlling a MachineMotion v2 or v2OneDrive system.

    Features:
      - Stepper/Servo config (microsteps, direction, current, closed-loop)
      - Motion commands (position, velocity, homing, stopping)
      - E-stop and system reset
      - Brake lock/unlock
      - Digital IO (read/write, push buttons, power switch)
      - Checking if smart drives are ready
    """

    def __init__(self,
                 ip_address: str = DEFAULT_IP_ADDRESS.ethernet,
                 is_one_drive: bool = False,
                 user_gcode_callback=None):
        """
        Initialize with the provided IP address. Defaults to Ethernet IP if not specified.
        :param ip_address: e.g. "192.168.0.2" (Ethernet) or "192.168.7.2" (USB).
        :param is_one_drive: True if using a MachineMotion v2OneDrive.
        :param user_gcode_callback: Optionally, a function to handle g-code replies.
        """
        self.ip_address = ip_address

        if is_one_drive:
            self.hw_version = MACHINEMOTION_HW_VERSIONS.MMv2OneDrive
            self.machine = MachineMotionV2OneDrive(machineIp=ip_address)
        else:
            self.hw_version = MACHINEMOTION_HW_VERSIONS.MMv2
            self.machine = MachineMotionV2(machineIp=ip_address)

        # If user wants a custom callback for raw g-code:
        if user_gcode_callback is not None:
            # In MachineMotion v2, the internal GCode instance is at self.machine.myGCode
            self.machine.myGCode.__setUserCallback__(user_gcode_callback)

        print(f"[VentionController] Initialized with IP={ip_address} (hw_version={self.hw_version})")

    # --------------------------------------------------------------------------
    # Configuration Methods
    # --------------------------------------------------------------------------

    def config_stepper(self,
                       drive: int,
                       mech_gain: float,
                       direction: str,
                       motor_current: float,
                       microsteps: int = MICRO_STEPS.ustep_8):
        """
        Configure a single drive as a stepper (open-loop).
        Calls machine.configStepper(...) from MachineMotion.
        """
        self.machine.configStepper(
            drive=drive,
            mechGain=mech_gain,
            direction=direction,
            motorCurrent=motor_current,
            microSteps=microsteps
        )

    def config_servo(self,
                     drives,
                     mech_gain: float,
                     directions,
                     motor_current: float,
                     tuning_profile=TUNING_PROFILES.DEFAULT,
                     parent_drive=None):
        """
        Configure one or multiple drives as servo (closed-loop).
        Calls machine.configServo(...).
        """
        self.machine.configServo(
            drives=drives,
            mechGain=mech_gain,
            directions=directions,
            motorCurrent=motor_current,
            tuningProfile=tuning_profile,
            parentDrive=parent_drive
        )

    def config_homing_speed(self, axes, speeds, units=UNITS_SPEED.mm_per_sec):
        """
        Sets homing speed for each axis.
        Calls machine.configHomingSpeed(...).
        """
        self.machine.configHomingSpeed(axes, speeds, units)

    # --------------------------------------------------------------------------
    # Motion Methods
    # --------------------------------------------------------------------------

    def move_to_position(self, axis: int, position: float):
        """Moves one axis to an absolute position (in mm)."""
        self.machine.moveToPosition(axis, position)

    def move_to_position_combined(self, axes, positions):
        """Moves multiple axes simultaneously to absolute positions."""
        self.machine.moveToPositionCombined(axes, positions)

    def move_relative(self, axis: int, distance: float):
        """Moves one axis by a relative distance (in mm)."""
        self.machine.moveRelative(axis, distance)

    def move_relative_combined(self, axes, distances):
        """Moves multiple axes by relative distances in mm, synchronously."""
        self.machine.moveRelativeCombined(axes, distances)

    def move_continuous(self, axis: int, speed_mm_s: float, accel_mm_s2: float):
        """
        Commands continuous velocity (speed_mm_s in mm/s) with a given acceleration in mm/s^2.
        """
        self.machine.moveContinuous(axis, speed_mm_s, accel_mm_s2)

    def stop_move_continuous(self, axis: int, accel_mm_s2: float):
        """
        Stops a continuous move by setting the speed to 0 with specified acceleration.
        """
        self.machine.stopMoveContinuous(axis, accel_mm_s2)

    def stop_all_motion(self):
        """Immediate stop of all axes (hard stop)."""
        self.machine.stopAllMotion()

    def home_all_axes(self):
        """Home all axes sequentially (G28)."""
        self.machine.moveToHomeAll()

    def home_axis(self, axis: int):
        """Home a single axis."""
        self.machine.moveToHome(axis)

    def set_speed(self, speed: float, units=UNITS_SPEED.mm_per_sec):
        """Sets the global feedrate speed for moves (mm/s or mm/min)."""
        self.machine.setSpeed(speed, units)

    def set_acceleration(self, accel: float, units=UNITS_ACCEL.mm_per_sec_sqr):
        """Sets the global acceleration for moves."""
        self.machine.setAcceleration(accel, units)

    def wait_for_motion_completion(self):
        """
        Blocks until the last non-continuous move has finished.
        (Does not handle ongoing continuous moves.)
        """
        self.machine.waitForMotionCompletion()

    def is_motion_completed(self) -> bool:
        """
        Returns True if the last move command is completed.
        """
        return self.machine.isMotionCompleted()

    def set_position(self, axis: int, position: float):
        """
        Overrides the machine's known position for an axis (similar to M92).
        """
        self.machine.setPosition(axis, position)

    # --------------------------------------------------------------------------
    # Query / State
    # --------------------------------------------------------------------------

    def get_position(self, axis: int) -> float:
        """Returns the actual position of the specified axis."""
        return self.machine.getActualPositions(axis=axis)

    def get_all_positions(self):
        """Returns a dict of all axis positions."""
        return self.machine.getActualPositions()

    def get_endstop_state(self):
        """
        Returns a dict of endstop states. For v2OneDrive, only x_min/x_max exist.
        e.g.:
          { 'x_min': 'open', 'x_max': 'TRIGGERED', 'y_min':..., etc. }
        """
        return self.machine.getEndStopState()

    # --------------------------------------------------------------------------
    # E-Stop
    # --------------------------------------------------------------------------

    def trigger_estop(self) -> bool:
        """
        Triggers the software E-stop, cutting power to drives. Return True/False success.
        """
        return self.machine.triggerEstop()

    def release_estop(self) -> bool:
        """
        Releases software E-stop, re-energizing drives. Return True/False success.
        """
        return self.machine.releaseEstop()

    def reset_system(self) -> bool:
        """
        Resets after an E-stop. Waits for drives to come online if v2.
        Returns True/False success.
        """
        return self.machine.resetSystem()

    def bind_estop_event(self, callback_func):
        """
        Registers a callback triggered on eStop or release events.
        """
        self.machine.bindeStopEvent(callback_func)

    # --------------------------------------------------------------------------
    # Brakes & SmartDrive Readiness
    # --------------------------------------------------------------------------

    def lock_brake(self, aux_port: int):
        """
        Lock the brake by setting it to 0V. 
        For v2, you must pass safety_adapter_presence=True, 
        so we do that automatically.
        """
        self.machine.lockBrake(aux_port, safety_adapter_presence=True)

    def unlock_brake(self, aux_port: int):
        """
        Unlock the brake by applying 24V.
        """
        self.machine.unlockBrake(aux_port, safety_adapter_presence=True)

    def get_brake_state(self, aux_port: int) -> str:
        """
        Returns the brake state: 'locked' / 'unlocked' / 'unknown' 
        for the given aux_port.
        """
        return self.machine.getBrakeState(aux_port, safety_adapter_presence=True)

    @property
    def are_smart_drives_ready(self) -> bool:
        """
        Check if the MachineMotion v2 or v2OneDrive drives are energized/ready.
        Corresponds to 'self.machine.areSmartDrivesReady'.
        """
        return bool(self.machine.areSmartDrivesReady)

    # --------------------------------------------------------------------------
    # Digital IO, Push Buttons, Power Switch
    # --------------------------------------------------------------------------

    def detect_io_modules(self):
        """Returns a dict of discovered IO modules."""
        return self.machine.detectIOModules()

    def digital_read(self, device_id: int, pin: int) -> int:
        """Returns 1 if input is High(24V), 0 if Low(0V)."""
        return self.machine.digitalRead(device_id, pin)

    def digital_write(self, device_id: int, pin: int, value: int):
        """Sets an IO expander output pin to 1(24V) or 0(0V)."""
        self.machine.digitalWrite(device_id, pin, value)

    def set_power_switch(self, device_id: int, switch_state):
        """
        For a power-switch module: POWER_SWITCH.ON or .OFF 
        (or True/False).
        """
        self.machine.setPowerSwitch(device_id, switch_state)

    def wait_on_push_button(self,
                            device_id: int,
                            button_color: int,
                            desired_state=PUSH_BUTTON.STATE.PUSHED,
                            timeout=None) -> bool:
        """
        Blocks until the push button reaches a desired state or times out.
        """
        return self.machine.waitOnPushButton(device_id, button_color, desired_state, timeout)

    def bind_push_button_event(self, device_id: int, button_color: int, callback_function):
        """
        Registers a callback whenever the push button changes state.
        """
        self.machine.bindPushButtonEvent(device_id, button_color, callback_function)

    def read_push_button(self, device_id: int, button_color: int) -> str:
        """
        Returns 'pushed' or 'released' for the push button.
        """
        return self.machine.readPushButton(device_id, button_color)

    # --------------------------------------------------------------------------
    # Shutdown Helpers
    # --------------------------------------------------------------------------

    def stop_all(self):
        """Alias for stop_all_motion."""
        self.stop_all_motion()

    def shutdown(self):
        """
        Gracefully shuts down the system (stops all motion).
        Optionally, you could stop the MQTT loop here as well.
        """
        try:
            self.stop_all_motion()
        except:
            pass
        print("[VentionController] Shutdown complete. All motion stopped.")


# --------------------------------------------------------------------------
# Optional Quick Test
# --------------------------------------------------------------------------
if __name__ == "__main__":
    # Example usage/test if run directly
    vc = VentionController(ip_address="192.168.7.2", is_one_drive=False)
    try:
        # Config stepper on drive #2:
        vc.config_stepper(
            drive=3,
            mech_gain=MECH_GAIN.timing_belt_150mm_turn,
            direction=DIRECTION.NORMAL,
            motor_current=3.0,
            microsteps=MICRO_STEPS.ustep_8
        )

        # Move absolute
        vc.move_to_position(axis=3, position=200.0)
        vc.wait_for_motion_completion()

        # Check positions
        pos_2 = vc.get_position(2)
        print(f"[TEST] drive #2 is at {pos_2} mm")

        # Stop & shutdown
        vc.shutdown()

    except Exception as e:
        print(f"[ERROR] {e}")
        vc.shutdown()
