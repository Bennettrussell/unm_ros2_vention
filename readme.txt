
This ROS 2 package contains everything you need to control a **Vention MachineMotion v2** (or v2OneDrive) system.
Right now it is only configured for use with Drive_1 and Drive_3; however this is easily modified.
The package includes:

1. vention_command.py which is a simple control loop that does not need to be built with the ROS2 node
2. A **high-level Python wrapper** (`vention_api_wrapper.py`) providing the `VentionController` class.
3. A **ROS 2 node** (`vention_node.py`) that exposes standard topics for velocity, position, e-stop, brakes, etc.
4. An **optional ephemeral reset node** (`vention_reset.py`) to ensure e-stop is cleared on startup.

---

## Package Layout

unm_ros2_vention/ 
├── package.xml 
├── CMakeLists.txt 
├── unm_ros2_vention/
│       	├── MachineMotion.py # find your version of this from VentionCo, based on your firmware revision 
|      		├── vention_api_wrapper.py # Wrapper providing VentionController 
|      		├── vention_node.py # Main ROS 2 node 
|      		└── vention_reset.py # Optional ephemeral node to reset e-stop 
└── launch/ 
│         └── unified_launch.launch.py 
├── vention_command.py
└── README.md

## Installation & Build

1. **Clone** into your ROS 2 workspace:
   ```bash
   cd ~/ros2_jazzy/src
   git clone <this-repo> unm_ros2_vention

1. Verify:
    ros2 pkg list | grep unm_ros2_vention

## Key Components 
1
. MachineMotion.py

    A library from Vention that connects via MQTT/HTTP to MachineMotion v2 hardware.
    Contains the lower-level classes and logic (e.g. GCode, enumerations, etc.). must be sourced from VentionCo's github

2. vention_api_wrapper.py (The VentionController)

    A high-level wrapper that internally creates a MachineMotionV2 or MachineMotionV2OneDrive instance.

3. vention_node.py (Main ROS 2 Node)

    A ROS 2 node wrapping VentionController functionality.
    Subscribes to topics to set velocity, position, speed, e-stop, brakes, etc.
    Publishes axis positions (e.g. drive_2_position).
    Default IP param is 192.168.7.2.

4. vention_reset.py (Optional E-stop Reset Node)

    A short-lived node that calls release_estop() and reset_system() on launch, then exits.
    Use it if you suspect the machine might be stuck in e-stop from a previous run.
    
5. vention_command.py, makes a sample control loop that is easily modified for system control


## Launch methods
direct:
    ros2 run unm_ros2_vention vention_node
Via launch file:
    ros2 launch unm_ros2_vention vention_node.launch

## Subscribers
Positions:

    /drive_3_position (std_msgs/Float32): drive #3’s position in mm.

Velocity:

    /drive_3_vel_cmd (std_msgs/Float32): mm/s velocity for drive #3.

Position:

    /drive_3_pos_cmd (std_msgs/Float32): absolute pos in mm for drive #3.

E-stop:

    /estop_cmd (std_msgs/String): commands "TRIGGER", "RELEASE", "RESET".

Brakes:

    /brake_cmd (std_msgs/String): e.g. "LOCK_2", "UNLOCK_2".

Power Switch:

    /power_switch_cmd (std_msgs/String): e.g. "SWITCH_ON_1", "SWITCH_OFF_1".

Homing:

    /homing_cmd (std_msgs/String): e.g. "HOME_2", "HOME_ALL".

Speed/Accel:

    /speed_cmd (std_msgs/String): e.g. "SET_SPEED 250.0".
    /accel_cmd (std_msgs/String): e.g. "SET_ACCEL 300.0".

Set Position:

    /set_position_cmd (std_msgs/String): e.g. "SET_POS_2 100.0".

Digital IO:

    /digital_write_cmd (std_msgs/String): e.g. "2,0,1" => digitalWrite(2, 0, 1).

## Sample Commands:

Velocity:   
    ros2 topic pub /drive_2_vel_cmd std_msgs/Float32 "data: 50.0"

Absolute Move:
    ros2 topic pub /drive_2_pos_cmd std_msgs/Float32 "data: 200.0"

Homing:
    ros2 topic pub /homing_cmd std_msgs/String "data: 'HOME_ALL'"

E-stop:
    ros2 topic pub /estop_cmd std_msgs/String "data: 'TRIGGER'"
    ros2 topic pub /estop_cmd std_msgs/String "data: 'RELEASE'"
    ros2 topic pub /estop_cmd std_msgs/String "data: 'RESET'"

Brakes:
    ros2 topic pub /brake_cmd std_msgs/String "data: 'LOCK_2'"
    ros2 topic pub /brake_cmd std_msgs/String "data: 'UNLOCK_2'"

Digital write:
    ros2 topic pub /digital_write_cmd std_msgs/String "data: '2,0,1'"


