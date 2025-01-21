#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    """
    Launch file that:
      1) Starts an ephemeral 'vention_reset' node to release e-stop & reset system
      2) Once 'vention_reset' node exits, it starts the main 'vention_node'
    If the system is already fine, the reset node quickly does nothing & stops.
    Then 'vention_node' runs as usual.
    """

    # 1) Node that attempts to release e-stop and reset system
    reset_node = Node(
        package='unm_ros2_vention',
        executable='vention_reset.py',
        name='vention_reset_node',
        output='screen',
        # Optionally pass parameters to override default IP if desired
        parameters=[{'vention_ip': '192.168.7.2'}]
    )

    # 2) Main Vention node. We'll start it AFTER reset node finishes
    vention_node = Node(
        package='unm_ros2_vention',
        executable='vention_node.py',
        name='vention_node',
        output='screen',
        parameters=[{
            # put your usual defaults or param file here:
            'vention_ip': '192.168.7.2',
            'timer_period_s': 0.2,
            'use_drive_1': True,
            'use_drive_2': False,
            'use_drive_3': True,
            'use_drive_4': False
        }]
    )

    # We only launch 'vention_node' after 'vention_reset' completes, 
    # to ensure the system is cleared first.
    event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_node,
            on_exit=[vention_node],
        )
    )

    return LaunchDescription([
        reset_node,
        event_handler,
    ])
