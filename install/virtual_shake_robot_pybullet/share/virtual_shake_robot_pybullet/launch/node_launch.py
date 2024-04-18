#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    simulation_node = Node(
        package='virtual_shake_robot_pybullet',  # Replace this with your package name
        executable='simulation_node',  # This should match the name of your node executable
        name='simulation_node',  # The ROS2 node name for this instance
        output='screen'  # Optionally, direct the output to the screen
    )

    return LaunchDescription([
        simulation_node
    ])
