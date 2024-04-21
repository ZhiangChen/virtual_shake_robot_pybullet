#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   
    simulation_node = Node(
        package='virtual_shake_robot_pybullet',  
        executable='simulation_node.py',  
        name='simulation_node', 
        output='screen'  
    )

    

    return LaunchDescription([
        simulation_node
    ])
