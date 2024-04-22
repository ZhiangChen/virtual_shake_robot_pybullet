#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
import os 
def generate_launch_description():

    config_directory = '/home/akshay/ros2_ws/virtual_shake_robot_pybullet/config'

    physics_engine_parameters_path = os.path.join(config_directory, 'physics_engine_parameters.yaml')
    physics_parameters_path = os.path.join(config_directory, 'pyhsics_parameters.yaml')
    vsr_structure_path = os.path.join(config_directory, 'vsr_structure.yaml')

    simulation_node = Node(
        package='virtual_shake_robot_pybullet',  
        executable='simulation_node.py',  
        name='simulation_node', 
        output='screen' ,
        parameters= [physics_engine_parameters_path,physics_parameters_path,vsr_structure_path]
    )

    

    return LaunchDescription([
        simulation_node
    ])
