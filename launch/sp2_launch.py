#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare the motion_mode argument
    motion_mode_arg = DeclareLaunchArgument(
        'motion_mode',
        default_value='',
        description='Motion mode for the control node: grid_cosine, single_recording, all_recordings'
    )
    
    # Set up the ROS2 workspace path
    ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))

    # Define the configuration directory and YAML paths
    config_directory = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/config')

    physics_engine_parameters_path = os.path.join(config_directory, 'physics_engine_parameters.yaml')
    physics_parameters_path = os.path.join(config_directory, 'physics_parameters.yaml')
    vsr_structure_path = os.path.join(config_directory, 'vsr_structure_box.yaml')
    pbr_structure_path = os.path.join(config_directory, 'pbr_box.yaml')
    sp2_mesh_structure_path = os.path.join(config_directory, 'sp2.yaml')

    # Define URDF file path
    urdf_file_path = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/models/SP2_PBRmodel/sp2.urdf')

    # Define the simulation node with direct YAML file loading (no temp files)
    simulation_node = Node(
        package='virtual_shake_robot_pybullet',
        executable='simulation_node.py',
        name='simulation_node',
        output='screen',
        parameters=[
            physics_engine_parameters_path,
            physics_parameters_path,
            vsr_structure_path,
            pbr_structure_path,
            sp2_mesh_structure_path,
            {'urdf_file': urdf_file_path}  # Using the direct URDF file path
        ]
    )

    control_node = Node(
        package='virtual_shake_robot_pybullet',
        executable='control_node.py',
        name='control_node',
        output='screen',
        parameters=[
            {'motion_mode': LaunchConfiguration('motion_mode')}
        ]
    )

    return LaunchDescription([
        motion_mode_arg,
        simulation_node,
        control_node    
    ])
