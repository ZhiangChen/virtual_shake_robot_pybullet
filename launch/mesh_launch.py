#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os 
import yaml

def generate_launch_description():

    # Retrieve the ROS2 workspace path from the environment or use a default
    ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))

    # Define the configuration directory using the workspace path
    config_directory = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/config')

    # Define paths to the configuration files with placeholders
    physics_engine_parameters_path = os.path.join(config_directory, 'physics_engine_parameters.yaml')
    physics_parameters_path = os.path.join(config_directory, 'physics_parameters.yaml')
    vsr_structure_path = os.path.join(config_directory, 'vsr_structure.yaml')
    pbr_structure_path = os.path.join(config_directory, 'pbr_physics.yaml')

    # Function to replace placeholders in the YAML files
    def replace_placeholders(file_path, placeholder, value):
        with open(file_path, 'r') as file:
            content = yaml.safe_load(file)
        content_str = yaml.dump(content)
        content_str = content_str.replace(placeholder, value)
        updated_content = yaml.safe_load(content_str)
        return updated_content

    # Replace placeholders in the paths
    physics_engine_parameters_content = replace_placeholders(physics_engine_parameters_path, '{{ROS2_WS}}', ros2_ws)
    physics_parameters_content = replace_placeholders(physics_parameters_path, '{{ROS2_WS}}', ros2_ws)
    vsr_structure_content = replace_placeholders(vsr_structure_path, '{{ROS2_WS}}', ros2_ws)
    pbr_structure_content = replace_placeholders(pbr_structure_path, '{{ROS2_WS}}', ros2_ws)

    # Create a temporary directory for modified YAML files
    temp_dir = os.path.join(os.path.dirname(__file__), 'temp')
    os.makedirs(temp_dir, exist_ok=True)

    # Function to write the modified YAML content to temporary files
    def write_temp_yaml(content, filename):
        temp_path = os.path.join(temp_dir, filename)
        with open(temp_path, 'w') as file:
            yaml.safe_dump(content, file)
        return temp_path

    # Write the modified YAML contents to temporary files
    physics_engine_parameters_temp_path = write_temp_yaml(physics_engine_parameters_content, 'physics_engine_parameters.yaml')
    physics_parameters_temp_path = write_temp_yaml(physics_parameters_content, 'physics_parameters.yaml')
    vsr_structure_temp_path = write_temp_yaml(vsr_structure_content, 'vsr_structure.yaml')
    pbr_structure_temp_path = write_temp_yaml(pbr_structure_content, 'pbr_physics.yaml')

    # Define the simulation node with updated parameter paths
    simulation_node = Node(
        package='virtual_shake_robot_pybullet',  
        executable='simulation_node.py',  
        name='simulation_node', 
        output='screen',
        parameters=[
            physics_engine_parameters_temp_path,
            physics_parameters_temp_path,
            vsr_structure_temp_path,
            pbr_structure_temp_path
        ]
    )

    return LaunchDescription([
        simulation_node
    ])
