#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():
    
    ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))

    config_directory = os.path.join(ros2_ws, 'virtual_shake_robot_pybullet/config')

    physics_engine_parameters_path = os.path.join(config_directory, 'physics_engine_parameters.yaml')
    physics_parameters_path = os.path.join(config_directory, 'physics_parameters.yaml')
    vsr_structure_path = os.path.join(config_directory, 'vsr_structure_box.yaml')
    pbr_structure_path = os.path.join(config_directory, 'pbr_box.yaml')
    sp2_mesh_structure_path = os.path.join(config_directory, 'sp2.yaml')
    
    # Define URDF file path with a placeholder
    urdf_file_path = os.path.join('{{ROS2_WS}}', 'virtual_shake_robot_pybullet/models/SP2_PBRmodel/sp2.urdf')

    def replace_placeholders(file_path, placeholder, value):
        with open(file_path, 'r') as file:
            content = yaml.safe_load(file)
        content_str = yaml.dump(content)
        content_str = content_str.replace(placeholder, value)
        updated_content = yaml.safe_load(content_str)
        return updated_content

    physics_engine_parameters_content = replace_placeholders(physics_engine_parameters_path, '{{ROS2_WS}}', ros2_ws)
    physics_parameters_content = replace_placeholders(physics_parameters_path, '{{ROS2_WS}}', ros2_ws)
    vsr_structure_content = replace_placeholders(vsr_structure_path, '{{ROS2_WS}}', ros2_ws)
    pbr_structure_content = replace_placeholders(pbr_structure_path, '{{ROS2_WS}}', ros2_ws)
    sp2_mesh_content = replace_placeholders(sp2_mesh_structure_path, '{{ROS2_WS}}', ros2_ws)
    
    # Replace placeholder in URDF file path
    urdf_file_path_content = urdf_file_path.replace('{{ROS2_WS}}', ros2_ws)

    temp_dir = os.path.join(os.path.dirname(__file__), 'temp')
    os.makedirs(temp_dir, exist_ok=True)

    def write_temp_yaml(content, filename):
        temp_path = os.path.join(temp_dir, filename)
        with open(temp_path, 'w') as file:
            yaml.safe_dump(content, file)
        return temp_path

    physics_engine_parameters_temp_path = write_temp_yaml(physics_engine_parameters_content, 'physics_engine_parameters.yaml')
    physics_parameters_temp_path = write_temp_yaml(physics_parameters_content, 'physics_parameters.yaml')
    vsr_structure_temp_path = write_temp_yaml(vsr_structure_content, 'vsr_structure_box.yaml')
    pbr_structure_temp_path = write_temp_yaml(pbr_structure_content, 'pbr_box.yaml')
    sp2_mesh_temp_path = write_temp_yaml(sp2_mesh_content, 'sp2.yaml')

    simulation_node = Node(
        package='virtual_shake_robot_pybullet',
        executable='simulation_node.py',
        name='simulation_node',
        output='screen',
        parameters=[
            physics_engine_parameters_temp_path,
            physics_parameters_temp_path,
            vsr_structure_temp_path,
            pbr_structure_temp_path,
            sp2_mesh_temp_path,
            {'urdf_file': urdf_file_path_content}  # Using the updated URDF file path
        ]
    )

    control_node = Node(
        package='virtual_shake_robot_pybullet',
        executable='control_node.py',
        name='control_node',
        output='screen'
    )

    return LaunchDescription([
        simulation_node,
        control_node    
    ])
