#!/usr/bin/env python3
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():

    # Declare the motion_mode argument
    motion_mode_arg = DeclareLaunchArgument(
        'motion_mode',
        default_value='',
        description='Motion mode for the control node: grid_cosine, single_recording, all_recordings'
    )
    
    # Declare the test_no argument (but we will conditionally pass it later)
    test_no_arg = DeclareLaunchArgument(
        'test_no',
        default_value='',  # Set a default value
        description='Test number for single_recording mode'
    )

    ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))
    config_directory = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/config')

    physics_engine_parameters_path = os.path.join(config_directory, 'physics_engine_parameters.yaml')
    physics_parameters_path = os.path.join(config_directory, 'physics_parameters.yaml')
    vsr_structure_path = os.path.join(config_directory, 'vsr_structure_box.yaml')
    pbr_structure_path = os.path.join(config_directory, 'pbr_box.yaml')
    sp1_mesh_structure_path = os.path.join(config_directory, 'sp1.yaml')

    # Define URDF file path with a placeholder
    urdf_file_path = os.path.join('{{ROS2_WS}}', 'src/virtual_shake_robot_pybullet/models/SP1_PBRmodel/sp1.urdf')

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
    sp1_mesh_content = replace_placeholders(sp1_mesh_structure_path, '{{ROS2_WS}}', ros2_ws)

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
    sp1_mesh_temp_path = write_temp_yaml(sp1_mesh_content, 'sp1.yaml')

    # Function to handle passing the test_no only if motion_mode is 'single_recording'
    def launch_setup(context, *args, **kwargs):
        motion_mode = LaunchConfiguration('motion_mode').perform(context)

        control_node_params = [
            {'motion_mode': LaunchConfiguration('motion_mode')}
        ]

        # Add test_no parameter only if motion_mode is 'single_recording'
        if motion_mode == 'single_recording':
            control_node_params.append({'test_no': LaunchConfiguration('test_no')})

        control_node = Node(
            package='virtual_shake_robot_pybullet',
            executable='control_node.py',
            name='control_node',
            output='screen',
            parameters=control_node_params
        )

        return [control_node]

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
            sp1_mesh_temp_path,
            {'urdf_file': urdf_file_path_content}  # Using the updated URDF file path
        ]
    )

    return LaunchDescription([
        motion_mode_arg,
        test_no_arg,
        simulation_node,
        OpaqueFunction(function=launch_setup)  # Use OpaqueFunction to conditionally include the test_no parameter
    ])
