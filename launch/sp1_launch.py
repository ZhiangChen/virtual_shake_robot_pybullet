#!/usr/bin/env python3
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Declare the motion_mode argument
    motion_mode_arg = DeclareLaunchArgument(
        'motion_mode',
        default_value='',
        description='Motion mode for the control node: grid_cosine, single_recording, all_recordings'
    )
    
    # Declare the test_no argument (conditionally passed later)
    test_no_arg = DeclareLaunchArgument(
        'test_no',
        default_value='0',  # Default value
        description='Test number for single_recording mode'
    )

    # Set the ROS2 workspace path
    ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))
    config_directory = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/config')

    # Define paths to YAML configuration files
    physics_engine_parameters_path = os.path.join(config_directory, 'physics_engine_parameters.yaml')
    physics_parameters_path = os.path.join(config_directory, 'physics_parameters.yaml')
    vsr_structure_path = os.path.join(config_directory, 'vsr_structure_box.yaml')
    pbr_structure_path = os.path.join(config_directory, 'pbr_box.yaml')
    sp1_mesh_structure_path = os.path.join(config_directory, 'sp1.yaml')

    # Define URDF file path
    urdf_file_path = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/models/SP1_PBRmodel/sp1.urdf')

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
            sp1_mesh_structure_path,
            {'urdf_file': urdf_file_path}  # Using the direct URDF file path
        ]
    )

    return LaunchDescription([
        motion_mode_arg,
        test_no_arg,
        simulation_node,
        OpaqueFunction(function=launch_setup)  # Use OpaqueFunction to conditionally include the test_no parameter
    ])
