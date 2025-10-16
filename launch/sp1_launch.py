
#!/usr/bin/env python3
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
import os

def generate_launch_description():

    # Declare the motion_mode argument
    motion_mode_arg = DeclareLaunchArgument(
        'motion_mode',
        default_value='',
        description='Motion mode for the control node: grid_cosine, single_recording, all_recordings'
    )

    test_no_arg = DeclareLaunchArgument(
        'test_no',
        default_value='0',
        description='Test number'
    )

    test_number_range_arg = DeclareLaunchArgument(
        'test_number_range',
        default_value='',
        description='Range of test numbers for single_recording_range mode (e.g., "1-100")'
    )

    ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))

    # Paths to configuration files
    physics_engine_parameters_path = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/config', 'physics_engine_parameters.yaml')
    physics_parameters_path = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/config', 'physics_parameters.yaml')
    vsr_structure_path = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/config', 'vsr_structure_box.yaml')
    pbr_structure_box_path = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/config', 'pbr_box.yaml')
    pbr_structure_mesh_path = os.path.join(ros2_ws, 'src/virtual_shake_robot_pybullet/config', 'sp1.yaml')

    DeclareLaunchArgument('test_number_range', default_value='')


    # Simulation node for namespace sim_1 using the YAML file
    simulation_node = Node(
        package='virtual_shake_robot_pybullet',
        executable='simulation_node.py',
        name='simulation_node',
        namespace='sim_1',
        output='screen',
        parameters=[
            physics_engine_parameters_path,
            physics_parameters_path,
            vsr_structure_path,
            pbr_structure_box_path,
            pbr_structure_mesh_path,
            #{'recording_folder_name': LaunchConfiguration('recording_folder_name')}
        ]
    )

    # Control node for namespace sim_1
    control_node = Node(
        package='virtual_shake_robot_pybullet',
        executable='control_node.py',
        name='control_node',
        namespace='sim_1',
        output='screen',
        parameters=[
            {'motion_mode': LaunchConfiguration('motion_mode')},
            {'test_no': LaunchConfiguration('test_no')},
            {'test_number_range': LaunchConfiguration('test_number_range')}
        ]
    )

    return LaunchDescription([
        motion_mode_arg,
        test_number_range_arg,
        GroupAction([
            PushRosNamespace('sim_1'),
            simulation_node,
            control_node    
        ])
    ])
        