
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

    physics_engine_parameters_path = os.path.join('/home/akshay/ros2_ws/virtual_shake_robot_pybullet/config', 'physics_engine_parameters.yaml')
    physics_parameters_path = os.path.join('/home/akshay/ros2_ws/virtual_shake_robot_pybullet/config', 'physics_parameters.yaml')
    vsr_structure_path = os.path.join('/home/akshay/ros2_ws/virtual_shake_robot_pybullet/config', 'vsr_structure_box.yaml')
    pbr_structure_path = os.path.join('/home/akshay/ros2_ws/virtual_shake_robot_pybullet/config', 'pbr_box.yaml')

    # Perception node for namespace sim_37
    perception_node_sim_37 = Node(
        package='virtual_shake_robot_pybullet',
        executable='perception_node.py',
        name='perception_node',
        namespace='sim_37',
        output='screen'
    )

    # Simulation node for namespace sim_37 using the YAML file
    simulation_node_sim_37 = Node(
        package='virtual_shake_robot_pybullet',
        executable='simulation_node.py',
        name='simulation_node',
        namespace='sim_37',
        output='screen',
        parameters=[
            physics_engine_parameters_path,
            physics_parameters_path,
            vsr_structure_path,
            pbr_structure_path,
            '/home/akshay/ros2_ws/virtual_shake_robot_pybullet/config/sp1_36.yaml'
        ]
    )

    control_node_sim_37 = Node(
        package='virtual_shake_robot_pybullet',
        executable='control_node.py',
        name='control_node',
        namespace='sim_37',
        output='screen',
        parameters=[
            {'motion_mode': LaunchConfiguration('motion_mode')}
        ]
    )

    return LaunchDescription([
        motion_mode_arg,
        GroupAction([
            PushRosNamespace('sim_37'),
            perception_node_sim_37,
            simulation_node_sim_37,
            control_node_sim_37    
        ])
    ])
        