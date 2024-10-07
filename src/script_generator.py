import os
import yaml
import itertools

def generate_yaml_files(output_dir, base_content, parameter_combinations, model_choice):
    os.makedirs(output_dir, exist_ok=True)

    ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))
    
    # Update the mesh path based on the model choice
    if model_choice == 'sp1':
        mesh_path = os.path.join(ros2_ws, 'src','virtual_shake_robot_pybullet/models/SP1_PBRmodel/sp1.urdf')
    elif model_choice == 'sp2':
        mesh_path = os.path.join(ros2_ws, 'src','virtual_shake_robot_pybullet/models/SP2_PBRmodel/sp2.urdf')
    else:  # default to pbr_mesh'
        mesh_path = os.path.join(ros2_ws, 'src','virtual_shake_robot_pybullet/models/double_rock_pbr/pbr_mesh.urdf')

    yaml_files = []

    for idx, params in enumerate(parameter_combinations):
        content = base_content.copy()
        content['/**']['ros__parameters']['rock_structure_mesh'].update(params)
        content['/**']['ros__parameters']['rock_structure_mesh']['mesh'] = mesh_path  # Set the correct mesh path

        # Naming each YAML file sequentially
        filename = f"{model_choice}_{idx}.yaml"
        file_path = os.path.join(output_dir, filename)

        # Check if the file already exists to avoid overwriting
        if os.path.exists(file_path):
            print(f"Duplicate found, skipping: {file_path}")
            continue

        with open(file_path, 'w') as file:
            yaml.safe_dump(content, file, default_flow_style=None, sort_keys=False, indent=2, width=120)

        print(f'Generated: {file_path}')
        yaml_files.append(file_path)

    return yaml_files

def generate_launch_files(output_dir, yaml_files, model_choice):
    ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))
    config_directory = os.path.join('{{ROS2_WS}}', 'src','virtual_shake_robot_pybullet/config')  # Placeholder for ROS2_WS
    launch_directory = os.path.join('{{ROS2_WS}}', 'src','virtual_shake_robot_pybullet/launch')  # Placeholder for ROS2_WS

    launch_file_paths = []

    for i, yaml_file in enumerate(yaml_files):
        # Calculate dynamic namespaces
        sim_ns = f"sim_{i+1}"

        launch_content = f"""
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

    physics_engine_parameters_path = os.path.join('{config_directory}', 'physics_engine_parameters.yaml')
    physics_parameters_path = os.path.join('{config_directory}', 'physics_parameters.yaml')
    vsr_structure_path = os.path.join('{config_directory}', 'vsr_structure_box.yaml')
    pbr_structure_path = os.path.join('{config_directory}', 'pbr_box.yaml')

    # Perception node for namespace {sim_ns}
    perception_node_{sim_ns} = Node(
        package='virtual_shake_robot_pybullet',
        executable='perception_node.py',
        name='perception_node',
        namespace='{sim_ns}',
        output='screen'
    )

    # Simulation node for namespace {sim_ns} using the YAML file
    simulation_node_{sim_ns} = Node(
        package='virtual_shake_robot_pybullet',
        executable='simulation_node.py',
        name='simulation_node',
        namespace='{sim_ns}',
        output='screen',
        parameters=[
            physics_engine_parameters_path,
            physics_parameters_path,
            vsr_structure_path,
            pbr_structure_path,
            '{yaml_file}'
        ]
    )

    control_node_{sim_ns} = Node(
        package='virtual_shake_robot_pybullet',
        executable='control_node.py',
        name='control_node',
        namespace='{sim_ns}',
        output='screen',
        parameters=[
            {{'motion_mode': LaunchConfiguration('motion_mode')}}
        ]
    )

    return LaunchDescription([
        motion_mode_arg,
        GroupAction([
            PushRosNamespace('{sim_ns}'),
            perception_node_{sim_ns},
            simulation_node_{sim_ns},
            control_node_{sim_ns}    
        ])
    ])
        """

        # Replace placeholder with the correct ROS2_WS directory
        launch_content = launch_content.replace('{{ROS2_WS}}', ros2_ws)

        # Naming each launch file sequentially
        launch_filename = os.path.join(output_dir, f'{model_choice}_{i}.launch.py')
        with open(launch_filename, 'w') as file:
            file.write(launch_content)
        launch_file_paths.append(launch_filename)
        print(f'Generated: {launch_filename}')

    return launch_file_paths

def generate_master_launch_file(launch_files, output_dir, model_choice):
    master_launch_content = """
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
    """

    for launch_file in launch_files:
        master_launch_content += f"""
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('{launch_file}'))
        ),
        """

    master_launch_content += """
    ])
    """

    master_launch_filename = os.path.join(output_dir, f'{model_choice}_master_launch.py')
    with open(master_launch_filename, 'w') as file:
        file.write(master_launch_content)
    print(f'Generated master launch file: {master_launch_filename}')

def main():
    base_yaml_content = {
        '/**': {
            'ros__parameters': {
                'rock_structure_mesh': {
                    'meshScale': [1.0, 1.0, 1.0],
                    'mass': 353.8022,
                    'restitution': 0.3,
                    'lateralFriction': 0.3,
                    'spinningFriction': 0.3,
                    'contactDamping': 1.0,
                    'contactStiffness': 100000.0,
                    'rock_position': [0.0, 0.0, 2.80]
                }
            }
        }
    }

    model_choice = input("Select a model (pbr_mesh, sp1, sp2): ").strip().lower()

    # Hardcoded ranges for the parameters
    restitution_range = [0.3, 0.4, 0.5]
    lateral_friction_range = [0.3, 0.5]
    spinning_friction_range = [0.3, 0.5]
    contact_damping_range = [10000.0, 20000.0]
    contact_stiffness_range = [1000000.0, 2000000.0]

    # Generate all combinations of the parameters
    parameter_combinations = [
        {'restitution': r, 'lateralFriction': lf, 'spinningFriction': sf, 'contactDamping': cd, 'contactStiffness': cs}
        for r, lf, sf, cd, cs in itertools.product(restitution_range, lateral_friction_range, spinning_friction_range, contact_damping_range, contact_stiffness_range)
    ]

    output_directory = os.path.join(os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws')),'src', 'virtual_shake_robot_pybullet/config')
    yaml_files = generate_yaml_files(output_directory, base_yaml_content, parameter_combinations, model_choice)

    launch_files = generate_launch_files(os.path.join(os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws')), 'src','virtual_shake_robot_pybullet/launch'), yaml_files, model_choice)

    generate_master_launch_file(launch_files, os.path.join(os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws')), 'src','virtual_shake_robot_pybullet/launch'), model_choice)

if __name__ == "__main__":
    main()
