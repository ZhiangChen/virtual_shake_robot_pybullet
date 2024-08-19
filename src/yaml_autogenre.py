import os
import yaml

def generate_yaml_files(output_dir, base_content, parameter_ranges):
    os.makedirs(output_dir, exist_ok=True)

    for restitution in parameter_ranges.get('restitution', [0.3]):
        for friction in parameter_ranges.get('friction', [0.3]):
            content = base_content.copy()
            content['simulation_node']['ros__parameters']['rock_structure_mesh']['restitution'] = restitution
            content['simulation_node']['ros__parameters']['rock_structure_mesh']['lateralFriction'] = friction
            content['simulation_node']['ros__parameters']['rock_structure_mesh']['spinningFriction'] = friction
            content['simulation_node']['ros__parameters']['rock_structure_mesh']['rollingFriction'] = friction

            filename = f'restitution_{restitution:.2f}_friction_{friction:.2f}.yaml'
            file_path = os.path.join(output_dir, filename)

            with open(file_path, 'w') as file:
                yaml.dump(content, file, default_flow_style=None, sort_keys=False, indent=2, width=120)

            print(f'Generated: {file_path}')

def main():
    base_yaml_content = {
        'simulation_node': {
            'ros__parameters': {
                'rock_structure_mesh': {
                    'meshScale': [1.0, 1.0, 1.0],
                    'mass': 0.105,
                    'restitution': 0.3,
                    'lateralFriction': 0.3,
                    'spinningFriction': 0.3,
                    'rollingFriction': 0.3,
                    'contactDamping': 1.0,
                    'contactStiffness': 100000.0,
                    'rock_position': [0.0, 0.0, 3.3]
                }
            }
        }
    }

    restitution_range = list(map(float, input("Enter restitution values (comma-separated): ").split(',')))
    friction_range = list(map(float, input("Enter friction values (comma-separated): ").split(',')))

    parameter_ranges = {
        'restitution': restitution_range,
        'friction': friction_range,
    }

    output_directory = 'generated_files'
    generate_yaml_files(output_directory, base_yaml_content, parameter_ranges)

if __name__ == "__main__":
    main()
