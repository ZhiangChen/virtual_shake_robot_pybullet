# README: Setting Contact Stiffness and Contact Damping in PyBullet

## Introduction

In physics simulations, particularly those involving rigid body dynamics, setting the correct contact parameters is crucial for realistic and stable interactions between objects. Two key parameters in this context are **contact stiffness** and **contact damping**. This document explains how to configure these parameters in PyBullet to achieve desired collision behaviors.

## Contact Parameters Overview

### Contact Stiffness
- **Definition**: Contact stiffness determines the resistance of an object to deformation upon collision. Higher values result in less deformation, mimicking harder materials.
- **Effect**: Affects the positional level of collision response. Higher stiffness reduces penetration depth but can lead to instability if set too high without proper damping.

### Contact Damping
- **Definition**: Contact damping controls the dissipation of kinetic energy during collisions. It acts like a damper in a spring-damper system.
- **Effect**: Affects the velocity level of collision response. Higher damping helps in reducing oscillations and stabilizing the simulation.

## Why Higher Mass Requires Higher Stiffness

Heavier objects exert more force upon collision due to their greater inertia. To prevent significant interpenetration and ensure realistic collision behavior, the contact stiffness must be increased proportionally. However, this also necessitates careful tuning of the damping to maintain stability.

## Example Script for Setting Contact Parameters

Here’s an example script demonstrating how to set up a simple simulation in PyBullet with specific contact stiffness and damping values:

```python
import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set up the environment
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

# Define the base of the shake table
base_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[25, 7, 1])
base_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[25, 7, 1], rgbaColor=[1, 1, 0, 1])  # Yellow color

# Define the slide pad of the shake table
slide_pad_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[5, 5, 0.05])
slide_pad_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[5, 5, 0.05], rgbaColor=[1, 0, 0, 1])  # Red color

# Create the base and slide pad as a single multi-body
base_position = [0, 0, 0.5]
base_orientation = p.getQuaternionFromEuler([0, 0, 0])

link_masses = [10000.0]
link_collision_shapes = [slide_pad_collision_shape]
link_visual_shapes = [slide_pad_visual_shape]
link_positions = [[0, 0, 1.025]]  # Adjusted to ensure the slide pad is above the base
link_orientations = [p.getQuaternionFromEuler([0, 0, 0])]
link_inertial_frame_positions = [[0, 0, 0]]
link_inertial_frame_orientations = [p.getQuaternionFromEuler([0, 0, 0])]
indices = [0]
joint_types = [p.JOINT_PRISMATIC]
joint_axis = [[1, 0, 0]]

base = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=base_collision_shape,
    baseVisualShapeIndex=base_visual_shape,
    basePosition=base_position,
    baseOrientation=base_orientation,
    linkMasses=link_masses,
    linkCollisionShapeIndices=link_collision_shapes,
    linkVisualShapeIndices=link_visual_shapes,
    linkPositions=link_positions,
    linkOrientations=link_orientations,
    linkInertialFramePositions=link_inertial_frame_positions,
    linkInertialFrameOrientations=link_inertial_frame_orientations,
    linkParentIndices=indices,
    linkJointTypes=joint_types,
    linkJointAxis=joint_axis
)

# Change contact stiffness for the slide pad
p.changeDynamics(base, 0, contactStiffness=1e7, contactDamping=0.5)
p.changeDynamics(base, 1, contactStiffness=1e7, contactDamping=0.5)

# Load a box on top of the shake table
box_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 1, 1.5])
box_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 1, 1.5], rgbaColor=[0, 0, 1, 1])  # Blue color
box_position = [0, 0, 4]

box = p.createMultiBody(baseMass=5000.0, baseCollisionShapeIndex=box_collision_shape, baseVisualShapeIndex=box_visual_shape, basePosition=box_position)

# Change contact stiffness for the box
p.changeDynamics(box, -1, contactStiffness=1e7, contactDamping=0.5)

# Run the simulation
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    pass

# Disconnect from PyBullet
p.disconnect()

```


## Handling Parameter Retrieval from .yaml Files

When retrieving parameters from a .yaml file, it is essential to first retrieve the parameters and then set them in the changeDynamics function. Setting them directly without retrieval can cause issues, as the parameters may not reflect correctly in the simulation. This issue can cause unexpected behavior or failures in the simulation setup.

Correct way of defining parameters : 

```
def load_pbr_callback(self, goal_handle):
    '''Load the PBR callback function'''

    self.logger.info("Loading PBR structure...")
    structure_name = goal_handle.request.structure_name
    structure_type = goal_handle.request.structure_type
    feedback_msg = LoadPBR.Feedback()

    if structure_name != "rock":
        self.logger.error(f"Unknown structure: {structure_name}")
        goal_handle.abort()
        return LoadPBR.Result(success=False)

    try:
        pedestal_height = 2.1 
        if structure_type == 'mesh':
            # Load the URDF file
            urdf_path = '/home/akshay/ros2_ws/virtual_shake_robot_pybullet/models/double_rock_pbr/pbr_mesh.urdf'

            self.get_logger().info(f"Loading rock mesh from {urdf_path}...")
            
            rock_position = [0, 0, pedestal_height + 1.2] 

            rock_id = p.loadURDF(urdf_path, rock_position, [0, 0, 0, 1], physicsClientId=self.client_id)

        elif structure_type == 'box':
            # Retrieve parameters from parameter server
            rock_dimensions = self.rock_structure_box_config['dimensions']
            rock_mass = self.rock_structure_box_config['mass']
            rock_restitution = self.rock_structure_box_config['restitution']
            rock_lateralFriction = self.rock_structure_box_config['lateralFriction']
            rock_spinningFriction = self.rock_structure_box_config['spinningFriction']
            rock_rollingFriction = self.rock_structure_box_config['rollingFriction']
            rock_contactDamping = self.rock_structure_box_config['contactDamping']
            rock_contactStiffness = self.rock_structure_box_config['contactStiffness']
            principal_moments = self.rock_structure_box_config['localInertiaDiagonal']

            self.get_logger().info("Loading rock as a box...")
            collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[d / 2 for d in rock_dimensions])
            visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[d / 2 for d in rock_dimensions], rgbaColor=[0.5, 0.5, 0.5, 1])
            rock_height = rock_dimensions[2]

            rock_position = [0, 0, 4.5]

            rock_id = p.createMultiBody(
                baseMass=rock_mass,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=rock_position,
                baseOrientation=[0, 0, 0, 1],
                physicsClientId=self.client_id
            )

            p.changeDynamics(
                rock_id, -1,
                restitution=rock_restitution,
                lateralFriction=rock_lateralFriction,
                spinningFriction=rock_spinningFriction,
                rollingFriction=rock_rollingFriction,
                contactDamping=rock_contactDamping,
                contactStiffness=rock_contactStiffness,
                localInertiaDiagonal=principal_moments,  
                physicsClientId=self.client_id
            )

            self.get_logger().info(f"Rock structure loaded with ID: {rock_id}")
            self.rock_id = rock_id

            feedback_msg.status = "Rock structure loaded successfully"
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()

            result = LoadPBR.Result()
            result.success = True
            return result

        except Exception as e:
            self.logger.error(f"Error loading PBR: {str(e)}")
            feedback_msg.status = f"Error loading PBR: {str(e)}"
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.abort()
            return LoadPBR.Result(success=False)
```


Setting the correct contact parameters in PyBullet is essential for realistic simulations. Always ensure parameters are correctly retrieved from configuration files before using them in functions like changeDynamics to avoid unexpected issues. This practice ensures that your simulations are stable, realistic, and reflective of the defined physical properties.