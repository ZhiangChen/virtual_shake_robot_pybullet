import pybullet as p
import pybullet_data
import time
from math import sin, cos, pi

# Initialize PyBullet with GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load a plane
plane_id = p.loadURDF("plane.urdf")

# Define block parameters
block_half_extents = [0.2, 0.2, 0.2]

# Create base block (fixed to the ground)
base_block_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=block_half_extents)
base_block_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=block_half_extents, rgbaColor=[1, 0, 0, 1])

base_block_id = p.createMultiBody(
    baseMass=0,  # Fixed block
    baseCollisionShapeIndex=base_block_collision,
    baseVisualShapeIndex=base_block_visual,
    basePosition=[0, 0, block_half_extents[2]],  # Positioned on the ground
)

# Create top block with a planar joint
top_block_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=block_half_extents)
top_block_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=block_half_extents, rgbaColor=[0, 1, 0, 1])

top_block_id = p.createMultiBody(
    baseMass=1.0,  # Movable block
    baseCollisionShapeIndex=top_block_collision,
    baseVisualShapeIndex=top_block_visual,
    basePosition=[0, 0, 2 * block_half_extents[2]],  # Positioned above the base block
    baseOrientation=[0, 0, 0, 1],  # Default orientation
    linkMasses=[],  # No links
    linkCollisionShapeIndices=[],
    linkVisualShapeIndices=[],
    linkPositions=[],
    linkOrientations=[],
    linkInertialFramePositions=[],
    linkInertialFrameOrientations=[],
    linkParentIndices=[],
    linkJointTypes=[p.JOINT_PLANAR],  # Planar joint for X-Y motion
    linkJointAxis=[[1, 1, 0]]  # Allow X-Y motion
)

# Motion parameters for the top block
duration = 10.0  # Total duration
timestep = 0.01  # Time step
frequency = 0.5  # Frequency of motion
amplitude_xy = 0.5  # Amplitude for sinusoidal motion in X and Y

print("Step | Position (X, Y) | Velocity (X, Y)")

# Simulate motion
num_steps = int(duration / timestep)
for step in range(num_steps):
    t = step * timestep

    # Target positions for the planar joint
    target_x = amplitude_xy * sin(2 * pi * frequency * t)
    target_y = amplitude_xy * cos(2 * pi * frequency * t)

    # Apply motion using the planar joint
    p.setJointMotorControlMultiDof(
        top_block_id,
        0,  # Planar joint index
        p.POSITION_CONTROL,
        targetPosition=[target_x, target_y, 0],  # Motion in X-Y plane
        force=[500, 500, 0],  # Forces applied in X, Y directions
    )

    # Get and log the joint state
    joint_state = p.getJointStateMultiDof(top_block_id, 0)
    position = joint_state[0]  # Multi-dimensional position
    velocity = joint_state[1]  # Multi-dimensional velocity
    print(f"{step} | Position: {position[:2]} | Velocity: {velocity[:2]}")

    p.stepSimulation()
    time.sleep(timestep)

# Disconnect
p.disconnect()
