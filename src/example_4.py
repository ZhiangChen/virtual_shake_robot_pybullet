import pybullet as p
import pybullet_data
import time
from math import pi, sin, cos

# Initialize PyBullet with GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load the plane
plane_id = p.loadURDF("plane.urdf")

# Base Parameters
base_half_extents = [0.5, 0.5, 0.1]
base_position = [0, 0, base_half_extents[2]]
base_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=base_half_extents)
base_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=base_half_extents, rgbaColor=[1, 1, 0, 1])

# Movable Block Parameters
block_half_extents = [0.3, 0.3, 0.1]
block_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=block_half_extents)
block_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=block_half_extents, rgbaColor=[0, 0, 1, 1])

# Joint and Link Configuration
link_masses = [1.0, 1.0, 1.0, 1.0]  # Prismatic (X, Y, Z) and Spherical
link_collision_shapes = [-1, -1, -1, block_collision_shape]
link_visual_shapes = [-1, -1, -1, block_visual_shape]
link_positions = [[0, 0, 0.1], [0, 0, 0.2], [0, 0, 0.3], [0, 0, 0.4]]
link_orientations = [[0, 0, 0, 1]] * 4
link_inertial_positions = [[0, 0, 0]] * 4
link_inertial_orientations = [[0, 0, 0, 1]] * 4
link_parent_indices = [0, 1, 2, 3]

# Joint Types and Axes
joint_types = [p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_SPHERICAL]
joint_axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0]]  # X, Y, Z translation, and rotation

# Create MultiBody
robot_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=base_collision_shape,
    baseVisualShapeIndex=base_visual_shape,
    basePosition=base_position,
    linkMasses=link_masses,
    linkCollisionShapeIndices=link_collision_shapes,
    linkVisualShapeIndices=link_visual_shapes,
    linkPositions=link_positions,
    linkOrientations=link_orientations,
    linkInertialFramePositions=link_inertial_positions,
    linkInertialFrameOrientations=link_inertial_orientations,
    linkParentIndices=link_parent_indices,
    linkJointTypes=joint_types,
    linkJointAxis=joint_axes
)

# Set Dynamics for Realistic Interaction
for i in range(len(link_masses)):
    p.changeDynamics(robot_id, i, lateralFriction=0.8, spinningFriction=0.3, restitution=0.1)

# Motion Parameters
duration = 20.0   # Total duration
timestep = 0.01  # Time step
frequency = 0.5  # Frequency for motion
amplitude_translation = 0.5  # Amplitude for X, Y, Z translation
rotation_amplitude = pi / 12  # Amplitude for spherical joint rotation

print("Joint State Log:")
print("Step | Joint ID | Position | Velocity")

# Apply Motion and Log Joint States
num_steps = int(duration / timestep)
for i in range(num_steps):
    t = i * timestep

    # Prismatic Joints: Translation in X, Y, Z
    target_x = amplitude_translation * sin(2 * pi * frequency * t)
    target_y = amplitude_translation * cos(2 * pi * frequency * t)
    target_z = amplitude_translation * sin(2 * pi * frequency * t)

    p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, targetPosition=target_x, force=500)
    p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition=target_y, force=500)
    p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=target_z, force=500)

    # Spherical Joint: Roll, Pitch, Yaw
    roll = rotation_amplitude * sin(2 * pi * frequency * t)
    pitch = rotation_amplitude * cos(2 * pi * frequency * t)
    yaw = rotation_amplitude * sin(2 * pi * frequency * t / 2)
    target_orientation = p.getQuaternionFromEuler([roll, pitch, yaw])

    p.setJointMotorControlMultiDof(robot_id, 3, p.POSITION_CONTROL, targetPosition=target_orientation)

    # Log joint states
    for joint_id in range(p.getNumJoints(robot_id)):
        if joint_id == 3:  # Spherical joint
            joint_state = p.getJointStateMultiDof(robot_id, joint_id)
            position = joint_state[0]  # Multi-dimensional position
            velocity = joint_state[1]  # Multi-dimensional velocity
        else:  # Prismatic joints
            joint_state = p.getJointState(robot_id, joint_id)
            position = joint_state[0]  # Scalar position
            velocity = joint_state[1]  # Scalar velocity
        print(f"{i} | Joint {joint_id} | Position: {position} | Velocity: {velocity}")

    p.stepSimulation()
    time.sleep(timestep)

# Disconnect
p.disconnect()
