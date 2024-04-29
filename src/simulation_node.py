#!/usr/bin/env python3
# This script will basically act as a intermediary between ROS2 and pybullet 
#client  and also will subscribe to the data published by the control node 
import rclpy
import os
import yaml
import pybullet as p
import pybullet_data
import time
from rclpy.node import Node
from rclpy.node import ParameterDescriptor
import rclpy.type_support

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')

        ##Decalring parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ('engineSettings.gravity', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('engineSettings.timeStep', rclpy.Parameter.Type.DOUBLE),
                ('engineSettings.useSplitImpulse', rclpy.Parameter.Type.BOOL),
                ('engineSettings.splitImpulsePenetrationThreshold', rclpy.Parameter.Type.DOUBLE),
                ('engineSettings.enableConeFriction', rclpy.Parameter.Type.BOOL),
                ('engineSettings.deterministicOverlappingPairs', rclpy.Parameter.Type.INTEGER),
                ('engineSettings.numSolverIterations', rclpy.Parameter.Type.INTEGER),
                ('dynamics.world_box.restitution', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.world_box.lateralFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.world_box.spinningFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.world_box.rollingFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.world_box.contactDamping', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.world_box.contactStiffness', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.pedestal.restitution', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.pedestal.lateralFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.pedestal.spinningFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.pedestal.rollingFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.pedestal.contactDamping', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.pedestal.contactStiffness', rclpy.Parameter.Type.DOUBLE),
                ('structure.pedestal.mass', rclpy.Parameter.Type.DOUBLE),
                ('structure.pedestal.inertia', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.pedestal.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.pedestal.mesh', rclpy.Parameter.Type.STRING),
                ('structure.pedestal.meshScale', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.world_box.mass', rclpy.Parameter.Type.DOUBLE),
                ('structure.world_box.inertia', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.world_box.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY)
            ])
        ##Retrieve parameters from the parameter server

        self.engine_settings = {
            'gravity': self.get_parameter('engineSettings.gravity').value,
            'timestep': self.get_parameter('engineSettings.timeStep').value,
            'useSplitImpulse': self.get_parameter('engineSettings.useSplitImpulse').value,
            'splitImpulsePenetrationThreshold': self.get_parameter('engineSettings.splitImpulsePenetrationThreshold').value,
            'enableConeFriction': self.get_parameter('engineSettings.enableConeFriction').value,
            'deterministicOverlappingPairs': self.get_parameter('engineSettings.deterministicOverlappingPairs').value,
            'numSolverIterations': self.get_parameter('engineSettings.numSolverIterations').value,
        }

        self.dynamics_config = {
            'world_box': {
                'restitution': self.get_parameter('dynamics.world_box.restitution').value,
                'lateralFriction': self.get_parameter('dynamics.world_box.lateralFriction').value,
                'spinningFriction': self.get_parameter('dynamics.world_box.spinningFriction').value,
                'rollingFriction': self.get_parameter('dynamics.world_box.rollingFriction').value,
                'contactDamping': self.get_parameter('dynamics.world_box.contactDamping').value,
                'contactStiffness': self.get_parameter('dynamics.world_box.contactStiffness').value,
            },
            'pedestal': {
                'restitution': self.get_parameter('dynamics.pedestal.restitution').value,
                'lateralFriction': self.get_parameter('dynamics.pedestal.lateralFriction').value,
                'spinningFriction': self.get_parameter('dynamics.pedestal.spinningFriction').value,
                'rollingFriction': self.get_parameter('dynamics.pedestal.rollingFriction').value,
                'contactDamping': self.get_parameter('dynamics.pedestal.contactDamping').value,
                'contactStiffness': self.get_parameter('dynamics.pedestal.contactStiffness').value,
            },
            
            }


        self.structure_config = {
            'pedestal': {
                'mass': self.get_parameter('structure.pedestal.mass').value,
                'inertia': self.get_parameter('structure.pedestal.inertia').value,
                'dimensions': self.get_parameter('structure.pedestal.dimensions').value,
                'meshScale': self.get_parameter('structure.pedestal.meshScale').value
            },
            
            
            'world_box': {
                'mass': self.get_parameter('structure.world_box.mass').value,
                'inertia': self.get_parameter('structure.world_box.inertia').value,
                'dimensions': self.get_parameter('structure.world_box.dimensions').value
            }
        }

        self.client = []
        

      

    def server_connection(self):
        """establish a connection the the Pybullet GUI"""
        client = p.connect(p.GUI) #will return a client ID
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.client.append(client) # adding in the list t
        return client
    

    def create_servers(self, number_of_servers):
        """Create multiple PyBullet physics servers. """
        for _ in  range(number_of_servers):
            client = p.connect(p.DIRECT)
            self.client.append(client)

    def setup_simulation(self, client_id):
        '''this helps to setup the simulation using the physics_engine parmeters file'''
        gravity = self.get_parameter('engineSettings.gravity').value
        print(f"Gravity retrived : {gravity}")
        time_step = self.get_parameter('engineSettings.timeStep').value
        num_solver_iterations = self.get_parameter('engineSettings.numSolverIterations').value
        use_split_impulse = self.get_parameter('engineSettings.useSplitImpulse').value
        split_impulse_threshold = self.get_parameter('engineSettings.splitImpulsePenetrationThreshold').value
        enable_cone_friction = self.get_parameter('engineSettings.enableConeFriction').value
        overlapping_pairs = self.get_parameter('engineSettings.deterministicOverlappingPairs').value

        # Use these settings to configure the physics engine
        p.setGravity(*gravity, physicsClientId=client_id)
        print(f"Gravity set for the client {client_id}:{gravity}")
        p.setTimeStep(time_step, physicsClientId=client_id)
        p.setPhysicsEngineParameter(numSolverIterations=num_solver_iterations,
                                    useSplitImpulse=use_split_impulse,
                                    splitImpulsePenetrationThreshold=split_impulse_threshold,
                                    enableConeFriction=enable_cone_friction,
                                    deterministicOverlappingPairs=overlapping_pairs,
                                    physicsClientId=client_id)


    def subscribing_control_topic(self):
        """this will subscribe to the control node to send the commands to the robot in simualtion"""
        pass


    
    def create_robot(self, client_id):
            
        
        """Create the robot using dynamics and structure settings with joints, using a mesh for the pedestal."""
        p.loadURDF("plane.urdf", physicsClientId=client_id)

        # Create the collision and visual shapes for world_box and pedestal
        world_box_shape = {
            'collision': p.createCollisionShape(p.GEOM_BOX, halfExtents=[x / 2 for x in self.structure_config['world_box']['dimensions']]),
            'visual': p.createVisualShape(p.GEOM_BOX, halfExtents=[x / 2 for x in self.structure_config['world_box']['dimensions']])
        }

        # Fetch the mesh path for the pedestal from ROS2 parameters
        pedestal_mesh_path = self.get_parameter('structure.pedestal.mesh').get_parameter_value().string_value

        self.get_logger().info(f"Using meshScale: {self.structure_config['pedestal']['meshScale']}")
    
        # Load the mesh for the pedestal
        pedestal_shape = {
        'collision': p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=pedestal_mesh_path, meshScale=self.structure_config['pedestal']['meshScale']),
        'visual': p.createVisualShape(shapeType=p.GEOM_MESH, fileName=pedestal_mesh_path, meshScale=self.structure_config['pedestal']['meshScale'])
        }


        # Define the mass, collision shape, and visual shape for the pedestal link
        link_masses = [self.structure_config['pedestal']['mass']]
        link_collision_shape_indices = [pedestal_shape['collision']]
        link_visual_shape_indices = [pedestal_shape['visual']]
        link_positions = [[0, 0, self.structure_config['world_box']['dimensions'][2] + self.structure_config['pedestal']['dimensions'][2]/2]]
        link_orientations = [[0, 0, 0, 1]]
        link_inertial_frame_positions = [[0, 0, 0]]
        link_inertial_frame_orientations = [[0, 0, 0, 1]]
        link_parent_indices = [0]  # Linking to the base
        link_joint_types = [p.JOINT_PRISMATIC]
        link_joint_axes = [[0, 0, 1]]  # Prismatic joint allowing movement along the z-axis

        # Create the base (world_box) with a prismatic joint to the pedestal
        vsr_id = p.createMultiBody(
            baseMass=self.structure_config['world_box']['mass'],
            baseCollisionShapeIndex=world_box_shape['collision'],
            baseVisualShapeIndex=world_box_shape['visual'],
            basePosition=[0, 0, self.structure_config['world_box']['dimensions'][2]/2],
            baseInertialFramePosition=[0, 0, 0],
            linkMasses=link_masses,
            linkCollisionShapeIndices=link_collision_shape_indices,
            linkVisualShapeIndices=link_visual_shape_indices,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkInertialFramePositions=link_inertial_frame_positions,
            linkInertialFrameOrientations=link_inertial_frame_orientations,
            linkParentIndices=link_parent_indices,
            linkJointTypes=link_joint_types,
            linkJointAxis=link_joint_axes,
            physicsClientId=client_id
        )
        assert vsr_id != -1, f"Failed to create the robot body. Returned ID: {vsr_id}"
        # Change dynamics of the base
        p.changeDynamics(
            vsr_id, -1,
            restitution=self.dynamics_config['world_box']['restitution'],
            lateralFriction=self.dynamics_config['world_box']['lateralFriction'],
            spinningFriction=self.dynamics_config['world_box']['spinningFriction'],
            rollingFriction=self.dynamics_config['world_box']['rollingFriction'],
            contactDamping=self.dynamics_config['world_box']['contactDamping'],
            contactStiffness=self.dynamics_config['world_box']['contactStiffness'],
            physicsClientId=client_id
        )

        


    def run_simulation(self, client, duration=5):
        """run the simulation for a specfic number of time steps"""
        start_time = time.time()
        while (time.time()- start_time) < duration:
            p.stepSimulation(client)
            time.sleep(1./240.) # Simulation time step       

        

    def disconnect(self): 
        """disconect all the client"""

        for client in self.client:
            p.disconnect(client)

def main(args=None):
    rclpy.init(args=args)
    simulation_node = SimulationNode()
    simulation_node.server_connection()  # Connect GUI for visualization
    simulation_node.create_servers(2)
    simulation_node.setup_simulation(simulation_node.client[0])
    simulation_node.create_robot(simulation_node.client[0])
    simulation_node.run_simulation(simulation_node.client[0], duration=5000)
    simulation_node.disconnect()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
