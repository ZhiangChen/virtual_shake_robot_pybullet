#!/usr/bin/env python3
# This script will basically act as a intermediary between ROS2 and pybullet
#client  and also will subscribe to the data published by the control node
import rclpy
import os
import yaml
import pybullet as p
import pybullet_data
import time
import numpy as np
from rclpy.node import Node
from rclpy.node import ParameterDescriptor
from rclpy.action import ActionServer
import rclpy.type_support
from virtual_shake_robot_pybullet.action import AF

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self._action_server = ActionServer(
            self,
            AF,
            'execute_movement',
            self.execute_callback
        )
        self.get_logger().info("Action server up and running!")

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
                ('structure.pedestal.mesh',rclpy.Parameter.Type.STRING),
                ('structure.pedestal.meshScale',rclpy.Parameter.Type.DOUBLE_ARRAY),
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

    


    def execute_callback(self, goal_handle):
        '''retrieve values from the action server'''
        a = goal_handle.request.a
        f = goal_handle.request.f

        self.get_logger().debug(f'Received amplitude (a): {a}')
        self.get_logger().debug(f'Received frequency (f): {f}')

        # Calculate velocities based on the received values
        velocities = self.calculate_target_velocity(a, f)

        self.get_logger.debug(f'Calculated velocity : {velocities}')

        # Apply these velocities to the pedestal joint
        self.apply_joint_velocities(velocities)

        # Assuming successful application of velocities
        goal_handle.succeed()
        result = AF.Result()
        result.success = True
        return result


    def create_robot(self, client_id):
        p.loadURDF("plane.urdf", physicsClientId=client_id)

        # Define shapes and properties for world_box
        world_box_shape = {
            'collision': p.createCollisionShape(p.GEOM_BOX, halfExtents=[x / 2 for x in self.structure_config['world_box']['dimensions']]),
            'visual': p.createVisualShape(p.GEOM_BOX, halfExtents=[x / 2 for x in self.structure_config['world_box']['dimensions']])
        }
        world_box_id = p.createMultiBody(baseMass=self.structure_config['world_box']['mass'],
                                        baseCollisionShapeIndex=world_box_shape['collision'],
                                        baseVisualShapeIndex=world_box_shape['visual'],
                                        basePosition=[0, 0, self.structure_config['world_box']['dimensions'][2] / 2],
                                        physicsClientId=client_id)

        # Attempt to retrieve mesh parameters safely
        try:
            pedestal_mesh_path = self.get_parameter('structure.pedestal.mesh').value
            mesh_scale = self.get_parameter('structure.pedestal.meshScale').value
            pedestal_shape = {
                'collision': p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=pedestal_mesh_path, meshScale=mesh_scale),
                'visual': p.createVisualShape(shapeType=p.GEOM_MESH, fileName=pedestal_mesh_path, meshScale=mesh_scale)
            }
        except rclpy.exceptions.ParameterUninitializedException:
            # Fallback to box dimensions if mesh parameters are not found
            pedestal_shape = {
                'collision': p.createCollisionShape(p.GEOM_BOX, halfExtents=[d / 2 for d in self.structure_config['pedestal']['dimensions']]),
                'visual': p.createVisualShape(p.GEOM_BOX, halfExtents=[d / 2 for d in self.structure_config['pedestal']['dimensions']], rgbaColor=[1, 0, 0, 1])
            }

        # Create and configure pedestal
        vsr_id = p.createMultiBody(baseMass=self.structure_config['pedestal']['mass'],
                                        baseCollisionShapeIndex=pedestal_shape['collision'],
                                        baseVisualShapeIndex=pedestal_shape['visual'],
                                        basePosition=[0, 0, 2.1],
                                        physicsClientId=client_id)
        # Apply dynamics settings to pedestal
        p.changeDynamics(
            vsr_id, -1,
            restitution=self.dynamics_config['pedestal']['restitution'],
            lateralFriction=self.dynamics_config['pedestal']['lateralFriction'],
            spinningFriction=self.dynamics_config['pedestal']['spinningFriction'],
            rollingFriction=self.dynamics_config['pedestal']['rollingFriction'],
            contactDamping=self.dynamics_config['pedestal']['contactDamping'],
            contactStiffness=self.dynamics_config['pedestal']['contactStiffness'],
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
    simulation_node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    rclpy.spin(simulation_node)
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
