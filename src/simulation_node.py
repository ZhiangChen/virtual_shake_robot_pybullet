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
                ('dynamics.box.restitution', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.box.lateralFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.box.spinningFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.box.rollingFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.box.contactDamping', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.box.contactStiffness', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.slide_box.restitution', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.slide_box.lateralFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.slide_box.spinningFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.slide_box.rollingFriction', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.slide_box.contactDamping', rclpy.Parameter.Type.DOUBLE),
                ('dynamics.slide_box.contactStiffness', rclpy.Parameter.Type.DOUBLE),
                ('structure.box.mass', rclpy.Parameter.Type.DOUBLE),
                ('structure.box.inertia', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.box.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.slide_box.inertia', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.slide_box.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.slide_box.mass', rclpy.Parameter.Type.DOUBLE),
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
            'box': {
                'restitution': self.get_parameter('dynamics.box.restitution').value,
                'lateralFriction': self.get_parameter('dynamics.box.lateralFriction').value,
                'spinningFriction': self.get_parameter('dynamics.box.spinningFriction').value,
                'rollingFriction': self.get_parameter('dynamics.box.rollingFriction').value,
                'contactDamping': self.get_parameter('dynamics.box.contactDamping').value,
                'contactStiffness': self.get_parameter('dynamics.box.contactStiffness').value,
            },
            'slide_box': {
                'restitution': self.get_parameter('dynamics.slide_box.restitution').value,
                'lateralFriction': self.get_parameter('dynamics.slide_box.lateralFriction').value,
                'spinningFriction': self.get_parameter('dynamics.slide_box.spinningFriction').value,
                'rollingFriction': self.get_parameter('dynamics.slide_box.rollingFriction').value,
                'contactDamping': self.get_parameter('dynamics.slide_box.contactDamping').value,
                'contactStiffness': self.get_parameter('dynamics.slide_box.contactStiffness').value,
            }
        }

        self.structure_config = {
            'box': {
                'mass': self.get_parameter('structure.box.mass').value,
                'inertia': self.get_parameter('structure.box.inertia').value,
                'dimensions': self.get_parameter('structure.box.dimensions').value
            },
            'slide_box': {
                'mass': self.get_parameter('structure.slide_box.mass').value,
                'inertia': self.get_parameter('structure.slide_box.inertia').value,
                'dimensions': self.get_parameter('structure.slide_box.dimensions').value
            },
            'world_box': {
                'mass': self.get_parameter('structure.world_box.mass').value,
                'inertia': self.get_parameter('structure.world_box.inertia').value,
                'dimensions': self.get_parameter('structure.world_box.dimensions').value
            }
        }

        self.client = []
        self.robots = []

      

    # def load_config(self, filename):
    #     """Load configuration from a YAML file."""
    #     if not os.path.exists(filename):
    #        self.get_logger().error(f"Configuration file not found: {filename}")
    #        return{} 
        
    #     with open(filename, 'r') as file:
    #         config = yaml.safe_load(file)
    #         print(config)
    #         return config
        
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
        time_step = self.get_parameter('engineSettings.timeStep').value
        num_solver_iterations = self.get_parameter('engineSettings.numSolverIterations').value
        use_split_impulse = self.get_parameter('engineSettings.useSplitImpulse').value
        split_impulse_threshold = self.get_parameter('engineSettings.splitImpulsePenetrationThreshold').value
        enable_cone_friction = self.get_parameter('engineSettings.enableConeFriction').value
        overlapping_pairs = self.get_parameter('engineSettings.deterministicOverlappingPairs').value

        # Use these settings to configure the physics engine
        p.setGravity(*gravity, physicsClientId=client_id)
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
        """Create the robot based on the dynamics and structure settings."""
        # Retrieve and parse the configuration for the robot's structure and dynamics
        for link_name in self.structure_config.keys():
            # Extract the configuration for this particular link
            link_params = self.get_parameter(f'structure.{link_name}').get_value()
            link_dynamics = self.get_parameter(f'dynamics.{link_name}').get_value()

            # Create collision and visual shapes for the link
            collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[x / 2 for x in link_params['dimensions']])
            visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[x / 2 for x in link_params['dimensions']])
            
            # Create the multibody for the link
            body_id = p.createMultiBody(baseMass=link_params['mass'],
                                        baseCollisionShapeIndex=collision_shape_id,
                                        baseVisualShapeIndex=visual_shape_id,
                                        basePosition=link_params.get('position', [0, 0, 0]),  
                                        baseInertialFramePosition=link_params.get('inertia_position', [0, 0, 0]),
                                        physicsClientId=client_id)
            
            # Set the dynamics properties for the link
            p.changeDynamics(body_id, -1,
                            restitution=link_dynamics['restitution'],
                            lateralFriction=link_dynamics['lateralFriction'],
                            spinningFriction=link_dynamics['spinningFriction'],
                            rollingFriction=link_dynamics['rollingFriction'],
                            contactDamping=link_dynamics['contactDamping'],
                            contactStiffness=link_dynamics['contactStiffness'],
                            physicsClientId=client_id)
            
            # Store the body ID for later use (e.g., control or simulation)
            self.robots.append(body_id)

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
    simulation_node.run_simulation(simulation_node.client[0], duration=50)
    simulation_node.disconnect()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
