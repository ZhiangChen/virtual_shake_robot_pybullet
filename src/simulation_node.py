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

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self.engine_config = self.load_config("/home/akshay/ros2_ws/virtual_shake_robot_pybullet/config/physics_engine_parameters.yaml")
        self.dynamics_config = self.load_config("/home/akshay/ros2_ws/virtual_shake_robot_pybullet/config/pyhsics_parameters.yaml")
        self.strucutre_config = self.load_config("/home/akshay/ros2_ws/virtual_shake_robot_pybullet/config/vsr_structure.yaml")
        self.client = []
        self.robots = []

    def load_config(self, filename):
        """Load configuration from a YAML file."""
        if not os.path.exists(filename):
           self.get_logger().error(f"Configuration file not found: {filename}")
           return{} 
        
        with open(filename, 'r') as file:
            config = yaml.safe_load(file)
            print(config)
            return config
        
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
        """Configure the simulation with parameters from the YAML files."""
        settings = self.engine_config['engineSettings']
        p.setGravity(*settings['gravity'], physicsClientId=client_id)
        p.setTimeStep(settings['timeStep'], physicsClientId=client_id)
        p.setPhysicsEngineParameter(numSolverIterations=settings['numSolverIterations'],
                                    useSplitImpulse=settings['useSplitImpulse'],
                                    splitImpulsePenetrationThreshold=settings['splitImpulsePenetrationThreshold'],
                                    enableConeFriction=settings['enableConeFriction'],
                                    deterministicOverlappingPairs=settings['deterministicOverlappingPairs'],
                                    physicsClientId=client_id)
        self.create_robot(client_id)

    def subscribing_control_topic(self):
        """this will subscribe to the control node to send the commands to the robot in simualtion"""
        pass


    def create_robot(self,client_id):
        '''VSR Structure using the MultiBody Create in Pybullet'''
        """Create the robot based on the dynamics and structure settings."""
        structure = self.strucutre_config['structure']
        dynamics = self.dynamics_config['dynamics']

        for link_name, link_params in structure.items():
            collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[x / 2 for x in link_params['dimensions']])
            visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[x / 2 for x in link_params['dimensions']])
            body_id = p.createMultiBody(baseMass=link_params['mass'],
                                        baseCollisionShapeIndex=collision_shape_id,
                                        baseVisualShapeIndex=visual_shape_id,
                                        basePosition=[0, 0, 0],  # You may want to adjust positions
                                        baseInertialFramePosition=[0, 0, 0])
            dyn_params = dynamics[link_name]
            p.changeDynamics(body_id, -1,
                             restitution=dyn_params['restitution'],
                             lateralFriction=dyn_params['lateralFriction'],
                             spinningFriction=dyn_params['spinningFriction'],
                             rollingFriction=dyn_params['rollingFriction'],
                             contactDamping=dyn_params['contactDamping'],
                             contactStiffness=dyn_params['contactStiffness'],
                             physicsClientId=client_id)
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
