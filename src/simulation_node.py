# This script will basically act as a intermediary between ROS2 and pybullet 
#client  and also will subscribe to the data published by the control node 
#!/usr/bin/env python3
import rclpy
import yaml
import pybullet as p
import pybullet_data
import time
from rclpy.node import Node

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self.engine_config = self.load_config("/home/akshay/virtual_shake_robot_pybullet/config/physics_engine_parameters.yaml")
        self.dynamics_config = self.load_config("/home/akshay/virtual_shake_robot_pybullet/config/pyhsics_parameters.yaml")
        self.client = []

    def load_config(self, filename):
        """Load configuration from a YAML file."""
        with open(filename, 'r') as file:
            return yaml.safe_load(file)
        
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

    def subscribing_control_topic(self):
        """this will subscribe to the control node to send the commands to the robot in simualtion"""
        pass
        
    def run_simulation(self, client, duration=5):
        """run the simulation for a specfic number of time steps"""
        start_time = time.time()
        while (time.time()- start_time) < duration:
            p.stepSimulation(client)
            time.sleep(1./240.) # Simulation time step       

        pass

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




