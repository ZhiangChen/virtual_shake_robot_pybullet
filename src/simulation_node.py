#!/usr/bin/env python3
import rclpy
import pybullet as p
import pybullet_data
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from virtual_shake_robot_pybullet.action import TrajectoryAction

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self.logger = self.get_logger()
        self.get_logger().info("Creating action server...")
        self._action_server = ActionServer(
            self,
            TrajectoryAction,
            'trajectory_action',
            execute_callback=self.execute_trajectory_callback
        )
        self.client_id = None
        self.vsr_id = None
        self.get_logger().info("Action server up and running!")

        # Declaring parameters
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
        # Retrieve parameters from the parameter server

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
        """Establish a connection to the PyBullet GUI"""
        self.client_id = p.connect(p.GUI)  # Will return a client ID
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.client.append(self.client_id)  # Adding in the list
        return self.client_id

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

    def create_robot(self, client_id):
        self.get_logger().info("Loading plane URDF...")
        p.loadURDF("plane.urdf", physicsClientId=client_id)

        self.get_logger().info("Creating world box shape...")
        world_box_shape = {
            'collision': p.createCollisionShape(p.GEOM_BOX, halfExtents=[x / 2 for x in self.structure_config['world_box']['dimensions']]),
            'visual': p.createVisualShape(p.GEOM_BOX, halfExtents=[x / 2 for x in self.structure_config['world_box']['dimensions']])
        }
        world_box_id = p.createMultiBody(baseMass=self.structure_config['world_box']['mass'],
                                        baseCollisionShapeIndex=world_box_shape['collision'],
                                        baseVisualShapeIndex=world_box_shape['visual'],
                                        basePosition=[0, 0, self.structure_config['world_box']['dimensions'][2] / 2],
                                        physicsClientId=client_id)
        self.get_logger().info(f"World box created with ID: {world_box_id}")

        try:
            pedestal_mesh_path = self.get_parameter('structure.pedestal.mesh').value
            mesh_scale = self.get_parameter('structure.pedestal.meshScale').value
            pedestal_shape = {
                'collision': p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=pedestal_mesh_path, meshScale=mesh_scale),
                'visual': p.createVisualShape(shapeType=p.GEOM_MESH, fileName=pedestal_mesh_path, meshScale=mesh_scale)
            }
        except rclpy.exceptions.ParameterUninitializedException:
            pedestal_shape = {
                'collision': p.createCollisionShape(p.GEOM_BOX, halfExtents=[d / 2 for d in self.structure_config['pedestal']['dimensions']]),
                'visual': p.createVisualShape(p.GEOM_BOX, halfExtents=[d / 2 for d in self.structure_config['pedestal']['dimensions']], rgbaColor=[1, 0, 0, 1])
            }

        self.get_logger().info("Creating and configuring pedestal with prismatic joint...")
        link_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[d / 2 for d in self.structure_config['pedestal']['dimensions']], rgbaColor=[1, 0, 0, 1])
        link_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[d / 2 for d in self.structure_config['pedestal']['dimensions']])
        link_mass = self.structure_config['pedestal']['mass']

        vsr_id = p.createMultiBody(
            baseMass=self.structure_config['world_box']['mass'],
            baseCollisionShapeIndex=world_box_shape['collision'],
            baseVisualShapeIndex=world_box_shape['visual'],
            basePosition=[0, 0, self.structure_config['world_box']['dimensions'][2] / 2],
            linkMasses=[link_mass],
            linkCollisionShapeIndices=[link_collision_shape],
            linkVisualShapeIndices=[link_visual_shape],
            linkPositions=[[0, 0, self.structure_config['world_box']['dimensions'][2] / 2 + self.structure_config['pedestal']['dimensions'][2] / 2]],
            linkOrientations=[[0, 0, 0, 1]],
            linkInertialFramePositions=[[0, 0, 0]],
            linkInertialFrameOrientations=[[0, 0, 0, 1]],
            linkParentIndices=[0],
            linkJointTypes=[p.JOINT_PRISMATIC],
            linkJointAxis=[[1, 0, 0]], 
            physicsClientId=client_id
        )
        self.get_logger().info(f"Pedestal with prismatic joint created with ID: {vsr_id}")

        
        p.changeDynamics(
            vsr_id, 1,
            restitution=self.dynamics_config['pedestal']['restitution'],
            lateralFriction=self.dynamics_config['pedestal']['lateralFriction'],
            spinningFriction=self.dynamics_config['pedestal']['spinningFriction'],
            rollingFriction=self.dynamics_config['pedestal']['rollingFriction'],
            contactDamping=self.dynamics_config['pedestal']['contactDamping'],
            contactStiffness=self.dynamics_config['pedestal']['contactStiffness'],
            physicsClientId=client_id
        )

      

        self.vsr_id = vsr_id

        # Capture the initial link state
        self.initial_link_state = p.getLinkState(self.vsr_id, 0, physicsClientId=self.client_id)
        self.get_logger().info(f"Initial link state: {self.initial_link_state}")


    def execute_trajectory_callback(self, goal_handle):
        '''Execute the trajectory from the data received'''
        self.get_logger().info("Executing trajectory callback...")

        positions = goal_handle.request.position_list
        velocities = goal_handle.request.velocity_list
        timestamps = goal_handle.request.timestamp_list

        
        if not (len(positions) == len(velocities) == len(timestamps)):
            self.get_logger().error("Length of positions, velocities, and timestamps must be equal.")
            goal_handle.abort()
            return TrajectoryAction.Result(success=False)

        # Initial delay before the first control command
        time.sleep(timestamps[0])

         # Loop through each point in the trajectory
        for i in range(1, len(timestamps)):
            delta_t = timestamps[i] - timestamps[i - 1]
            pre_control_joint_state = p.getJointState(self.vsr_id, 0, physicsClientId=self.client_id)
            self.get_logger().info(f"Pre-control joint state: {pre_control_joint_state}")

          
            p.setJointMotorControl2(
                bodyUniqueId=self.vsr_id,
                jointIndex=0,  
                controlMode=p.POSITION_CONTROL,
                targetPosition=positions[i],
                targetVelocity=velocities[i],
                force=10000,  
                physicsClientId=self.client_id
            )

            # Step the simulation
            p.stepSimulation(physicsClientId=self.client_id)

            # Wait for delta_t seconds before proceeding to the next step
            time.sleep(delta_t)

            # Get joint state after control
            post_control_joint_state = p.getJointState(self.vsr_id, 0, physicsClientId=self.client_id)
            self.get_logger().info(f"Post-control joint state: {post_control_joint_state}")

            # Send feedback
            feedback_msg = TrajectoryAction.Feedback()
            feedback_msg.current_position = post_control_joint_state[0]
            feedback_msg.current_velocity = post_control_joint_state[1]
            goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info("Trajectory execution completed successfully.")
        goal_handle.succeed()  
        
        
        final_link_state = p.getLinkState(self.vsr_id, 0, physicsClientId=self.client_id)
        self.get_logger().info(f"Final link state: {final_link_state}")

        
       
        return TrajectoryAction.Result(success=True)

   

    def disconnect(self):
        """Disconnect all the clients"""
        for client in self.client:
            p.disconnect(client)

def main(args=None):
    rclpy.init(args=args)
    simulation_node = SimulationNode()
    simulation_node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    simulation_node.server_connection()  # Connect GUI for visualization
    simulation_node.create_servers(2)
    simulation_node.setup_simulation(simulation_node.client[0])
    simulation_node.create_robot(simulation_node.client[0])

    rclpy.spin(simulation_node)
    rclpy.shutdown()

    simulation_node.disconnect()

if __name__ == '__main__':
    main()
