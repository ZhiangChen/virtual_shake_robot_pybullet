#!/usr/bin/env python3
import rclpy
import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt
from rclpy.node import Node
from rclpy.action import ActionServer
from virtual_shake_robot_pybullet.action import TrajectoryAction, LoadPBR
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3

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
        self._load_pbr_action_server = ActionServer(
            self,
            LoadPBR,
            'load_pbr_action',
            execute_callback=self.load_pbr_callback

        )
        
        self.get_logger().info("Action server up and running!")

        # Publishers for postion and velocity data in the  simuulation thread
        self.postion_publisher = self.create_publisher(Float64, 'pedestal_postion_publisher', 10)
        self.velocity_publisher = self.create_publisher(Float64, 'pedestal_velocity_publisher', 10)

        # #desired state publisher

        self.desired_position_publisher = self.create_publisher(Float64, 'desired_position_topic', 10)
        self.desired_velocity_publisher = self.create_publisher(Float64, 'desired_velocity_topic', 10)


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
                ('structure.world_box.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure.mass', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure.inertia', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure.restitution', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure.lateralFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure.spinningFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure.rollingFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure.contactDamping', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure.contactStiffness', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure.mesh', rclpy.Parameter.Type.STRING),
                ('rock_structure.meshScale', rclpy.Parameter.Type.DOUBLE_ARRAY),

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

        # Retrieve rock structure parameters
        self.rock_structure_config = {
            'dimensions': self.get_parameter('rock_structure.dimensions').value,
            'mass': self.get_parameter('rock_structure.mass').value,
            'inertia': self.get_parameter('rock_structure.inertia').value,
            'restitution': self.get_parameter('rock_structure.restitution').value,
            'lateralFriction': self.get_parameter('rock_structure.lateralFriction').value,
            'spinningFriction': self.get_parameter('rock_structure.spinningFriction').value,
            'rollingFriction': self.get_parameter('rock_structure.rollingFriction').value,
            'contactDamping': self.get_parameter('rock_structure.contactDamping').value,
            'contactStiffness': self.get_parameter('rock_structure.contactStiffness').value,
            'mesh': self.get_parameter('rock_structure.mesh').value,
            'meshScale': self.get_parameter('rock_structure.meshScale').value,

        }

        self.client = []

        ## To store the values for the plot

        self.desired_positions = []
        self.desired_velocities = []
        self.actual_positions = []
        self.actual_velocities = []
        self.timestamps = []


        self.rock_id = None

    def server_connection(self):
        """Establish a connection to the PyBullet GUI"""
        client_id = p.connect(p.GUI)  # Will return a client ID
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.client.append(client_id)  # Adding in the list
        self.get_logger().info(f"The Client_id is : {client_id}")
        return client_id

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


        ##verfiy the physics parameters
        physics_params = p.getPhysicsEngineParameters(physicsClientId = client_id)
        self.get_logger().info(f"physics Parameters : {physics_params}")

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

        # Set dynamics parameters for the world box
        p.changeDynamics(
            world_box_id, -1,
            localInertiaDiagonal=self.structure_config['world_box']['inertia'],
            restitution=self.dynamics_config['world_box']['restitution'],
            lateralFriction=self.dynamics_config['world_box']['lateralFriction'],
            spinningFriction=self.dynamics_config['world_box']['spinningFriction'],
            rollingFriction=self.dynamics_config['world_box']['rollingFriction'],
            contactDamping=self.dynamics_config['world_box']['contactDamping'],
            contactStiffness=self.dynamics_config['world_box']['contactStiffness'],
            physicsClientId=client_id
        )

        try:
            pedestal_mesh_path = self.get_parameter('structure.pedestal.mesh').value
            mesh_scale = self.get_parameter('structure.pedestal.meshScale').value
            if pedestal_mesh_path:
                self.get_logger().info(f"Loading pedestal mesh from {pedestal_mesh_path}...")
                pedestal_shape = {
                    'collision': p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=pedestal_mesh_path, meshScale=mesh_scale),
                    'visual': p.createVisualShape(shapeType=p.GEOM_MESH, fileName=pedestal_mesh_path, meshScale=mesh_scale)
                }
            else:
                raise rclpy.exceptions.ParameterUninitializedException()
        except rclpy.exceptions.ParameterUninitializedException:
            self.get_logger().info("No mesh file provided, creating pedestal as a box...")
            pedestal_shape = {
                'collision': p.createCollisionShape(p.GEOM_BOX, halfExtents=[d / 2 for d in self.structure_config['pedestal']['dimensions']]),
                'visual': p.createVisualShape(p.GEOM_BOX, halfExtents=[d / 2 for d in self.structure_config['pedestal']['dimensions']], rgbaColor=[1, 0, 0, 1])
            }

        self.get_logger().info("Creating and configuring pedestal with prismatic joint...")
        link_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[d / 2 for d in self.structure_config['pedestal']['dimensions']], rgbaColor=[1, 0, 0, 1])
        link_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[d / 2 for d in self.structure_config['pedestal']['dimensions']])
        link_mass = self.structure_config['pedestal']['mass']

        robot_id = p.createMultiBody(
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
        self.get_logger().info(f"Pedestal with prismatic joint created with ID: {robot_id}")

        p.changeDynamics(
            robot_id, 1,
            localInertiaDiagonal=self.structure_config['pedestal']['inertia'],
            restitution=self.dynamics_config['pedestal']['restitution'],
            lateralFriction=self.dynamics_config['pedestal']['lateralFriction'],
            spinningFriction=self.dynamics_config['pedestal']['spinningFriction'],
            rollingFriction=self.dynamics_config['pedestal']['rollingFriction'],
            contactDamping=self.dynamics_config['pedestal']['contactDamping'],
            contactStiffness=self.dynamics_config['pedestal']['contactStiffness'],
            physicsClientId=client_id
        )

        # Ensure collision is enabled
        p.setCollisionFilterGroupMask(robot_id, -1, collisionFilterGroup=1, collisionFilterMask=1)
        self.get_logger().info(f"Collision filter set for pedestal with ID: {robot_id}")


        # Enable collision rendering for debugging
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=client_id)


        # Get and log the initial position of the pedestal
        initial_position_state, initial_orientation_state = p.getBasePositionAndOrientation(robot_id, physicsClientId=client_id)
        self.get_logger().info(f"Initial position of the pedestal: {initial_position_state}, Initial orientation: {initial_orientation_state}")



    def load_pbr_callback(self, goal_handle):
        '''Load the Pbr callback function'''

        self.logger.info("Loading PBR structure...")
        structure_name = goal_handle.request.structure_name
        structure_type = goal_handle.request.structure_type
        feedback_msg = LoadPBR.Feedback()

        if structure_name != "rock":
            self.logger.error(f"Unknown structure: {structure_name}")
            goal_handle.abort()
            return LoadPBR.Result(success=False)

        try:
            if structure_type == 'mesh':
                rock_mesh_path = self.rock_structure_config['mesh']
                rock_mesh_scale = self.rock_structure_config['meshScale']
                rock_mass = self.rock_structure_config['mass']
                rock_inertia = self.rock_structure_config['inertia']
                rock_restitution = self.rock_structure_config['restitution']
                rock_lateralFriction = self.rock_structure_config['lateralFriction']
                rock_spinningFriction = self.rock_structure_config['spinningFriction']
                rock_rollingFriction = self.rock_structure_config['rollingFriction']
                rock_contactDamping = self.rock_structure_config['contactDamping']
                rock_contactStiffness = self.rock_structure_config['contactStiffness']

                self.get_logger().info(f"Loading rock mesh from {rock_mesh_path}...")
                collision_shape = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=rock_mesh_path, meshScale=rock_mesh_scale)
                visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=rock_mesh_path, meshScale=rock_mesh_scale, rgbaColor=[0.5, 0.5, 0.5, 1])
                rock_height = rock_mesh_scale[2]  

            elif structure_type == 'box':
                rock_dimensions = self.rock_structure_config['dimensions']
                rock_mass = self.rock_structure_config['mass']
                rock_inertia = self.rock_structure_config['inertia']
                rock_restitution = self.rock_structure_config['restitution']
                rock_lateralFriction = self.rock_structure_config['lateralFriction']
                rock_spinningFriction = self.rock_structure_config['spinningFriction']
                rock_rollingFriction = self.rock_structure_config['rollingFriction']
                rock_contactDamping = self.rock_structure_config['contactDamping']
                rock_contactStiffness = self.rock_structure_config['contactStiffness']

                self.get_logger().info("Loading rock as a box...")
                collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[d / 2 for d in rock_dimensions])
                visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[d / 2 for d in rock_dimensions], rgbaColor=[0.5, 0.5, 0.5, 1])
                rock_height = rock_dimensions[2]

            else:
                self.get_logger().error("Unknown rock structure type.")
                goal_handle.abort()
                return LoadPBR.Result(success=False)

            # # Common position logic
            # world_box_height = self.structure_config['world_box']['dimensions'][2]
            # pedestal_height = self.structure_config['pedestal']['dimensions'][2]

            rock_position = [0, 0, 2.2]
            self.get_logger().info(f"Placing rock at position: {rock_position}")

            rock_id = p.createMultiBody(
                baseMass=rock_mass, 
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=rock_position,
                physicsClientId=self.client[0]
            )

            self.get_logger().info(f"Rock structure loaded with ID: {rock_id}")
            self.rock_id = rock_id

            p.changeDynamics(
                rock_id, -1,
                localInertiaDiagonal=rock_inertia,
                restitution=rock_restitution,
                lateralFriction=rock_lateralFriction,
                spinningFriction=rock_spinningFriction,
                rollingFriction=rock_rollingFriction,
                contactDamping=rock_contactDamping,
                contactStiffness=rock_contactStiffness,
                physicsClientId=self.client[0]
            )

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


    def execute_trajectory_callback(self, goal_handle):
        '''Execute the trajectory from the data received'''
        self.logger.info("Executing trajectory callback...")

        client_id = goal_handle.request.client_id
        robot_id = goal_handle.request.robot_id
        positions = goal_handle.request.position_list
        velocities = goal_handle.request.velocity_list
        timestamps = goal_handle.request.timestamp_list

        feedback_msg = TrajectoryAction.Feedback()

        if not (len(positions) == len(velocities) == len(timestamps)):
            self.logger.error("Length of positions, velocities, and timestamps must be equal.")
            goal_handle.abort()
            return TrajectoryAction.Result(success=False)
        
        self.get_logger().info(f"Length of the trajectory: {len(positions)}")
        time_step = self.engine_settings['timestep']

        last_time = time.time()

        self.actual_positions = []
        self.actual_velocities = []

        for i in range(len(timestamps)):
            self.desired_positions.append(positions[i])
            self.desired_velocities.append(velocities[i])
            self.timestamps.append(timestamps[i])

            self.desired_position_publisher.publish(Float64(data=positions[i]))
            self.desired_velocity_publisher.publish(Float64(data=velocities[i]))

            # Simulate the joint control
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=0,
                controlMode=p.POSITION_CONTROL,
                targetPosition=positions[i],
                targetVelocity=velocities[i],
                force= 5 * 10**8,
                maxVelocity=200,
                physicsClientId=client_id
            )

            p.stepSimulation(physicsClientId=client_id)

            # Capture the actual joint state
            joint_state = p.getJointState(robot_id, 0, physicsClientId=client_id)
            actual_position, actual_velocity = joint_state[0], joint_state[1]


            

            self.actual_positions.append(actual_position)
            self.actual_velocities.append(actual_velocity)

            # Publish the actual position and velocity
            self.postion_publisher.publish(Float64(data=actual_position))
            self.velocity_publisher.publish(Float64(data=actual_velocity))



             
            #Feedback

            feedback_msg.current_position = actual_position
            feedback_msg.current_velocity = actual_velocity
            goal_handle.publish_feedback(feedback_msg)
          
            current_time = time.time()
            delta_t = current_time - last_time
            if time_step > delta_t:
                time.sleep(time_step - delta_t)
            last_time = current_time



        # Check the joint information again to ensure max velocity is set
        joint_info = p.getJointInfo(robot_id, 0, physicsClientId=client_id)
        self.get_logger().info(f"Updated joint information after trajectory execution: {joint_info}")

        max_velocity = joint_info[11]  
        self.get_logger().info(f"Updated maximum velocity of the joint after trajectory execution: {max_velocity}")

        self.get_logger().info(f"Final actual positions: {self.actual_positions}")
        self.get_logger().info(f"Final actual velocities: {self.actual_velocities}")

        self.get_logger().info("Trajectory execution completed successfully.")
        goal_handle.succeed()
        result = TrajectoryAction.Result()
        result.success = True
        result.actual_positions = self.actual_positions
        result.actual_velocities = self.actual_velocities
        return result


    def disconnect(self):
        """Disconnect all the clients"""
        for client in self.client:
            p.disconnect(client)

def main(args=None):
    rclpy.init(args=args)
    simulation_node = SimulationNode()
    simulation_node.server_connection()  # Connect GUI for visualization
    simulation_node.create_servers(1)
    simulation_node.setup_simulation(simulation_node.client[0])
    simulation_node.create_robot(simulation_node.client[0])
    rclpy.spin(simulation_node)
    rclpy.shutdown()
    simulation_node.disconnect()


if __name__ == '__main__':
    main()
