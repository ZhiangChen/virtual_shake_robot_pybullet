#!/usr/bin/env python3
import rclpy
import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from rclpy.action import ActionServer
from virtual_shake_robot_pybullet.action import TrajectoryAction, LoadPBR
from std_msgs.msg import Float64
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from rclpy.service import Service
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from virtual_shake_robot_pybullet.srv import ManageModel

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

        # Publishers for position and velocity data in the simulation thread
        self.position_publisher = self.create_publisher(Float64, 'pedestal_position_publisher', 10)
        self.velocity_publisher = self.create_publisher(Float64, 'pedestal_velocity_publisher', 10)

        # Desired state publisher
        self.desired_position_publisher = self.create_publisher(Float64, 'desired_position_topic', 10)
        self.desired_velocity_publisher = self.create_publisher(Float64, 'desired_velocity_topic', 10)

        # PBR rock state publisher
        self.pbr_pose_publisher = self.create_publisher(PoseStamped, 'pbr_pose_topic', 10)


        self._manage_model_service = self.create_service(ManageModel, 'manage_model', self.manage_model_callback)
        



        # Declaring parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ('simulation_node.engineSettings.model_wait_time', 2.0),
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
                ('structure.pedestal.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.pedestal.mesh', rclpy.Parameter.Type.STRING),
                ('structure.pedestal.meshScale', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.world_box.mass', rclpy.Parameter.Type.DOUBLE),
                ('structure.world_box.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure_box.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure_box.mass', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.restitution', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.lateralFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.localInertiaDiagonal', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure_box.spinningFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.rollingFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.contactDamping', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.contactStiffness', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.mesh', rclpy.Parameter.Type.STRING),
                ('rock_structure_mesh.meshScale', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure_mesh.mass', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.restitution', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.lateralFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.spinningFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.rollingFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.contactDamping', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.contactStiffness', rclpy.Parameter.Type.DOUBLE),
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
                'dimensions': self.get_parameter('structure.pedestal.dimensions').value,
            },
            'world_box': {
                'mass': self.get_parameter('structure.world_box.mass').value,
                'dimensions': self.get_parameter('structure.world_box.dimensions').value
            }
        }

        # Retrieve rock structure parameters for both box and mesh
        self.rock_structure_box_config = {
            'dimensions': self.get_parameter('rock_structure_box.dimensions').value,
            'mass': self.get_parameter('rock_structure_box.mass').value,
            'restitution': self.get_parameter('rock_structure_box.restitution').value,
            'lateralFriction': self.get_parameter('rock_structure_box.lateralFriction').value,
            'spinningFriction': self.get_parameter('rock_structure_box.spinningFriction').value,
            'rollingFriction': self.get_parameter('rock_structure_box.rollingFriction').value,
            'contactDamping': self.get_parameter('rock_structure_box.contactDamping').value,
            'contactStiffness': self.get_parameter('rock_structure_box.contactStiffness').value,
            'localInertiaDiagonal': self.get_parameter('rock_structure_box.localInertiaDiagonal').value
        }

        self.rock_structure_mesh_config = {
            'mesh': self.get_parameter('rock_structure_mesh.mesh').value,
            'meshScale': self.get_parameter('rock_structure_mesh.meshScale').value,
            'mass': self.get_parameter('rock_structure_mesh.mass').value,
            'restitution': self.get_parameter('rock_structure_mesh.restitution').value,
            'lateralFriction': self.get_parameter('rock_structure_mesh.lateralFriction').value,
            'spinningFriction': self.get_parameter('rock_structure_mesh.spinningFriction').value,
            'rollingFriction': self.get_parameter('rock_structure_mesh.rollingFriction').value,
            'contactDamping': self.get_parameter('rock_structure_mesh.contactDamping').value,
            'contactStiffness': self.get_parameter('rock_structure_mesh.contactStiffness').value
        }

        self.client_id = self.server_connection()

        ## To store the values for the plot
        self.desired_positions = []
        self.desired_velocities = []
        self.actual_positions = []
        self.actual_velocities = []
        self.timestamps = []

        self.rock_id = None

        self.setup_simulation()
        self.create_robot()

    def server_connection(self):
        """Establish a connection to the PyBullet GUI"""
        client_id = p.connect(p.GUI)  # Will return a client ID
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.get_logger().info(f"The Client_id is : {client_id}")
        return client_id

    def setup_simulation(self):
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
        p.setGravity(*gravity, physicsClientId=self.client_id)
        print(f"Gravity set for the client {self.client_id}:{gravity}")
        p.setTimeStep(time_step, physicsClientId=self.client_id)
        p.setPhysicsEngineParameter(numSolverIterations=num_solver_iterations,
                                    useSplitImpulse=use_split_impulse,
                                    splitImpulsePenetrationThreshold=split_impulse_threshold,
                                    enableConeFriction=enable_cone_friction,
                                    deterministicOverlappingPairs=overlapping_pairs,
                                    physicsClientId=self.client_id)        

        ##verfiy the physics parameters
        physics_params = p.getPhysicsEngineParameters(physicsClientId=self.client_id)
        self.get_logger().info(f"physics Parameters : {physics_params}")

    def manage_model_callback(self, request, response):
        self.get_logger().info(f"manage_model_callback called with action: {request.action}")
        model_wait_time = self.get_parameter('simulation_node.engineSettings.model_wait_time').value
        time_step = self.engine_settings['timestep']

        if request.action == "spawn":
            rock_urdf_path = "/home/akshay/ros2_ws/virtual_shake_robot_pybullet/models/double_rock_pbr/pbr_mesh.urdf"
            rock_position = [0, 0, 3.3]  
            model_id = p.loadURDF(rock_urdf_path, basePosition=rock_position, physicsClientId=self.client_id)

            if model_id < 0:
                response.success = False
                response.message = "Failed to load the rock model."
            else:
                self.rock_id = model_id
                response.success = True
                response.message = f"Rock model loaded with ID: {self.rock_id}"
                self.get_logger().info(response.message)  # Log the success message

                # Wait after spawning the model
                self.get_logger().info(f'Waiting for {model_wait_time} seconds after model spawn.')
                end_wait_time = time.time() + model_wait_time
                while time.time() < end_wait_time:
                    # Set joint control to keep position and velocity at zero
                    p.setJointMotorControl2(
                        bodyUniqueId=self.robot_id,
                        jointIndex=0,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=0,
                        targetVelocity=0,
                        force=5 * 10**8,
                        maxVelocity=200,
                        physicsClientId=self.client_id
                    )
                    p.stepSimulation(physicsClientId=self.client_id)
                    time.sleep(time_step)

        elif request.action == "delete":
            if self.rock_id is not None:
                p.removeBody(self.rock_id, physicsClientId=self.client_id)
                self.get_logger().info(f"Rock model with ID {self.rock_id} deleted successfully.")  # Log deletion
                self.rock_id = None
                response.success = True
                response.message = "Rock model deleted successfully."
            else:
                response.success = False
                response.message = "No rock model to delete."
                self.get_logger().info(response.message)  # Log the failure message
        else:
            response.success = False
            response.message = "Unknown action."
            self.get_logger().info(response.message)  # Log the unknown action

        return response




    def create_robot(self):
        self.get_logger().info("Loading plane URDF...")
        plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        self.get_logger().info(f"Plane loaded with ID: {plane_id}")

        # Define the base of the shake table
        base_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[25, 7, 1])
        base_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[25, 7, 1], rgbaColor=[1, 1, 0, 1])  # Yellow color

        # Define the pedestal of the shake table
        pedestal_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[5, 5, 0.25])
        pedestal_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[5, 5, 0.25], rgbaColor=[1, 0, 0, 1])  # Red color

        # Create the base and pedestal as a single multi-body
        base_position = [0, 0, 1.0]  # Base is positioned at z = 1.0
        base_orientation = p.getQuaternionFromEuler([0, 0, 0])

        link_masses = [self.structure_config['pedestal']['mass']]  # Use the mass from parameters
        link_collision_shapes = [pedestal_collision_shape]
        link_visual_shapes = [pedestal_visual_shape]
        link_positions = [[0, 0, 1.25]]  
        link_orientations = [p.getQuaternionFromEuler([0, 0, 0])]
        link_inertial_frame_positions = [[0, 0, 0]]
        link_inertial_frame_orientations = [p.getQuaternionFromEuler([0, 0, 0])]
        indices = [0]
        joint_types = [p.JOINT_PRISMATIC]
        joint_axis = [[1, 0, 0]]  

        base = p.createMultiBody(
            baseMass=self.structure_config['world_box']['mass'],  # Use the mass from parameters
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
            linkJointAxis=joint_axis,
            physicsClientId=self.client_id
        )

        if base < 0:
            self.get_logger().error("Failed to create the combined multi-body.")
            return

        self.robot_id = base
        self.get_logger().info(f"Combined world box and pedestal created with ID: {self.robot_id}")

        # Enable collision for the base
        p.setCollisionFilterGroupMask(self.robot_id, -1, collisionFilterGroup=1, collisionFilterMask=1)
        self.get_logger().info(f"Collision filter set for base link of robot ID: {self.robot_id}")

        # Enable collision for the pedestal
        p.setCollisionFilterGroupMask(self.robot_id, 0, collisionFilterGroup=1, collisionFilterMask=1)
        self.get_logger().info(f"Collision filter set for pedestal link 0 of robot ID: {self.robot_id}")

        # Verify the number of joints
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        self.get_logger().info(f"Number of joints in the robot: {num_joints}")

        if num_joints == 0:
            self.get_logger().error("No joints found in the created robot.")
            return

        # Retrieve dynamics parameters
        world_box_dynamics = self.dynamics_config['world_box']
        pedestal_dynamics = self.dynamics_config['pedestal']

        # Set dynamics parameters for the base and pedestal
        p.changeDynamics(
            self.robot_id, -1,
            restitution=world_box_dynamics['restitution'],
            lateralFriction=world_box_dynamics['lateralFriction'],
            spinningFriction=world_box_dynamics['spinningFriction'],
            rollingFriction=world_box_dynamics['rollingFriction'],
            contactDamping=world_box_dynamics['contactDamping'],
            contactStiffness=world_box_dynamics['contactStiffness'],
            collisionMargin=0.01,
            localInertiaDiagonal=[0,0,0],
            physicsClientId=self.client_id
        )

        p.changeDynamics(
            self.robot_id, 0,  # Link index for the pedestal
            restitution=pedestal_dynamics['restitution'],
            lateralFriction=pedestal_dynamics['lateralFriction'],
            spinningFriction=pedestal_dynamics['spinningFriction'],
            rollingFriction=pedestal_dynamics['rollingFriction'],
            contactDamping=pedestal_dynamics['contactDamping'],
            contactStiffness=pedestal_dynamics['contactStiffness'],
            collisionMargin=0.01,
            localInertiaDiagonal=[1668333.33, 1668333.33, 3333333.33],
            physicsClientId=self.client_id
        )

        # Print joint info
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self.client_id)
            self.get_logger().info(f"Joint {joint_index} info: {joint_info}")

        # Print link state
        for link_index in range(num_joints):
            link_state = p.getLinkState(self.robot_id, link_index, physicsClientId=self.client_id)
            self.get_logger().info(f"Link {link_index} state: {link_state}")

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
                    baseOrientation=[0, 0, 0, 1],  # No rotation for base
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

            else:
                self.get_logger().error("Unknown rock structure type.")
                goal_handle.abort()
                return LoadPBR.Result(success=False)

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

    def execute_trajectory_callback(self, goal_handle):
        '''Execute the trajectory from the data received'''
        self.logger.info("Executing trajectory callback...")

        client_id = goal_handle.request.client_id
        robot_id = goal_handle.request.robot_id
        positions = goal_handle.request.position_list
        velocities = goal_handle.request.velocity_list
        timestamps = goal_handle.request.timestamp_list
        wait_time = goal_handle.request.wait_time

        feedback_msg = TrajectoryAction.Feedback()

        if not (len(positions) == len(velocities) == len(timestamps)):
            self.logger.error("Length of positions, velocities, and timestamps must be equal.")
            goal_handle.abort()
            return TrajectoryAction.Result(success=False)
        
        self.get_logger().info(f"Length of the trajectory: {len(positions)}")
        time_step = self.engine_settings['timestep']

        self.actual_positions = []
        self.actual_velocities = []

        loop_start_time = time.time()  # Record the start time of the loop

        for i in range(len(timestamps)):
            iteration_start_time = time.time()  # Record the start time of this iteration

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
                force=5 * 10**8,
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
            self.position_publisher.publish(Float64(data=actual_position))
            self.velocity_publisher.publish(Float64(data=actual_velocity))

            # Feedback
            feedback_msg.current_position = actual_position
            feedback_msg.current_velocity = actual_velocity
            goal_handle.publish_feedback(feedback_msg)
            
            # Ensure each loop iteration takes approximately time_step seconds
            iteration_end_time = time.time()
            elapsed_time = iteration_end_time - iteration_start_time
            sleep_time = time_step - elapsed_time

            if sleep_time > 0:
                time.sleep(sleep_time)

        loop_end_time = time.time()  # Record the end time of the loop
        total_execution_time = loop_end_time - loop_start_time  # Calculate the total execution time
        self.get_logger().info(f"Total execution time: {total_execution_time} seconds")

        # Wait for the specified wait time after the trajectory execution
        self.get_logger().info(f"Waiting for {wait_time} seconds after trajectory completion.")
        end_wait_time = time.time() + wait_time
        while time.time() < end_wait_time:
            p.stepSimulation(physicsClientId=client_id)
            time.sleep(time_step)

        # Get the position and orientation of the PBR
        if self.rock_id is not None:
            pbr_position, pbr_orientation = p.getBasePositionAndOrientation(self.rock_id, physicsClientId=client_id)

            # Create a PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"

            pose_msg.pose.position = Point(x=pbr_position[0], y=pbr_position[1], z=pbr_position[2])
            pose_msg.pose.orientation = Quaternion(x=pbr_orientation[0], y=pbr_orientation[1], z=pbr_orientation[2], w=pbr_orientation[3])

            # Publish the PoseStamped message
            self.pbr_pose_publisher.publish(pose_msg)
        else:
            self.get_logger().error("rock_id is None, cannot get base position and orientation.")
            goal_handle.abort()
            return TrajectoryAction.Result(success=False)

        

        # Check the joint information again to ensure max velocity is set
        joint_info = p.getJointInfo(robot_id, 0, physicsClientId=client_id)
        # self.get_logger().info(f"Updated joint information after trajectory execution: {joint_info}")

        max_velocity = joint_info[11]
        # self.get_logger().info(f"Updated maximum velocity of the joint after trajectory execution: {max_velocity}")

        # self.get_logger().info(f"Final actual positions: {self.actual_positions}")
        # self.get_logger().info(f"Final actual velocities: {self.actual_velocities}")

        self.get_logger().info("Trajectory execution completed successfully.")
        goal_handle.succeed()
        result = TrajectoryAction.Result()
        result.success = True
        result.actual_positions = self.actual_positions
        result.actual_velocities = self.actual_velocities
        return result


    def disconnect(self):
        """Disconnect all the clients"""
        p.disconnect(self.client_id)

def main(args=None):
    rclpy.init(args=args)
    simulation_node = SimulationNode()
    rclpy.spin(simulation_node)
    rclpy.shutdown()
    simulation_node.disconnect()

if __name__ == '__main__':
    main()
