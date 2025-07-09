#!/usr/bin/env python3
import objgraph
import gc
import os
import rclpy
import pybullet as p
import pybullet_data
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import random
import pandas as pd
from trimesh.transformations import quaternion_matrix, euler_from_matrix
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.action import ActionServer
from virtual_shake_robot_pybullet.action import TrajectoryAction, LoadPBR, LoadDispl,VelocityTrajectory
from std_msgs.msg import Float64
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from rclpy.service import Service
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from virtual_shake_robot_pybullet.srv import ManageModel
from memory_profiler import profile

class SimulationNode(Node):

    """
    A ROS2 node for simulating the Virtual Shake Robot (VSR) using PyBullet.

    The SimulationNode class is responsible for managing the simulation environment, executing trajectory actions, 
    loading and manipulating Precariously Balanced Rocks (PBR), and publishing simulation data. It includes:
    
    - Action servers for handling trajectory, PBR loading, and displacement actions.
    - Publishers for the position, velocity, and state of the pedestal and PBR in the simulation.
    - Parameters for configuring the physics engine, dynamics, and structure of the simulated objects.
    
    This class is the core component of the simulation system, integrating the PyBullet physics engine with ROS2.
    """
    def __init__(self):
        super().__init__('simulation_node')
        self.logger = self.get_logger()
        self.get_logger().info("Creating action server...")
        self._action_server = ActionServer(
            self,
            TrajectoryAction,
            'trajectory_action',
            execute_callback=self.execute_pos_vel_trajectory_callback
        )
        self._load_pbr_action_server = ActionServer(
            self,
            LoadPBR,
            'load_pbr_action',
            execute_callback=self.load_pbr_callback
        )
        self._load_displ_action_server = ActionServer(
            self,
            LoadDispl,
            'load_displ_action',
            execute_callback=self.execute_position_trajectory_callback
        )
        self._velocity_trajectory_action_server = ActionServer(
            self,
            VelocityTrajectory,
            'velocity_trajectory_action',
            execute_callback=self.execute_velocity_trajectory_callback  # NEW callback below
        )


        self.get_logger().info("Action server up and running!")
        self.experiment_times = []

        # Desired state publisher
        self.desired_position_publisher = self.create_publisher(Float64, 'desired_position_topic', 10)
        self.desired_velocity_publisher = self.create_publisher(Float64, 'desired_velocity_topic', 10)

        # Get the full namespace
        full_namespace = self.get_namespace()

        # Check if namespace contains valid sim_no
        if full_namespace and len(full_namespace.split('/')) > 1:
            sim_no = full_namespace.split('/')[1]
            if sim_no:  # Ensure sim_no is not empty
                self.get_logger().info(f"Using namespace: {full_namespace}, sim_no: {sim_no}")
                
                # Publishers with sim_no
                self.position_publisher = self.create_publisher(Float64, f'/{sim_no}/{sim_no}/pedestal_position_publisher', 10)
                self.velocity_publisher = self.create_publisher(Float64, f'/{sim_no}/{sim_no}/pedestal_velocity_publisher', 10)
                self.pbr_pose_publisher = self.create_publisher(PoseStamped, f'/{sim_no}/{sim_no}/pbr_pose_topic', 10)
            else:
                self.get_logger().info("sim_no is empty, running without namespace.")
                # Publishers without sim_no (default names)
                self.position_publisher = self.create_publisher(Float64, 'pedestal_position_publisher', 10)
                self.velocity_publisher = self.create_publisher(Float64, 'pedestal_velocity_publisher', 10)
                self.pbr_pose_publisher = self.create_publisher(PoseStamped, 'pbr_pose_topic', 10)
        else:
            self.get_logger().info("No sim_no mentioned, running without namespace.")
            
            # Publishers without sim_no (default names)
            self.position_publisher = self.create_publisher(Float64, 'pedestal_position_publisher', 10)
            self.velocity_publisher = self.create_publisher(Float64, 'pedestal_velocity_publisher', 10)
            self.pbr_pose_publisher = self.create_publisher(PoseStamped, 'pbr_pose_topic', 10)

        # Manage model service
        self._manage_model_service = self.create_service(ManageModel, 'manage_model', self.manage_model_callback)

        
        
        #Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[

                ('simSettings.realtime_flag', rclpy.Parameter.Type.BOOL),
                ('simSettings.use_gui', rclpy.Parameter.Type.BOOL),
                ('simSettings.enable_plotting', rclpy.Parameter.Type.BOOL), 
                ('engineSettings.loading_wait_time', rclpy.Parameter.Type.DOUBLE),
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
                ('structure.world_box.inertia', rclpy.Parameter.Type.DOUBLE_ARRAY), 
                ('structure.pedestal.inertia', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('structure.world_box.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure_box.dimensions', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure_box.mass', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.restitution', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.lateralFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.localInertiaDiagonal', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure_box.spinningFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.contactDamping', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.contactStiffness', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_box.rock_position', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure_mesh.mesh', rclpy.Parameter.Type.STRING),
                ('rock_structure_mesh.meshScale', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure_mesh.mass', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.restitution', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.lateralFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.spinningFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.rollingFriction', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.contactDamping', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.contactStiffness', rclpy.Parameter.Type.DOUBLE),
                ('rock_structure_mesh.rock_position', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('rock_structure_mesh.rock_orientation', rclpy.Parameter.Type.DOUBLE_ARRAY),
            
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
                'contactDamping': self.get_parameter('dynamics.world_box.contactDamping').value,
                'contactStiffness': self.get_parameter('dynamics.world_box.contactStiffness').value,
            },
            'pedestal': {
                'restitution': self.get_parameter('dynamics.pedestal.restitution').value,
                'lateralFriction': self.get_parameter('dynamics.pedestal.lateralFriction').value,
                'spinningFriction': self.get_parameter('dynamics.pedestal.spinningFriction').value,
                'contactDamping': self.get_parameter('dynamics.pedestal.contactDamping').value,
                'contactStiffness': self.get_parameter('dynamics.pedestal.contactStiffness').value,
            },
        }

        self.structure_config = {
            'pedestal': {
                'mass': self.get_parameter('structure.pedestal.mass').value,
                'dimensions': self.get_parameter('structure.pedestal.dimensions').value,
                'inertia': self.get_parameter('structure.pedestal.inertia').value
            },
            'world_box': {
                'mass': self.get_parameter('structure.world_box.mass').value,
                'dimensions': self.get_parameter('structure.world_box.dimensions').value,
                'inertia': self.get_parameter('structure.world_box.inertia').value
            }
        }

        # Retrieve rock structure parameters for both box and mesh
        self.rock_structure_box_config = {
            'dimensions': self.get_parameter('rock_structure_box.dimensions').value,
            'mass': self.get_parameter('rock_structure_box.mass').value,
            'restitution': self.get_parameter('rock_structure_box.restitution').value,
            'lateralFriction': self.get_parameter('rock_structure_box.lateralFriction').value,
            'spinningFriction': self.get_parameter('rock_structure_box.spinningFriction').value,
            'contactDamping': self.get_parameter('rock_structure_box.contactDamping').value,
            'contactStiffness': self.get_parameter('rock_structure_box.contactStiffness').value,
            'localInertiaDiagonal': self.get_parameter('rock_structure_box.localInertiaDiagonal').value,
            'rock_position' : self.get_parameter('rock_structure_box.rock_position').value
        }

        self.rock_structure_mesh_config = {
            'mesh': self.get_parameter('rock_structure_mesh.mesh').value,
            'meshScale': self.get_parameter('rock_structure_mesh.meshScale').value,
            'mass': self.get_parameter('rock_structure_mesh.mass').value,
            'restitution': self.get_parameter('rock_structure_mesh.restitution').value,
            'lateralFriction': self.get_parameter('rock_structure_mesh.lateralFriction').value,
            'spinningFriction': self.get_parameter('rock_structure_mesh.spinningFriction').value,
            'contactDamping': self.get_parameter('rock_structure_mesh.contactDamping').value,
            'contactStiffness': self.get_parameter('rock_structure_mesh.contactStiffness').value,
            'rock_position' : self.get_parameter('rock_structure_mesh.rock_position').value,
            'rock_orientation' : self.get_parameter('rock_structure_mesh.rock_orientation').value
        }

        self.realtime_flag = self.get_parameter('simSettings.realtime_flag').value
        self.use_gui = self.get_parameter('simSettings.use_gui').value
        self.enable_plotting = self.get_parameter('simSettings.enable_plotting').value

        self.client_id = self.server_connection(self.use_gui)

        self.ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))
        self.rock_id = None

        self.setup_simulation()
        self.create_robot()
        self.declare_parameter('recording_folder_name', 'default_folder')
        self.recordings_folder = self.get_recording_folder_name()
        self.determism_seed = 42
        random.seed(self.determism_seed)
        np.random.seed(self.determism_seed)


    def server_connection(self, use_gui):
        """
        Establishes a connection to the PyBullet server with an optional GUI.

        Arguments:
            use_gui (bool): If True, connects to PyBullet with GUI; otherwise, connects in DIRECT mode.

        Returns:
            client_id (int): The ID of the PyBullet client.
        """
        if use_gui:
            client_id = p.connect(p.GUI)
        else:
            client_id = p.connect(p.DIRECT)  # No GUI

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
                                    solverResidualThreshold=0,
                                    physicsClientId=self.client_id)        

        ##verfiy the physics parameters
        physics_params = p.getPhysicsEngineParameters(physicsClientId=self.client_id)
        self.get_logger().info(f"physics Parameters : {physics_params}")


    def manage_model_callback(self, request, response):
        """
        Callback function for the 'manage_model' service. Handles spawning and deleting the model in the simulation.

        Arguments:
            request (ManageModel.Request): Service request containing the action ('spawn' or 'delete').
            response (ManageModel.Response): Service response indicating success or failure of the action.

        Returns:
            response (ManageModel.Response): Updated service response after processing the request.
        """
        self.get_logger().info(f"manage_model_callback called with action: {request.action}")
        model_wait_time = self.get_parameter('engineSettings.loading_wait_time').value
        time_step = self.engine_settings['timestep']

        if request.action == "spawn":

            urdf_path = self.rock_structure_mesh_config['mesh']
            urdf_path = os.path.join(self.ros2_ws, 'src', 'virtual_shake_robot_pybullet', urdf_path)
            rock_position = self.rock_structure_mesh_config['rock_position']  
            rock_orientation = self.rock_structure_mesh_config['rock_orientation']
            

            model_id = p.loadURDF(urdf_path, basePosition=rock_position, physicsClientId=self.client_id)
            if model_id >= 0:
                # Reset to the desired orientation immediately after loading
                p.resetBasePositionAndOrientation(
                    model_id,
                    rock_position,
                    rock_orientation,
                    physicsClientId=self.client_id
                )


            if model_id < 0:
                response.success = False
                response.message = "Failed to load the rock model."
            else:
                self.rock_id = model_id

                response.success = True
                response.message = f"Rock model loaded with ID: {self.rock_id}"
                pos, actual_quat = p.getBasePositionAndOrientation(self.rock_id, physicsClientId=self.client_id)
                self.get_logger().info(f"The Actual quaternion is : {actual_quat}")
                roll, pitch, yaw = p.getEulerFromQuaternion(actual_quat)
                q_int = np.array(rock_orientation)
                q_act = np.array(actual_quat)
                dot   = float(np.dot(q_int, q_act))
                self.get_logger().info(f"Quaternion dot-product (±1 = exact match): {dot:.6f}")

                # 2) PyBullet’s default Euler (for reference)
                roll, pitch, yaw = p.getEulerFromQuaternion(actual_quat)
                self.get_logger().info(f"-> spawned PBR RPY from PyBullet (rad): {[roll, pitch, yaw]}")

                tf4 = quaternion_matrix(actual_quat)  # build 4×4 transformation matrix from quaternion [x, y, z, w]
                tf4 = quaternion_matrix(actual_quat)       # build 4×4 from [x,y,z,w]
                rpy_sxyz = euler_from_matrix(tf4, axes='sxyz')
                rpy_deg  = np.degrees(rpy_sxyz)
                self.get_logger().info(
                    f"-> spawned PBR RPY via Trimesh sxyz (deg): {rpy_deg.tolist()}"
                )
                self.get_logger().info(f"-> spawned PBR RPY (rad): {[roll, pitch, yaw]}")
                self.get_logger().info(f"-> spawned PBR Position: {pos}")
                self.get_logger().info(response.message)  # Log the success message

                restitution = self.rock_structure_mesh_config['restitution']
                lateralFriction = self.rock_structure_mesh_config['lateralFriction']
                spinningFriction = self.rock_structure_mesh_config['spinningFriction']
                contactDamping = self.rock_structure_mesh_config['contactDamping']
                contactStiffness = self.rock_structure_mesh_config['contactStiffness']

              
                p.changeDynamics(
                    model_id, -1,
                    restitution=restitution,
                    lateralFriction=lateralFriction,
                    spinningFriction=spinningFriction,
                    contactDamping=contactDamping,
                    contactStiffness=contactStiffness,
                    physicsClientId=self.client_id
                )

                self.intial_postion,self.intial_orientation = p.getBasePositionAndOrientation(self.rock_id, physicsClientId = self.client_id)

                # Retrieve PBR dynamics
                pbr_dynamics_info = p.getDynamicsInfo(self.rock_id, -1)
                restitution = pbr_dynamics_info[5]
                lateralFriction = pbr_dynamics_info[1]
                spinningFriction = pbr_dynamics_info[7]
                contactDamping = pbr_dynamics_info[8]
                contactStiffness = pbr_dynamics_info[9]

                self.get_logger().info(f"Initial rock dynamics: {pbr_dynamics_info}")
                p.changeDynamics(
                self.robot_id, 0,
                restitution=restitution,
                lateralFriction=lateralFriction,
                spinningFriction=spinningFriction,
                contactDamping=contactDamping,
                contactStiffness=contactStiffness,
                physicsClientId=self.client_id
            )
                pedestal_dynamics = p.getDynamicsInfo(self.robot_id, 0)
                self.get_logger().info(f"Updated pedestal dynamics: {pedestal_dynamics}")   

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

        elif request.action == "reset":
            if self.rock_id is not None:
                
                # Reset the position and orientation of the rock
                p.resetBasePositionAndOrientation(
                    self.rock_id,
                    self.intial_postion,
                    self.intial_orientation,
                    physicsClientId=self.client_id
                )
                self.get_logger().info(f"Rock model with ID {self.rock_id} has been reset")

                # Reset the base velocity of the pedestal (robot_id)
                p.resetBaseVelocity(
                    objectUniqueId=self.robot_id,
                    linearVelocity=[0, 0, 0],
                    angularVelocity=[0, 0, 0],
                    physicsClientId=self.client_id
                )
                self.get_logger().info(f"Pedestal with ID {self.robot_id} velocity has been reset")


                response.success = True
                response.message = f"Rock and pedestal have been reset"
            else:
                response.success = False
                response.message = "Rock ID is not set."
                self.get_logger().info(response.message)


           
        return response
    
    def create_robot(self):
        """Creates the robot model in the PyBullet simulation based on the configuration."""
        self.get_logger().info("Loading the VSR_shake_table...")
        plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        self.get_logger().info(f"Plane loaded with ID: {plane_id}")

        # Define the base of the shake table using parameters from vsr_structure.yaml
        world_box_dimensions = [dim / 2 for dim in self.structure_config['world_box']['dimensions']]  # Divide by 2 for halfExtents
        base_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=world_box_dimensions)
        base_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=world_box_dimensions, rgbaColor=[1, 1, 0, 1])  

        # Create the base as a multi-body
        base_position = [0, 0, 1.0]  # Base is positioned at z = 1.0
        base_orientation = p.getQuaternionFromEuler([0, 0, 0])

        # Calculate the height of the base
        base_height = world_box_dimensions[2]  

        # Check if we are using a mesh or a box for the pedestal
        if 'mesh' in self.structure_config['pedestal']:
            # Define the pedestal using a mesh
            pedestal_mesh_path = self.structure_config['pedestal']['mesh']
            pedestal_mesh_scale = self.structure_config['pedestal']['meshScale']
            pedestal_collision_shape = p.createCollisionShape(p.GEOM_MESH, fileName=pedestal_mesh_path, meshScale=pedestal_mesh_scale)
            pedestal_visual_shape = p.createVisualShape(p.GEOM_MESH, fileName=pedestal_mesh_path, meshScale=pedestal_mesh_scale, rgbaColor=[1, 0, 0, 1])

            pedestal_dimensions = [dim / 2 for dim in self.structure_config['pedestal']['dimensions']]
            pedestal_height = pedestal_dimensions[2]  
        else:
            # Define the pedestal using a box
            pedestal_dimensions = [dim / 2 for dim in self.structure_config['pedestal']['dimensions']]
            self.get_logger().info(f"The pedestal_dimensions are {pedestal_dimensions}")
            pedestal_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=pedestal_dimensions)
            pedestal_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=pedestal_dimensions, rgbaColor=[1, 0, 0, 1])  

            # Calculate the pedestal height
            pedestal_height = pedestal_dimensions[2] 

        # Calculate the Z position of the pedestal such that its bottom aligns with the top of the base
        pedestal_position_z = base_height + pedestal_height
        self.get_logger().info(f"The pedestal height is {pedestal_position_z}")

        # Create the pedestal as a link to the base
        link_masses = [self.structure_config['pedestal']['mass']]  
        link_collision_shapes = [pedestal_collision_shape]
        link_visual_shapes = [pedestal_visual_shape]
        link_positions = [[0, 0, pedestal_position_z]]  # Calculated Z position
        link_orientations = [p.getQuaternionFromEuler([0, 0, 0])]
        link_inertial_frame_positions = [[0, 0, 0]]
        link_inertial_frame_orientations = [p.getQuaternionFromEuler([0, 0, 0])]
        indices = [0]
        joint_types = [p.JOINT_PRISMATIC]  
        joint_axis = [[1, 0, 0]]  
        base = p.createMultiBody(
            baseMass=self.structure_config['world_box']['mass'],  
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

        p.changeDynamics(
        self.robot_id, 0,  
        restitution=self.dynamics_config['pedestal']['restitution'],
        lateralFriction=self.dynamics_config['pedestal']['lateralFriction'],
        spinningFriction=self.dynamics_config['pedestal']['spinningFriction'],
        contactDamping=self.dynamics_config['pedestal']['contactDamping'],
        contactStiffness=self.dynamics_config['pedestal']['contactStiffness'],
        physicsClientId=self.client_id
    )

        # Enable collision for the base
        p.setCollisionFilterGroupMask(self.robot_id, -1, collisionFilterGroup=1, collisionFilterMask=1)
        self.get_logger().info(f"Collision filter set for base link of robot ID: {self.robot_id}")

        # Enable collision for the pedestal
        p.setCollisionFilterGroupMask(self.robot_id, 0, collisionFilterGroup=1, collisionFilterMask=1)
        self.get_logger().info(f"Collision filter set for pedestal link 0 of robot ID: {self.robot_id}")

        # Verify the number of joints
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        self.get_logger().info(f"Number of joints in the robot: {num_joints}")

        pedestal_dynamics = p.getDynamicsInfo(self.robot_id, 0)
        self.get_logger().info(f"The pedestal Dynamics are : {pedestal_dynamics}")

        if num_joints == 0:
            self.get_logger().error("No joints found in the created robot.")
            return


    def load_pbr_callback(self, goal_handle):
        """
        Callback function for loading the PBR structure as part of the LoadPBR action.

        Arguments:
            goal_handle (LoadPBR.GoalHandle): The goal handle containing the structure name and type (mesh or box).

        Returns:
            result (LoadPBR.Result): The result of the action, indicating success or failure.
        """

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


                urdf_path = self.rock_structure_mesh_config['mesh']
                urdf_path = os.path.join(self.ros2_ws, 'src', 'virtual_shake_robot_pybullet', urdf_path)

                self.get_logger().info(f"Loading rock mesh from {urdf_path}...")
                
                rock_position = self.rock_structure_mesh_config['rock_position']
                rock_id = p.loadURDF(urdf_path, rock_position, [0, 0, 0, 1], physicsClientId=self.client_id)
                

                dynamics_info = p.getDynamicsInfo(rock_id, -1)

                self.get_logger().info(f"The Dynamics before is {dynamics_info}")

                mass = self.rock_structure_mesh_config['mass']
                restitution = self.rock_structure_mesh_config['restitution']
                lateralFriction = self.rock_structure_mesh_config['lateralFriction']
                spinningFriction = self.rock_structure_mesh_config['spinningFriction']
                contactDamping = self.rock_structure_mesh_config['contactDamping']
                contactStiffness = self.rock_structure_mesh_config['contactStiffness']
                
                p.changeDynamics(
                    rock_id, -1,
                    restitution=restitution,
                    lateralFriction=lateralFriction,
                    spinningFriction=spinningFriction,
                    contactDamping=contactDamping,
                    contactStiffness=contactStiffness,
                    physicsClientId=self.client_id
                )
                dynamics_info = p.getDynamicsInfo(rock_id, -1)

                self.get_logger().info(f"The Dynamics info through callback is {dynamics_info}")

               
            elif structure_type == 'box':
                # Retrieve parameters from parameter server
                rock_dimensions = self.rock_structure_box_config['dimensions']
                rock_mass = self.rock_structure_box_config['mass']
                rock_restitution = self.rock_structure_box_config['restitution']
                rock_lateralFriction = self.rock_structure_box_config['lateralFriction']
                rock_spinningFriction = self.rock_structure_box_config['spinningFriction']
                rock_contactDamping = self.rock_structure_box_config['contactDamping']
                rock_contactStiffness = self.rock_structure_box_config['contactStiffness']
                principal_moments = self.rock_structure_box_config['localInertiaDiagonal']

                self.get_logger().info("Loading rock as a box...")
                collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[d / 2 for d in rock_dimensions])
                visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[d / 2 for d in rock_dimensions], rgbaColor=[0.5, 0.5, 0.5, 1])
                

                rock_position = self.rock_structure_box_config['rock_position']

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
        
    def get_recording_folder_name(self):
        """Retrieves the folder name from a parameter."""
        folder_name = self.get_parameter('recording_folder_name').value
        self.get_logger().info("The recording folder executed")
        if not folder_name:
            self.get_logger().error("Recording folder name is not provided.")
            raise ValueError("Recording folder name is required.")
            
        ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))
        recordings_folder = os.path.join(ros2_ws, 'src', 'virtual_shake_robot_pybullet', 'recordings', folder_name)
        os.makedirs(recordings_folder, exist_ok=True)
        return recordings_folder
    
    def execute_velocity_trajectory_callback(self, goal_handle):
        """
        Executes a velocity-only trajectory using the VelocityTrajectory action.
        Advances the simulation in small steps for each time interval from the provided timestamps,
        records measured velocity and position, and then plots comparisons between:
        - Desired vs. measured velocities
        - Desired (integrated) vs. measured positions
        """
        self.get_logger().info("Executing velocity-only trajectory callback...")
        
        desired_velocities = goal_handle.request.velocity_list
        timestamps = goal_handle.request.timestamp_list
        sim_dt = self.engine_settings['timestep']  # e.g., 0.001 s

        measured_velocities = []  # Recorded measured velocities at each command interval
        measured_positions = []   # Recorded measured positions
        recorded_times = []       # Recorded timestamps matching each measurement

        # Loop over each desired velocity command
        for i in range(len(desired_velocities)):
            # Set the velocity command
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=0,  # adjust if necessary
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=desired_velocities[i],
                force=5e8,  # adjust force as needed
                physicsClientId=self.client_id
            )
            
            # Determine the duration for which to apply this command.
            # For all but the last command, use the difference between consecutive timestamps.
            if i < len(timestamps) - 1:
                dt = timestamps[i+1] - timestamps[i]
            else:
                dt = sim_dt  # Fallback for the last command

            # Compute how many simulation steps to perform to cover the interval dt.
            num_steps = int(dt / sim_dt)
            # Step the simulation repeatedly to cover the entire dt.
            for _ in range(num_steps):
                p.stepSimulation(physicsClientId=self.client_id)
                if self.realtime_flag:
                    # Sleep to maintain real-time pacing
                    time.sleep(sim_dt)
                

            # After stepping, record the measured state.
            joint_state = p.getJointState(self.robot_id, 0, physicsClientId=self.client_id)
            measured_velocities.append(joint_state[1])  # Velocity at joint index 0
            measured_positions.append(joint_state[0])   # Position at joint index 0
            
            # Use the end of the interval as the recorded time.
            recorded_times.append(timestamps[i] + dt)

        # (Optional) Trim arrays if there is any slight mismatch
        min_len = min(len(recorded_times), len(desired_velocities))
        recorded_times = recorded_times[:min_len]
        desired_velocities = desired_velocities[:min_len]
        measured_velocities = measured_velocities[:min_len]
        
        # Plot the velocity comparison.
        if self.enable_plotting:
            self.plot_velocity_comparison(recorded_times, desired_velocities, measured_velocities)
        
        # Compute desired positions by integrating the desired velocities using a simple Riemann sum.
        # Assume initial desired position is 0.
        desired_positions = [0.0]
        for i in range(1, len(timestamps)):
            dt = timestamps[i] - timestamps[i-1]
            new_pos = desired_positions[-1] + desired_velocities[i-1] * dt
            desired_positions.append(new_pos)
        # Trim desired_positions to match recorded_times length if needed.
        desired_positions = desired_positions[:min_len]
        measured_positions = measured_positions[:min_len]
        
        # Plot the position comparison.
        if self.enable_plotting:
            self.plot_position_comparison(recorded_times, desired_positions, measured_positions)
        
        # Complete the action result.
        goal_handle.succeed()
        result = VelocityTrajectory.Result()
        result.success = True
        result.final_position = measured_positions[-1] if measured_positions else 0.0
        return result

    def plot_velocity_comparison(self, timestamps, desired_velocities, measured_velocities):
        """
        Plots the desired velocity versus the measured (pedestal) velocity.

        Args:
            timestamps (list): Time stamps corresponding to each command.
            desired_velocities (list): Velocity values sent to the simulation.
            measured_velocities (list): Velocity values measured from the pedestal.
        """
        import matplotlib.pyplot as plt
        plt.figure(figsize=(12, 6))
        plt.plot(timestamps, desired_velocities, 'r-', label="Desired Velocity")
        plt.plot(timestamps, measured_velocities, 'b--', label="Measured Velocity")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.title("Comparison of Desired and Measured Pedestal Velocity")
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_position_comparison(self, timestamps, desired_positions, measured_positions):
        """
        Plots the desired position (computed by integrating desired velocity) versus
        the measured pedestal position.

        Args:
            timestamps (list): Time stamps corresponding to each measurement.
            desired_positions (list): Computed desired positions.
            measured_positions (list): Measured positions from the simulation.
        """
        import matplotlib.pyplot as plt
        plt.figure(figsize=(12, 6))
        plt.plot(timestamps, desired_positions, 'r-', label="Desired Position")
        plt.plot(timestamps, measured_positions, 'b--', label="Measured Position")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.title("Comparison of Desired and Measured Pedestal Position")
        plt.legend()
        plt.grid(True)
        plt.show()




    def execute_position_trajectory_callback(self, goal_handle):
        """
        Executes the trajectory based on the positions and timestamps provided in the LoadDispl action goal.
        After execution, it records the total time taken for the experiment and stores it in a CSV file.
        """
        self.logger.info("Received LoadDispl action goal...")

        positions = goal_handle.request.positions
        timestamps = goal_handle.request.timestamps
        test_no = goal_handle.request.test_no  # Unique test number

        if len(positions) != len(timestamps):
            self.logger.error("Length of positions and timestamps must be equal.")
            goal_handle.abort()
            return LoadDispl.Result(success=False)
        
        feedback_msg = LoadDispl.Feedback()

        # Initialize lists to store data for plotting and saving
        target_positions = []
        actual_positions = []
        simulation_timestamps = []
        simulation_data = []  # This will contain all the relevant data

        self.logger.info(f"Executing LoadDispl with {len(positions)} positions and {len(timestamps)} timestamps...")

        # Record the start time for the experiment
        experiment_start_time = time.time()

        for i in range(len(positions)):
            iteration_start_time = time.time()  # Record the start time of this iteration

            current_position = positions[i]
            simulation_timestamps.append(timestamps[i])

            # Simulate the joint control based on the position
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=0,
                controlMode=p.POSITION_CONTROL,
                targetPosition=current_position,
                force=5 * 10**8,
                maxVelocity=200,
                physicsClientId=self.client_id
            )

            # Get the actual joint state after the step
            joint_state = p.getJointState(self.robot_id, 0)
            actual_position = joint_state[0]

            # Store positions for later saving
            target_positions.append(current_position)
            actual_positions.append(actual_position)

            # Get the pose of the PBR model and save the pose information
            if self.rock_id is not None:
                pbr_position, pbr_orientation = p.getBasePositionAndOrientation(self.rock_id, physicsClientId=self.client_id)
            else:
                pbr_position = [0, 0, 0]
                pbr_orientation = [0, 0, 0, 1]
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position = Point(x=pbr_position[0], y=pbr_position[1], z=pbr_position[2])
            pose_msg.pose.orientation = Quaternion(x=pbr_orientation[0], y=pbr_orientation[1], 
                                                z=pbr_orientation[2], w=pbr_orientation[3])
            self.pbr_pose_publisher.publish(pose_msg)

            # Append data to simulation_data (to be saved later: timestamp, target position, actual position, PBR pose)
            simulation_data.append([
                timestamps[i],
                current_position,
                actual_position,
                pbr_position[0], pbr_position[1], pbr_position[2],
                pbr_orientation[0], pbr_orientation[1], pbr_orientation[2], pbr_orientation[3]
            ])

            # Capture feedback
            feedback_msg.current_position = current_position
            feedback_msg.current_timestamp = timestamps[i]
            goal_handle.publish_feedback(feedback_msg)

            # Calculate the time difference for the next step
            if i < len(timestamps) - 1:
                time_diff = float(timestamps[i + 1]) - float(timestamps[i])
                if time_diff <= 0:
                    time_diff = self.engine_settings['timestep']
            else:
                time_diff = self.engine_settings['timestep']

            # Step the simulation forward by the calculated time difference
            num_steps = int(time_diff / self.engine_settings['timestep'])
            for _ in range(num_steps):
                p.stepSimulation(physicsClientId=self.client_id)

            # Ensure each loop iteration takes approximately time_diff seconds
            iteration_end_time = time.time()
            elapsed_time = iteration_end_time - iteration_start_time
            sleep_time = time_diff - elapsed_time
            if self.realtime_flag and sleep_time > 0:
                time.sleep(sleep_time)

        # Record the end time and compute the total execution time for the experiment
        experiment_end_time = time.time()
        total_execution_time = experiment_end_time - experiment_start_time
        self.get_logger().info(f"Total execution time for this experiment: {total_execution_time:.4f} seconds")

        # Store the experiment's execution time along with the test number in a CSV file
        csv_file = os.path.join(self.recordings_folder, "experiment_times.csv")
        current_experiment = pd.DataFrame({
            "test_no": [test_no],
            "execution_time": [total_execution_time]
        })

        if not os.path.exists(csv_file):
            current_experiment.to_csv(csv_file, index=False)
        else:
            current_experiment.to_csv(csv_file, mode='a', header=False, index=False)

        # Save simulation data to a numpy file as before
        full_namespace = self.get_namespace()
        sim_no = full_namespace.split('/')[1]
        file_name = f"trajectory_{sim_no}_test_{int(test_no)}.npy"
        file_path = os.path.join(self.recordings_folder, file_name)
        self.get_logger().info(f"Length of the .npy file(before): {len(simulation_data)}")
        np.save(file_path, np.array(simulation_data))
        self.get_logger().info(f"Simulation data saved to {file_path}")
        del simulation_data
        gc.collect()

        goal_handle.succeed()
        result = LoadDispl.Result()
        result.success = True
        return result

    def execute_pos_vel_trajectory_callback(self, goal_handle):
        """
        Executes the trajectory from the data received in the TrajectoryAction goal.
        Now also logs simulation data during the response_wait_time period at 100 Hz.
        """
        self.logger.info("Executing trajectory callback...")

        # Initialize lists to store data for plotting and saving
        desired_positions = []
        desired_velocities = []
        simulation_data = []  # This will contain all the relevant data

        client_id = self.client_id
        robot_id = goal_handle.request.robot_id
        positions = goal_handle.request.position_list
        velocities = goal_handle.request.velocity_list
        timestamps = goal_handle.request.timestamp_list
        response_wait_time = goal_handle.request.response_wait_time
        amplitude = goal_handle.request.amplitude
        frequency = goal_handle.request.frequency
        feedback_msg = TrajectoryAction.Feedback()

        if not (len(positions) == len(velocities) == len(timestamps)):
            self.logger.error("Length of positions, velocities, and timestamps must be equal.")
            goal_handle.abort()
            return TrajectoryAction.Result(success=False)

        self.get_logger().info(f"Length of the trajectory: {len(positions)}")
        time_step = self.engine_settings['timestep']
        loop_start_time = time.time()

        # ====== Main control loop ======
        for i in range(len(timestamps)):
            iteration_start_time = time.time()

            desired_positions.append(positions[i])
            desired_velocities.append(velocities[i])

            self.desired_position_publisher.publish(Float64(data=positions[i]))
            self.desired_velocity_publisher.publish(Float64(data=velocities[i]))

            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=0,
                controlMode=p.POSITION_CONTROL,
                targetPosition=positions[i],
                targetVelocity=velocities[i],
                force=5e8,
                maxVelocity=200,
                physicsClientId=client_id
            )

            p.stepSimulation(physicsClientId=client_id)

            joint_state = p.getJointState(robot_id, 0, physicsClientId=client_id)
            actual_position, actual_velocity = joint_state[0], joint_state[1]

            if self.rock_id is not None:
                pbr_position, pbr_orientation = p.getBasePositionAndOrientation(self.rock_id, physicsClientId=client_id)
            else:
                pbr_position = [0, 0, 0]
                pbr_orientation = [0, 0, 0, 1]

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position = Point(*pbr_position)
            pose_msg.pose.orientation = Quaternion(*pbr_orientation)
            self.pbr_pose_publisher.publish(pose_msg)

            simulation_data.append([
                timestamps[i],
                actual_position,
                actual_velocity,
                *pbr_position,
                *pbr_orientation
            ])

            self.position_publisher.publish(Float64(data=actual_position))
            self.velocity_publisher.publish(Float64(data=actual_velocity))

            feedback_msg.current_position = actual_position
            feedback_msg.current_velocity = actual_velocity
            goal_handle.publish_feedback(feedback_msg)

            elapsed_time = time.time() - iteration_start_time
            sleep_time = time_step - elapsed_time
            if self.realtime_flag and sleep_time > 0:
                time.sleep(sleep_time)

        # ====== New: Logging during response_wait_time ======
        self.get_logger().info(f"Logging during response_wait_time: {response_wait_time} seconds at 100 Hz.")
        logging_freq = 100  # Hz
        log_dt = 1.0 / logging_freq
        end_wait_time = time.time() + response_wait_time

        while time.time() < end_wait_time:
            current_time = time.time()
            p.stepSimulation(physicsClientId=client_id)

            joint_state = p.getJointState(robot_id, 0, physicsClientId=client_id)
            actual_position, actual_velocity = joint_state[0], joint_state[1]

            if self.rock_id is not None:
                pbr_position, pbr_orientation = p.getBasePositionAndOrientation(self.rock_id, physicsClientId=client_id)
            else:
                pbr_position = [0, 0, 0]
                pbr_orientation = [0, 0, 0, 1]

            # Relative timestamp since start
            timestamp = current_time - loop_start_time
            simulation_data.append([
                timestamp,
                actual_position,
                actual_velocity,
                *pbr_position,
                *pbr_orientation
            ])

            if self.realtime_flag:
                time.sleep(log_dt)

        # ====== Save data ======
        loop_end_time = time.time()
        total_execution_time = loop_end_time - loop_start_time
        self.get_logger().info(f"Total execution time: {total_execution_time:.4f} seconds")

        ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))
        recordings_folder = os.path.join(ros2_ws, 'src', 'virtual_shake_robot_pybullet', 'recordings', 'recordings_grid_cosine')
        os.makedirs(recordings_folder, exist_ok=True)

        file_name = f"trajectory_pos_vel_{amplitude}_{frequency}.npy"
        file_path = os.path.join(recordings_folder, file_name)
        self.get_logger().info(f"Length of simulation data: {len(simulation_data)}")
        np.save(file_path, np.array(simulation_data))
        self.get_logger().info(f"Simulation data saved to {file_path}")
        del simulation_data
        gc.collect()

        self.get_logger().info("Trajectory execution completed successfully.")
        goal_handle.succeed()
        result = TrajectoryAction.Result()
        result.success = True
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
