#!/usr/bin/env python3
import csv
import pybullet as p
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from virtual_shake_robot_pybullet.action import TrajectoryAction, AF, RecordingAction, LoadDispl
from std_srvs.srv import Empty
from math import pi
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Bool, Float64
from virtual_shake_robot_pybullet.srv import ManageModel
from geometry_msgs.msg import PoseStamped
import transforms3d.euler as euler
import threading
from rclpy.executors import MultiThreadedExecutor
from data_loader import DataLoader
import os
from ament_index_python.packages import get_package_share_directory
import argparse
from rclpy.parameter import Parameter

class ControlNode(Node):
    def __init__(self):
        """
        A ROS2 node that manages the control and execution of different experiments using the Virtual Shake Robot (VSR) simulation.

        The ControlNode class is responsible for:
        
        - Handling different types of experiments, including single and grid cosine experiments, and recordings.
        - Managing action clients for trajectory, recording, and displacement actions.
        - Publishing and subscribing to relevant ROS topics.
        - Plotting and logging data related to the experiments.
        """
        super().__init__('control_node')
        self.new_pose_received = False
        self.enable_plotting = self.declare_parameter('enable_plotting', False).value
        self.time_step = self.declare_parameter('engineSettings.timeStep', 0.001).value
        self.response_wait_time = self.declare_parameter('engineSettings.response_wait_time', 5.0).value
        self.loading_wait_time = self.declare_parameter('engineSettings.loading_wait_time', 5.0).value
        self.control_frequency = 1.0 / self.time_step

        # Action clients
        self._trajectory_action_client = ActionClient(self, TrajectoryAction, 'trajectory_action')
        self._recording_action_client = ActionClient(self, RecordingAction, 'manage_recording')
        self._displacement_action_client = ActionClient(self, LoadDispl, 'load_displ_action')

        # Publishers and Subscribers
        self.ready_publisher = self.create_publisher(Bool, 'control_node_ready', 10)
        self.create_subscription(PoseStamped, 'pbr_pose_topic', self.pbr_pose_callback, 10)

        # Live plotting variables
        self.live_fig = None
        self.live_ax = None
        self.live_toppling_data = []

        # Action server for manual amplitude-frequency setting
        self._action_server = ActionServer(
            self,
            AF,
            'set_amplitude_frequency_manual',
            self.run_single_cosine_experiment
        )

        # Initialize some defaults
        self.robot_id = 1
        self.rock_id = None
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.timestamps = []
        self.positions = []
        self.velocities = []
        self.latest_pose = None
        self.experiment_thread = None
        self.toppling_data = []
        self.amplitude_list = []
        self.frequency_list = []
        self.current_index = 0
        self.pedestal_reset_time = self.declare_parameter('simulation_node.engineSettings.pedestal_reset_time', 1.0).value

        # Publishers for PGA and PGV
        self.pga_publisher = self.create_publisher(Float64, 'pga_topic', 10)
        self.pgv_publisher = self.create_publisher(Float64, 'pgv_topic', 10)

        # Motion mode and test_no parameters
        self.motion_mode = self.declare_parameter('motion_mode', '').value
        self.test_no = self.declare_parameter('test_no', Parameter.Type.INTEGER, None).value

        # We only create the manage_model client here. We may or may not use it depending on the mode.
        self._manage_model_client = self.create_client(ManageModel, 'manage_model')

        # The following steps load YAML and DataLoader only if needed.
        if self.motion_mode in ['single_recording', 'all_recordings']:
            # Get the package share directory
            ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))
            config_path = os.path.join(ros2_ws, 'src', 'virtual_shake_robot_pybullet', 'config', 'data_path.yaml')

            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)

            if not config['file_paths']['excel_file_path'] or not config['file_paths']['folder_path'] or not config['file_paths']['pickle_file_path']:
                raise ValueError("One or more paths in the configuration file are empty.")
                    
            # Retrieve file paths from the configuration
            excel_file_path = os.path.join(ros2_ws, config['file_paths']['excel_file_path'])
            folder_path = os.path.join(ros2_ws, config['file_paths']['folder_path'])
            pickle_file_path = os.path.join(ros2_ws, config['file_paths']['pickle_file_path'])

            # Validate file paths
            if not os.path.exists(excel_file_path) or not os.path.exists(folder_path):
                raise FileNotFoundError("One or more paths specified in the YAML configuration file are invalid.")

            # Optionally print the paths for debugging
            self.get_logger().info(f"Excel file path: {excel_file_path}")
            self.get_logger().info(f"Folder path: {folder_path}")
            self.get_logger().info(f"Pickle file path: {pickle_file_path}")

            # Initialize the DataLoader and load the data
            self.data_loader = DataLoader(
                excel_file_path=excel_file_path,
                folder_path=folder_path,
                pickle_file_path=pickle_file_path
            )

            self.combined_data = self.data_loader.get_combined_data()
        else:
            # For other modes, no data loading is required
            self.data_loader = None
            self.combined_data = None

        self.get_logger().info("ControlNode initialized and action servers started.")

        # Now handle the selected motion mode
        if self.motion_mode == 'single_cosine':
            # Run single cosine experiment or just wait for manual AF action
            pass
        elif self.motion_mode == 'grid_cosine':
            self.run_grid_cosine_experiment()
        elif self.motion_mode == 'single_recording':
            if self.test_no is None:
                self.get_logger().error("Test number must be provided for single_recording mode.")
                raise ValueError("Test number must be provided for single_recording mode.")
            self.run_single_recording_experiment(self.test_no)
        elif self.motion_mode == 'all_recordings':
            self.run_all_recording_experiments()
        
        
    def run_single_recording_experiment(self, test_no):
        """
        Executes a single recording experiment by sending displacement data for a specific test_no to the simulation node.

        Args:
            test_no (int): The test number for which to run the experiment.

        Returns:
            bool: True if the experiment succeeded, False otherwise.

        """

        if self.rock_id is None:
            self.spawn_initial_model()
        if test_no in self.combined_data:
            test_data = self.combined_data[test_no]
            self.get_logger().info(f"Data type of test_data: {type(test_data)}")
            

            time_vs_displacement_df = test_data['Time vs Displacement']
           

            timestamps = time_vs_displacement_df.iloc[:, 0].tolist()
            

            positions = time_vs_displacement_df.iloc[:, 1].tolist()
            self.get_logger().info(f"Sending displacement data for Test No: {test_no}")

            displacement_goal = LoadDispl.Goal()
            displacement_goal.positions = positions
            displacement_goal.timestamps = timestamps
            displacement_goal.test_no = test_no

            self._displacement_action_client.wait_for_server()
            send_goal_future = self._displacement_action_client.send_goal_async(displacement_goal)

            # Wait for the goal to be sent and accepted
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                del send_goal_future, goal_handle
                self.get_logger().error('Goal rejected by trajectory action server.')
                return False

            # Now wait for the result
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)

            result = get_result_future.result()

            # Check the success attribute inside the result object
            if result.result.success:
                self.get_logger().info('Displacement action succeeded.')
                del send_goal_future, goal_handle, get_result_future, result
                return True
            else:
                self.get_logger().error('Displacement action failed.')
                del send_goal_future, goal_handle, get_result_future, result
                return False
        else:
            self.get_logger().error(f"Test data for Test No: {test_no} not found!")
            return False


    def update_live_plot(self, pga, pgv_to_pga, toppled):
        """
        Updates the live plot of PGA_g vs PGV/PGA with the new data point.
        Args:
            pga (float): The peak ground acceleration.
            pgv_to_pga (float): The ratio of PGV to PGA.
            toppled (bool): Whether the rock has toppled or not.
        """
        if not self.enable_plotting:
            self.get_logger().info("Plotting is disabled.")
            return

        # Calculate PGA_g
        PGA_g = pga / 9.807

        # Append the data point to self.live_toppling_data
        self.live_toppling_data.append((PGA_g, pgv_to_pga, toppled))

        # Prepare data for plotting
        toppled_data = [(x, y) for x, y, t in self.live_toppling_data if t]
        not_toppled_data = [(x, y) for x, y, t in self.live_toppling_data if not t]

        # Initialize figure and axis if not already done
        if self.live_fig is None or self.live_ax is None:
            plt.ion()  # Enable interactive mode
            self.live_fig, self.live_ax = plt.subplots(figsize=(10, 5))
            self.live_ax.set_xlabel('PGA_g')
            self.live_ax.set_ylabel('PGV / PGA')
            self.live_ax.set_title('Toppling Status')
            self.live_ax.grid(True)
            plt.show(block=False)

        # Clear the axes for updating
        self.live_ax.cla()
        self.live_ax.set_xlabel('PGA_g')
        self.live_ax.set_ylabel('PGV / PGA')
        self.live_ax.set_title('Toppling Status')
        self.live_ax.grid(True)

        # Plot data points
        if not_toppled_data:
            x_vals, y_vals = zip(*not_toppled_data)
            self.live_ax.scatter(x_vals, y_vals, c='b', marker='v', label='Not Toppled')
        if toppled_data:
            x_vals, y_vals = zip(*toppled_data)
            self.live_ax.scatter(x_vals, y_vals, c='r', label='Toppled')

        # Update legend without duplicates
        handles, labels = self.live_ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        self.live_ax.legend(by_label.values(), by_label.keys(), loc='upper right')

        # Refresh the plot
        self.live_fig.canvas.draw()
        self.live_fig.canvas.flush_events()

        self.get_logger().info("Live plot updated.")

    def pbr_pose_callback(self, msg):
        """
        Callback to handle the latest pose of the PBR, updating the internal state.

        Args:
            msg (PoseStamped): The PoseStamped message containing the latest pose of the PBR.

        Returns:
            None
        """
    
        self.latest_pose = msg.pose
        self.new_pose_received = True
        

    def sample_motion_param(self):
        """
        Generates sample motion parameters (frequency and amplitude) for grid cosine experiments.

        Returns:
            np.array: An array of tuples containing the sampled frequency and amplitude pairs.
        """
        PGA = np.linspace(0.1, 0.5, 3)
        PGV_2_PGA = np.linspace(0.1, 0.5, 3)
        Fs = 1.0 / (2 * pi * PGV_2_PGA)
        FA_data = []
        for F in Fs:
            for pga in PGA:
                A = 9.807 * pga / (4 * pi**2 * F**2)
                FA_data.append((F, A))
        for pair in FA_data:
            print("Frequency: {:.2f}, Amplitude: {:.6f}".format(pair[0], pair[1])) 
        return np.asarray(FA_data)
    
    def run_all_recording_experiments(self):
        """
        Runs recording experiments for all test numbers dynamically found from the combined data,
        handling pose updates and toppling status.

        Returns:
            None
        """
        # Get the workspace src directory dynamically
        ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))
        src_directory = os.path.join(ros2_ws, 'src', 'virtual_shake_robot_pybullet', 'data')

        # Define the file path using the simulation namespace
        sim_no = self.get_namespace().replace('/', '')
        csv_file_path = os.path.join(src_directory, f'toppling_status_{sim_no}.csv')

        self.get_logger().info(f"Saving the CSV file at: {csv_file_path}")
        topple_status_array = []

        # Dynamically find all the test numbers in the combined data
        test_numbers = sorted(self.combined_data.keys())
        self.get_logger().info(f"Found {len(test_numbers)} test numbers: {test_numbers}")

        for test_no in test_numbers:
            self.get_logger().info(f"Starting experiment on Test No: {test_no}")

            # Extract PGV, PGA, and PGV/PGA for the current test
            if test_no in self.combined_data:
                test_data = self.combined_data[test_no]
                pgv_to_pga = test_data.get('PGV/PGA', None)
                pga = test_data.get('Scaled PGA', None)

                if pgv_to_pga is None or pga is None:
                    self.get_logger().warning(f"Missing data for Test No: {test_no}. Skipping this test.")
                    continue

                self.get_logger().info(f"Started recording for Test No: {test_no} with PGA: {pga}, PGV/PGA: {pgv_to_pga}")
            else:
                self.get_logger().error(f"No data available for Test No: {test_no}. Skipping this test.")
                continue

            success = self.run_single_recording_experiment(test_no)

            if success:
                self.get_logger().info(f"Experiment on Test No: {test_no} displacement data sent successfully.")

                # Spin once to make sure the pose is the most updated
                rclpy.spin_once(self, timeout_sec=0.1)

                # Reset the flag before waiting for a new pose
                self.new_pose_received = False

                # Once the pose is received, process it
                if self.latest_pose:
                    self.get_logger().info(f"Processing pose for Test No: {test_no}: {self.latest_pose}")
                    toppled = self.check_Toppled(self.latest_pose)
                    self.update_live_plot(pga, pgv_to_pga, toppled)
                    if toppled:
                        self.get_logger().info("The Rock has toppled.")
                        topple_status_array.append(1)
                    else:
                        self.get_logger().info("The rock has not toppled.")
                        topple_status_array.append(0)
                else:
                    self.get_logger().warning(f"No pose received for Test No: {test_no} after waiting.")
                    topple_status_array.append(0)
            else:
                self.get_logger().error(f"Experiment on Test No: {test_no} failed or was not completed.")

            self.get_logger().info(f"Completed Test No: {test_no}. Moving to the next test.")

            # Delete and respawn the model for the next test
            self.reset_model_postion_and_orientation()

            self.rock_id = 2

            rclpy.spin_once(self, timeout_sec=0.001)

        # After all tests, save toppling status to CSV
        self.save_toppling_status_to_csv(topple_status_array, csv_file_path)


  

    def save_toppling_status_to_csv(self, topple_status_array,csv_file_path):
        """
        Save the toppling status array to a CSV file.

        Args:
            topple_status_array (list): List of toppling statuses for each test.
        """
        
        # Open the file in write mode and save the toppling status
        with open(csv_file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Test No', 'Toppling Status'])
            for i, status in enumerate(topple_status_array, start=1):  # Assuming test_no starts from 1
                writer.writerow([i, status])  # Write test number and corresponding toppling status
        
        self.get_logger().info(f"Toppling status saved to {csv_file_path}")
        
    def plot_pgv_pga(self, pga, pgv_to_pga, toppled):
        """
        Plots the PGV vs PGA data, updating the visualization with toppling status.

        Args:
            pga (float): The peak ground acceleration.
            pgv_to_pga (float): The ratio of PGV to PGA.
            toppled (bool): Whether the rock has toppled or not.

        Returns:
            None
        """

        if not self.enable_plotting:
            self.get_logger().info("Plotting is disabled.")
            return
        
        PGV = pga * pgv_to_pga
        PGA_g = pga / 9.807  

        # Calculate the range for the xlim and ylim
        pga_min = min(self.toppling_data, key=lambda x: x[0])[0] if self.toppling_data else PGA_g
        pga_max = max(self.toppling_data, key=lambda x: x[0])[0] if self.toppling_data else PGA_g
        pgv_to_pga_min = min(self.toppling_data, key=lambda x: x[1])[1] if self.toppling_data else pgv_to_pga
        pgv_to_pga_max = max(self.toppling_data, key=lambda x: x[1])[1] if self.toppling_data else pgv_to_pga

        # Extend the limits slightly for better visualization
        xlim_min = pga_min - 0.05 * (pga_max - pga_min)
        xlim_max = pga_max + 0.05 * (pga_max - pga_min)
        ylim_min = pgv_to_pga_min - 0.05 * (pgv_to_pga_max - pgv_to_pga_min)
        ylim_max = pgv_to_pga_max + 0.05 * (pgv_to_pga_max - pgv_to_pga_min)

        if not plt.get_fignums():
            plt.figure(figsize=(10, 5))
            plt.xlabel('PGA_g')
            plt.ylabel('PGV / PGA')
            plt.title('Toppling Status')
            plt.xlim(xlim_min, xlim_max)
            plt.ylim(ylim_min, ylim_max)
            plt.grid(True)
            plt.show(block=False)

        legend = plt.gca().get_legend()
        legend_texts = [text.get_text() for text in legend.get_texts()] if legend else []

        if toppled:
            plt.scatter(PGA_g, pgv_to_pga, c='r', label='Toppled' if 'Toppled' not in legend_texts else "")
        else:
            plt.scatter(PGA_g, pgv_to_pga, c='b', marker="v", label='Not Toppled' if 'Not Toppled' not in legend_texts else "")

        if not legend:
            plt.legend(loc='upper right')

        # Refresh the plot
        plt.draw()
        plt.pause(0.05)
        plt.close()
        self.get_logger().info("PGV vs PGA plot updated.")

    def run_grid_cosine_experiment(self):
        """
        Runs a grid cosine experiment by sampling motion parameters and checking the toppling status.

        Returns:
            None
        """
        self.spawn_initial_model()

        FA_data = self.sample_motion_param()

        for idx, (F, A) in enumerate(FA_data):
            self.amplitude = A
            self.frequency = F
            self.get_logger().info(f'Starting experiment {idx + 1}/{len(FA_data)} with Amplitude: {self.amplitude}, Frequency: {self.frequency}')

            self.new_pose_received = False  # Reset the flag before sending the trajectory goal
            
            self.calculate_and_send_trajectory()

            self.get_logger().info(f'Waiting for {self.response_wait_time} seconds for trajectory to complete.')
            end_wait_time = time.time() + self.response_wait_time 
            while time.time() < end_wait_time:
                rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().info('Waiting for new pose...')
            while not self.new_pose_received:
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_pose:
                self.get_logger().info(f"Checking pose for experiment {idx + 1}: {self.latest_pose}")
                toppled = self.check_Toppled(self.latest_pose)
                if self.enable_plotting:
                    self.logData(A, F, toppled)
                if toppled:
                    self.get_logger().info("The Rock has toppled")
                else:
                    self.get_logger().info("The rock has not toppled")
            else:
                self.get_logger().warning(f"No pose received for experiment {idx + 1}")

            self.get_logger().info(f"Breaking here")

            self.reset_model_postion_and_orientation()
            

        self.get_logger().info(f"Completed all {len(FA_data)} experiments")
      

    def reset_model_postion_and_orientation(self):

        self.get_logger().info(f"Sending reset request to the simulation node")

        reset_request = ManageModel.Request()
        reset_request.action = "reset"

        reset_future = self._manage_model_client.call_async(reset_request)
        rclpy.spin_until_future_complete(self, reset_future)

        if reset_future.result().success:
            self.get_logger().info("Model reset successfully")
            time.sleep(self.response_wait_time)
        else:
            self.get_logger().info(f"Failed to reset, {reset_future.result().message}")

    def spawn_initial_model(self):
        """Spawns the initial model in the simulation using the manage_model service."""
        self.get_logger().info("Spawning the initial model...")
        while not self._manage_model_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service available, proceeding to spawn the initial model.')

        spawn_request = ManageModel.Request()
        spawn_request.action = "spawn"
        future = self._manage_model_client.call_async(spawn_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Initial model spawned successfully.")
        else:
            self.get_logger().error(f"Failed to spawn initial model: {future.result().message}")
        
    


    def run_single_cosine_experiment(self, goal_handle):
        """
        Runs a single cosine experiment with the provided amplitude and frequency.

        Args:
            goal_handle (GoalHandle): The handle to the goal received from the action server.

        Returns:
            AF.Result: The result of the experiment, indicating success or failure.
        """
        self.get_logger().info('Executing goal...')
        self.amplitude = goal_handle.request.a
        self.frequency = goal_handle.request.f

        self.get_logger().info(f'Received Amplitude: {self.amplitude} and Frequency: {self.frequency}')

        if self.amplitude == 0 or self.frequency == 0:
            self.get_logger().info("Resetting the Pedestal to the base position")
            self.reset_trajectory()
        else:
            self.calculate_and_send_trajectory()

        goal_handle.succeed()
        result = AF.Result()
        result.success = True
        return result

    def send_recording_goal(self, command, pga, pgv, test_no):
        """
        Sends a recording goal to the PerceptionNode.

        Args:
            command (str): 'start' or 'stop' command.
            pga (float): Peak Ground Acceleration value.
            pgv (float): Peak Ground Velocity value.
            test_no (int): The test number to include in the recording.

        Returns:
            None
        """
        self.get_logger().info(f"Sending {command} recording goal with PGA: {pga}, PGV: {pgv}, Test No: {test_no}")
        recording_goal = RecordingAction.Goal()
        recording_goal.command = command
        recording_goal.pga = pga
        recording_goal.pgv = pgv
        recording_goal.test_no = float(test_no)

        self._recording_action_client.wait_for_server()
        send_goal_future = self._recording_action_client.send_goal_async(recording_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

    def recording_goal_response_callback(self, future):
        """
        Handles the response from the recording goal, triggering the result callback if accepted.

        Args:
            future (Future): The future object representing the asynchronous result.

        Returns:
            None
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Recording goal rejected by action server.')
        else:
            self.get_logger().info('Recording goal accepted by action server.')
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.recording_result_callback)

    def recording_result_callback(self, future):
        """
        Handles the result of the recording goal, logging the outcome.

        Args:
            future (Future): The future object representing the asynchronous result.

        Returns:
            None
        """
        result = future.result().result
        if result.success:
            self.get_logger().info("Recording action succeeded!")
        else:
            self.get_logger().error("Recording action failed!")

    def delete_and_spawn_model(self):
        """Deletes the current model and spawns a new one using the manage_model service."""
        self.get_logger().info("Deleting and respawning the model...")
        delete_request = ManageModel.Request()
        delete_request.action = "delete"
        delete_future = self._manage_model_client.call_async(delete_request)
        rclpy.spin_until_future_complete(self, delete_future)
        if not delete_future.result().success:
            self.get_logger().error("Failed to delete model")
            return

        spawn_request = ManageModel.Request()
        spawn_request.action = "spawn"
        spawn_future = self._manage_model_client.call_async(spawn_request)
        rclpy.spin_until_future_complete(self, spawn_future)
        if not spawn_future.result().success:
            self.get_logger().error("Failed to spawn model")
            return

        self.get_logger().info("Model respawned successfully.")
        self.ready_publisher.publish(Bool(data=True))

    def reset_trajectory(self):
        """Resets the trajectory of the pedestal to the base position."""
        current_pose = self.get_current_pose()
        positions, velocities, timestamps = self.generate_reset_trajectory(current_pose)
        self.send_trajectory_goal(positions, velocities, timestamps)

    def get_current_pose(self):
        """
        Returns the current position of the pedestal.

        Returns:
            float: The current position of the pedestal.
        """
        return self.current_position

    def feedback_callback(self, feedback_msg):
        """
        Callback to update the current position and velocity from trajectory feedback.

        Args:
            feedback_msg (TrajectoryAction.Feedback): The feedback message containing the current position and velocity.

        Returns:
            None
        """
        self.current_position = feedback_msg.feedback.current_position
        self.current_velocity = feedback_msg.feedback.current_velocity
        self.get_logger().info(f"Current Position: {self.current_position}, Current Velocity: {self.current_velocity}")

    def calculate_and_send_trajectory(self):
        """Calculates the trajectory for the current amplitude and frequency, then sends it to the simulation."""
        self.get_logger().info(f'Calculating trajectory for Amplitude: {self.amplitude} and Frequency: {self.frequency}')
        positions, velocities, timestamps = self.generate_trajectory()
        self.send_trajectory_goal(positions, velocities, timestamps)

    def generate_trajectory(self):
        """
        Generates a cosine trajectory based on the current amplitude and frequency.

        Returns:
            tuple: A tuple containing the positions, velocities, and timestamps of the generated trajectory.
        """
        T = 1.0 / self.frequency
        num_samples = int(self.control_frequency * T)
        t = np.linspace(0, T, num_samples)
        positions = -self.amplitude * np.cos(2 * np.pi * self.frequency * t) + self.amplitude
        positions = positions - positions[0]

        velocities = 2 * np.pi * self.amplitude * self.frequency * np.sin(2 * np.pi * self.frequency * t)
        timestamps = t.tolist()
        positions = positions.tolist()
        velocities = velocities.tolist()

        self.timestamps = timestamps
        self.positions = positions
        self.velocities = velocities

        self.plot_trajectory(timestamps, positions, velocities)

        return positions, velocities, timestamps

    def check_Toppled(self, pose):
        """
        Checks if the PBR has toppled based on the given pose.

        Args:
            pose (Pose): The pose of the PBR.

        Returns:
            bool: True if the PBR has toppled, False otherwise.
        """
        q = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        r, p, y = euler.quat2euler(q)
        return (abs(r) + abs(p)) >= 0.1

    def logData(self, A, F, state):
        """
        Logs the data for the current amplitude and frequency, updating the PGV vs PGA plot.

        Args:
            A (float): The amplitude of the motion.
            F (float): The frequency of the motion.
            state (bool): The state indicating whether the rock has toppled.

        Returns:
            None
        """
        PGV = 2 * pi * A * F
        PGA = 4 * pi**2 * F**2 * A
        PGV_2_PGA = PGV / PGA
        PGA_g = PGA / 9.807

        if not plt.get_fignums():  # Create the figure once if it doesn't exist
            plt.figure(figsize=(10, 5))
            plt.xlabel('PGA_g')
            plt.ylabel('PGV / PGA')
            plt.title('Toppling Status')
            plt.grid(True)
            plt.xlim(0, 0.6)  # Example fixed limits, adjust as necessary
            plt.ylim(0, 0.6)

        legend = plt.gca().get_legend()
        legend_texts = [text.get_text() for text in legend.get_texts()] if legend else []

        if state:
            # Toppled
            plt.scatter(PGA_g, PGV_2_PGA, c='r', label='Toppled' if 'Toppled' not in legend_texts else "")
            self.toppling_data.append((PGA_g, PGV_2_PGA, 1))
        else:
            plt.scatter(PGA_g, PGV_2_PGA, c='b', marker="v", label='Not Toppled' if 'Not Toppled' not in legend_texts else "")
            self.toppling_data.append((PGA_g, PGV_2_PGA, 0))

        if not legend:  # Only add the legend once
            plt.legend(loc='upper right')

        # Refresh the plot with a short pause to allow for dynamic updates
        plt.pause(0.05)
        plt.close()

    def get_Range(self, As, Fs):
        """
        Calculates the range of PGA and PGV/PGA values for the given amplitudes and frequencies.

        Args:
            As (list): A list of amplitude values.
            Fs (list): A list of frequency values.

        Returns:
            tuple: A tuple containing the maximum and minimum values of PGA_g and PGV/PGA.
        """
        data = []
        for A in As:
            for F in Fs:
                PGV = 2 * pi * A * F
                PGA = 4 * pi**2 * F**2 * A
                PGV_2_PGA = PGV / PGA
                PGA_g = PGV / 9.807
                data.append((PGA_g, PGV_2_PGA))

        nd = np.asarray(data)
        return (nd[:, 0].max(), nd[:, 0].min(), nd[:, 1].max(), nd[:, 1].min())

    def plot_trajectory(self, timestamps, positions, velocities):
        """
        Plots the generated trajectory if plotting is enabled.

        Args:
            timestamps (list): A list of timestamps.
            positions (list): A list of positions.
            velocities (list): A list of velocities.

        Returns:
            None
        """
        if self.enable_plotting:
            plt.figure(figsize=(10, 5))
            plt.plot(timestamps, positions, label='Desired Position')
            plt.plot(timestamps, velocities, label='Desired Velocity')
            plt.xlabel('Time (s)')
            plt.ylabel('Position / Velocity')
            plt.title('Desired Trajectory')
            plt.legend()
            plt.grid(True)

            # Save the plot with a relative path
            package_dir = os.path.dirname(os.path.realpath(__file__))
            graphs_dir = os.path.join(package_dir, 'graphs')
            os.makedirs(graphs_dir, exist_ok=True)
            filename = os.path.join(graphs_dir, f"Trajectory_A{self.amplitude}_F{self.frequency}.png")
            plt.savefig(filename)
            plt.show()
            plt.close()
        else:
            self.get_logger().info("Plotting is disabled.")

    def send_trajectory_goal(self, positions, velocities, timestamps):
        """
        Sends a trajectory goal to the TrajectoryAction server.

        Args:
            positions (list): A list of positions.
            velocities (list): A list of velocities.
            timestamps (list): A list of timestamps.

        Returns:
            None
        """
        trajectory_goal = TrajectoryAction.Goal()
        trajectory_goal.robot_id = self.robot_id
        trajectory_goal.position_list = positions
        trajectory_goal.velocity_list = velocities
        trajectory_goal.timestamp_list = timestamps
        trajectory_goal.response_wait_time = self.response_wait_time
        trajectory_goal.amplitude = self.amplitude
        trajectory_goal.frequency = self.frequency

        self.get_logger().info("Sending trajectory goal with positions, velocities, and timestamps.")
        if not self._trajectory_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Trajectory action server not available after waiting")
            return

        self.get_logger().info("Trajectory action server is available, sending goal.")
        send_goal_future = self._trajectory_action_client.send_goal_async(trajectory_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handles the response from the trajectory goal, triggering the result callback if accepted.

        Args:
            future (Future): The future object representing the asynchronous result.

        Returns:
            None
        """
        goal_handle = future.result()  # Store the goal handle
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by trajectory action server.')
        else:
            self.get_logger().info('Goal accepted by trajectory action server.')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.final_result_callback)

    def final_result_callback(self, future):
        """
        Handles the final result of the trajectory goal, plotting the results if successful.

        Args:
            future (Future): The future object representing the asynchronous result.

        Returns:
            None
        """
        result = future.result().result
        if result.success:
            self.get_logger().info("Trajectory action succeeded!")
            self.plot_trajectories(result.actual_positions, result.actual_velocities)
        else:
            self.get_logger().error("Trajectory action failed!")

    def generate_reset_trajectory(self, start_pose):
        """
        Generates a reset trajectory to bring the pedestal back to the base position.

        Args:
            start_pose (float): The starting position of the pedestal.

        Returns:
            tuple: A tuple containing the positions, velocities, and timestamps of the reset trajectory.
        """
        T = self.pedestal_reset_time
        num_samples = int(self.control_frequency * T)
        t = np.linspace(0, T, num_samples)
        positions = (start_pose / 2) * (1 - np.cos(np.pi * t / T))
        velocities = (start_pose * np.pi / T) * np.sin(np.pi * t / T)
        timestamps = t.tolist()
        positions = positions.tolist()
        velocities = velocities.tolist()
        return positions, velocities, timestamps

    def trajectory_feedback_callback(self, feedback_msg):
        """
        Handles feedback from the trajectory server, logging the current position and velocity.

        Args:
            feedback_msg (TrajectoryAction.Feedback): The feedback message containing the current position and velocity.

        Returns:
            None
        """
        self.get_logger().info(f"Feedback: {feedback_msg.feedback}")

    def plot_trajectories(self, actual_positions, actual_velocities):
        """
        Plots the actual positions and velocities against the desired trajectories.

        Args:
            actual_positions (list): A list of actual positions.
            actual_velocities (list): A list of actual velocities.

        Returns:
            None
        """
        if self.enable_plotting:
            if len(self.timestamps) == len(self.positions) == len(actual_positions) and len(self.timestamps) == len(self.velocities) == len(actual_velocities):
                plt.figure(figsize=(12, 6))

                plt.subplot(2, 1, 1)
                plt.plot(self.timestamps, self.positions, 'r-', label='Desired Position')
                plt.plot(self.timestamps, actual_positions, 'b--', label='Actual Position')
                plt.title('Position Comparison')
                plt.xlabel('Time (s)')
                plt.ylabel('Position')
                plt.legend()
                plt.grid(True)

                plt.subplot(2, 1, 2)
                plt.plot(self.timestamps, self.velocities, 'r-', label='Desired Velocity')
                plt.plot(self.timestamps, actual_velocities, 'b--', label='Actual Velocity')
                plt.title('Velocity Comparison')
                plt.xlabel('Time (s)')
                plt.ylabel('Velocity')
                plt.legend()
                plt.grid(True)

                plt.tight_layout()

                package_dir = os.path.dirname(os.path.realpath(__file__))
                graphs_dir = os.path.join(package_dir, 'graphs')
                os.makedirs(graphs_dir, exist_ok=True)

                # Save the plot with a relative path
                filename = os.path.join(graphs_dir, f"Trajectory_A{self.amplitude}_F{self.frequency}.png")
                plt.savefig(filename)
                plt.show()
                plt.close()
            else:
                self.get_logger().error("Lengths of timestamps, positions, and velocities do not match. Cannot plot trajectories.")

def main(args=None):
    """Main entry point for the ControlNode, initializing the ROS2 node and spinning with a multi-threaded executor."""
    rclpy.init(args=args)

    node = ControlNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected. Shutting down.')
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
