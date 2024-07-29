#!/usr/bin/env python3
import pybullet as p
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from virtual_shake_robot_pybullet.action import TrajectoryAction, AF, RecordingAction,LoadDispl
from std_srvs.srv import Empty
from math import pi
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Bool,Float64
from virtual_shake_robot_pybullet.srv import ManageModel
from geometry_msgs.msg import PoseStamped
import transforms3d.euler as euler
import threading
from rclpy.executors import MultiThreadedExecutor
from data_loader import DataLoader

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.new_pose_received  =False
        self.enable_plotting = self.declare_parameter('enable_plotting', False).value
        self.time_step = self.declare_parameter('simulation_node.engineSettings.timeStep', 0.001).value
        self.wait_time = self.declare_parameter('simulation_node.engineSettings.wait_time', 5.0).value
        self.model_wait_time = self.declare_parameter('simulation_node.engineSettings.model_wait_time', 10).value
        self.control_frequency = 1.0 / self.time_step
        self._trajectory_action_client = ActionClient(self, TrajectoryAction, 'trajectory_action')
        self._recording_action_client = ActionClient(self, RecordingAction, 'manage_recording')
        self._displacement_action_client = ActionClient(self, LoadDispl, 'load_displ_action')

        self.ready_publisher = self.create_publisher(Bool, 'control_node_ready', 10)
        self._action_server = ActionServer(
            self,
            AF,
            'set_amplitude_frequency_manual',
            self.execute_callback
        )

        # Initialize the DataLoader and load the data
        self.data_loader = DataLoader(
            excel_file_path='/home/akshay/ASU_ Shared_ Scans/Shake_ Table_ Response/Earthquake Records Info.xlsx',
            folder_path='/home/akshay/ASU_ Shared_ Scans/Shake_ Table_ Response',
            pickle_file_path='/home/akshay/ros2_ws/combined_data.pkl'
        )

        self.combined_data = self.data_loader.get_combined_data()
      
        self._manage_model_client = self.create_client(ManageModel, 'manage_model')

        self.get_logger().info("ControlNode initialized and action servers started.")
        self.client_id = 0
        self.robot_id = 1
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

        self.reset_time = self.declare_parameter('simulation_node.engineSettings.reset_time', 1.0).value
        
        # Publishers for PGA and PGV
        self.pga_publisher = self.create_publisher(Float64, 'pga_topic', 10)
        self.pgv_publisher = self.create_publisher(Float64, 'pgv_topic', 10)

        #Subscriber for the pose topic
        self.create_subscription(PoseStamped, 'pbr_pose_topic', self.pbr_pose_callback, 10)
        
        # Service to start the experiments
        self.create_service(Empty, 'start_experiments', self.start_experiments_callback)

        self.get_logger().info("Waiting for the start_experiments service to be called...")

        self.test_numbers = list(range(11,705))
        

        # self.send_displacement_data(116)
        self.run_continuous_experiments()


    def send_displacement_data(self, test_no):
        if test_no in self.combined_data:
            test_data = self.combined_data[test_no]
            self.get_logger().info(f"Data type of test_data: {type(test_data)}")
            self.get_logger().info(f"Loaded data for Test No: {test_no}: {list(test_data.keys())[:10]}")

            time_vs_displacement_df = test_data['Time vs Displacement']
            self.get_logger().info(f"Time vs Displacement Data (first 10 rows):\n{time_vs_displacement_df.head(10)}")

            timestamps = time_vs_displacement_df.iloc[:, 0].tolist()
            self.get_logger().info(f"Timestamps (first 10): {timestamps[:10]}")
            self.get_logger().info(f"Total timestamps: {len(timestamps)}")

            positions = time_vs_displacement_df.iloc[:, 1].tolist()
            self.get_logger().info(f"Sending displacement data for Test No: {test_no}")

            displacement_goal = LoadDispl.Goal()
            displacement_goal.positions = positions
            displacement_goal.timestamps = timestamps

            self._displacement_action_client.wait_for_server()
            send_goal_future = self._displacement_action_client.send_goal_async(displacement_goal)

            # Wait for the goal to be sent and accepted
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by trajectory action server.')
                return False

            # Now wait for the result
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)

            result = get_result_future.result()

            # Check the success attribute inside the result object
            if result.result.success:
                self.get_logger().info('Displacement action succeeded.')
                return True
            else:
                self.get_logger().error('Displacement action failed.')
                return False
        else:
            self.get_logger().error(f"Test data for Test No: {test_no} not found!")
            return False



                                

    def start_experiments_callback(self, request, response):
        self.get_logger().info("Starting experiments...")
        self.experiment_thread = threading.Thread(target=self.run_experiments)
        self.experiment_thread.start()
        return response

    def pbr_pose_callback(self, msg):
        self.latest_pose = msg.pose
        self.new_pose_received = True
        self.get_logger().info(f"Pose received: {self.latest_pose}")


    def get_robot_id(self):
        if self.robot_id is None:
            self.get_logger().error("Robot ID is not set.")
        return self.robot_id

    def sample_motion_param(self):
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
    
    def run_continuous_experiments(self):
        for test_no in range(11, 706):  # Test numbers from 011 to 705
            self.get_logger().info(f"Starting experiment on Test No: {test_no}")
            success = self.send_displacement_data(test_no)
            if success:
                self.get_logger().info(f"Experiment on Test No: {test_no} completed successfully.")
            else:
                self.get_logger().error(f"Experiment on Test No: {test_no} failed or was not completed.")


    def run_experiments(self):
        self.spawn_initial_model()

        FA_data = self.sample_motion_param()

        for idx, (F, A) in enumerate(FA_data):
            self.amplitude = A
            self.frequency = F
            self.get_logger().info(f'Starting experiment {idx + 1}/{len(FA_data)} with Amplitude: {self.amplitude}, Frequency: {self.frequency}')

            self.new_pose_received = False  # Reset the flag before sending the trajectory goal
            self.send_recording_goal('start', A, F)
            self.calculate_and_send_trajectory()

            self.get_logger().info(f'Waiting for {self.wait_time + 5.0} seconds for trajectory to complete.')
            end_wait_time = time.time() + self.wait_time + 5.0
            while time.time() < end_wait_time:
                rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().info('Waiting for new pose...')
            while not self.new_pose_received:
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_pose:
                self.get_logger().info(f"Checking pose for experiment {idx + 1}: {self.latest_pose}")
                toppled = self.check_Toppled(self.latest_pose)
                self.log_Data(A, F, toppled)
                if toppled:
                    self.get_logger().info("The Rock has toppled")
                else:
                    self.get_logger().info("The rock has not toppled")
            else:
                self.get_logger().warning(f"No pose received for experiment {idx + 1}")

            
            self.delete_and_spawn_model()

        self.get_logger().info(f"Completed all {len(FA_data)} experiments")
        self.send_recording_goal('stop', A, F)




 
    def spawn_initial_model(self):
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

    def execute_callback(self, goal_handle):
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
    

    def send_recording_goal(self, command, pga, pgv):
        self.get_logger().info(f"Sending recording goal: {command}, PGA: {pga}, PGV: {pgv}")
        
        recording_goal = RecordingAction.Goal()
        recording_goal.command = command
        recording_goal.pga = pga
        recording_goal.pgv = pgv

        if not self._recording_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Recording action server not available after waiting")
            return

        send_goal_future = self._recording_action_client.send_goal_async(recording_goal)
        send_goal_future.add_done_callback(self.recording_goal_response_callback)

    def recording_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Recording goal rejected by action server.')
        else:
            self.get_logger().info('Recording goal accepted by action server.')
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.recording_result_callback)

    def recording_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Recording action succeeded!")
        else:
            self.get_logger().error("Recording action failed!")


    
    def delete_and_spawn_model(self):
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
        current_pose = self.get_current_pose()
        positions, velocities, timestamps = self.generate_reset_trajectory(current_pose)
        self.send_trajectory_goal(positions, velocities, timestamps)
        
    def get_current_pose(self):
        return self.current_position
        
    def feedback_callback(self, feedback_msg):
        self.current_position = feedback_msg.feedback.current_position
        self.current_velocity = feedback_msg.feedback.current_velocity
        self.get_logger().info(f"Current Position: {self.current_position}, Current Velocity: {self.current_velocity}")

    def calculate_and_send_trajectory(self):
        self.get_logger().info(f'Calculating trajectory for Amplitude: {self.amplitude} and Frequency: {self.frequency}')
        positions, velocities, timestamps = self.generate_trajectory()
        # self.get_logger().info(f'Generated Positions: {positions}')
        # self.get_logger().info(f'Generated Velocities: {velocities}')
        # self.get_logger().info(f'Generated Timestamps: {timestamps}')
        self.send_trajectory_goal(positions, velocities, timestamps)

    def generate_trajectory(self):
        T = 1.0 / self.frequency
        num_samples = int(self.control_frequency * T)
        t = np.linspace(0, T, num_samples)
        positions = self.amplitude * np.cos(2 * np.pi * self.frequency * t) - self.amplitude
        positions = positions - positions[0]
        
        velocities = -2 * np.pi * self.amplitude * self.frequency * np.sin(2 * np.pi * self.frequency * t)
        timestamps = t.tolist()
        positions = positions.tolist()
        velocities = velocities.tolist()
        
        self.timestamps = timestamps
        self.positions = positions
        self.velocities = velocities

        self.plot_trajectory(timestamps, positions, velocities)
        
        return positions, velocities, timestamps
    
    def check_Toppled(self, pose):
        q = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        r, p, y = euler.quat2euler(q)
        return (abs(r) + abs(p)) >= 0.1

    def log_Data(self, A, F, state):
        PGV = 2 * pi * A * F
        PGA = 4 * pi**2 * F**2 * A
        PGV_2_PGA = PGV / PGA
        PGA_g = PGA / 9.807

        # Publish PGA and PGV values
        self.pga_publisher.publish(Float64(data=PGA))
        self.pgv_publisher.publish(Float64(data=PGV))

        legend_labels = [text.get_text() for text in plt.gca().get_legend().get_texts()] if plt.gca().get_legend() else []

        if state:
            if 'Toppled' not in legend_labels:
                plt.scatter(PGA_g, PGV_2_PGA, c='r', label='Toppled')
            else:
                plt.scatter(PGA_g, PGV_2_PGA, c='r')
            self.toppling_data.append((PGA_g, PGV_2_PGA, 1))
        else:
            if 'Not Toppled' not in legend_labels:
                plt.scatter(PGA_g, PGV_2_PGA, c='b', marker="v", label='Not Toppled')
            else:
                plt.scatter(PGA_g, PGV_2_PGA, c='b', marker="v")
            self.toppling_data.append((PGA_g, PGV_2_PGA, 0))

        plt.xlabel('PGA_g')
        plt.ylabel('PGV / PGA')
        plt.title('Toppling Status')

        # Set fixed axis limits
        plt.xlim(0, 0.6)  # Set the x-axis limit
        plt.ylim(0, 0.6)  # Set the y-axis limit

        plt.legend(loc='upper right')
        plt.grid(True)
        plt.pause(0.05)





    def get_Range(self, As, Fs):
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
        if self.enable_plotting:
            plt.figure(figsize=(10, 5))
            plt.plot(timestamps, positions, label='Desired Position')
            plt.plot(timestamps, velocities, label='Desired Velocity')
            plt.xlabel('Time (s)')
            plt.ylabel('Position / Velocity')
            plt.title('Desired Trajectory')
            plt.legend()
            plt.grid(True)
            plt.savefig('/home/akshay/Pictures/Plot_1.png') 
            plt.show()
        else:
            self.get_logger().info("PLotting is disabled.")

    def send_trajectory_goal(self, positions, velocities, timestamps):
        self.trajectory_goal = TrajectoryAction.Goal()
        self.trajectory_goal.client_id = self.client_id
        self.trajectory_goal.robot_id = self.robot_id
        self.trajectory_goal.position_list = positions
        self.trajectory_goal.velocity_list = velocities
        self.trajectory_goal.timestamp_list = timestamps
        self.trajectory_goal.wait_time = self.wait_time

        self.get_logger().info("Sending trajectory goal with positions, velocities, and timestamps.")
        if not self._trajectory_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Trajectory action server not available after waiting")
            return
        
        self.get_logger().info("Trajectory action server is available, sending goal.")
        send_goal_future = self._trajectory_action_client.send_goal_async(self.trajectory_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        self.goal_handle = future.result()  # Store the goal handle
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by trajectory action server.')
        else:
            self.get_logger().info('Goal accepted by trajectory action server.')
            
            get_result_future = self.goal_handle.get_result_async()
            get_result_future.add_done_callback(self.final_result_callback)

    def final_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Trajectory action succeeded!")
            self.plot_trajectories(result.actual_positions, result.actual_velocities)
        else:
            self.get_logger().error("Trajectory action failed!")

    
    def generate_reset_trajectory(self, start_pose):
        T = self.reset_time
        num_samples = int(self.control_frequency * T)
        t = np.linspace(0, T, num_samples)
        positions = (start_pose / 2) * (1 - np.cos(np.pi * t / T))
        velocities = (start_pose * np.pi / T) * np.sin(np.pi * t / T)
        timestamps = t.tolist()
        positions = positions.tolist()
        velocities = velocities.tolist()
        return positions, velocities, timestamps
    
    
    def trajectory_feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback}")

    def plot_trajectories(self, actual_positions, actual_velocities):
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
                filename = f"/home/akshay/Pictures/Trajectory_A{self.amplitude}_F{self.frequency}.png"
                plt.savefig(filename)
                plt.show()
            else:
                self.get_logger().error("Lengths of timestamps, positions, and velocities do not match. Cannot plot trajectories.")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    # Create a MultiThreadedExecutor to allow callbacks to be processed in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Keep the node running
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected. Shutting down.')
    finally:
        # Ensure proper shutdown
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




