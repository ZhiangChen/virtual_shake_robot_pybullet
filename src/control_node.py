#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from virtual_shake_robot_pybullet.action import TrajectoryAction
from virtual_shake_robot_pybullet.action import AF  
import numpy as np
import matplotlib.pyplot as plt

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.amplitude = None
        self.frequency = None
        self.time_step = self.declare_parameter('simulation_node.engineSettings.timeStep', 0.001).value
        self.control_frequency = 1.0 / self.time_step
        self._trajectory_action_client = ActionClient(self, TrajectoryAction, 'trajectory_action')
        self._action_server = ActionServer(
            self,
            AF,
            'set_amplitude_frequency',
            self.execute_callback
        )
        self.get_logger().info("ControlNode initialized and action server started.")
        self.client_id = 0
        self.robot_id = 2

        # Add attributes to store trajectory data
        self.timestamps = []
        self.positions = []
        self.velocities = []
        

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.amplitude = goal_handle.request.a
        self.frequency = goal_handle.request.f

        self.get_logger().info(f'Received Amplitude: {self.amplitude} and Frequency: {self.frequency}')

        # Calculate trajectory and send it
        self.calculate_and_send_trajectory()

        goal_handle.succeed()

        result = AF.Result()
        result.success = True
        return result

    def calculate_and_send_trajectory(self):
        positions, velocities, timestamps = self.generate_trajectory()
        self.send_trajectory_goal(positions, velocities, timestamps)

    def generate_trajectory(self):
        """Generate position and velocity trajectory based on amplitude and frequency."""
        T = 1.0 / self.frequency
        num_samples = int(self.control_frequency * T)
        t = np.linspace(0, T, num_samples)
        positions = self.amplitude * np.cos(2 * np.pi * self.frequency * t) - self.amplitude

        self.get_logger().info(f"Initial Positions: {positions}")
        # Adjust positions to start from zero
        positions = positions - positions[0]
        self.get_logger().info(f"Adjusted Positions: {positions}")
        velocities = -2 * np.pi * self.amplitude * self.frequency * np.sin(2 * np.pi * self.frequency * t)
        timestamps = t.tolist()
        positions = positions.tolist()
        velocities = velocities.tolist()
        
        
        self.timestamps = timestamps
        self.positions = positions
        self.velocities = velocities

        self.plot_trajectory(timestamps, positions, velocities)
        self.get_logger().info(f"Positions : {positions} , velocities:{velocities}")
        return positions, velocities, timestamps




    def plot_trajectory(self, timestamps, positions,velocities):
        plt.figure(figsize=(10, 5))
        plt.plot(timestamps, positions, label='Desired Position')
        plt.plot(timestamps,velocities, label = 'Desired Velocity ')
        plt.xlabel('Time (s)')
        plt.ylabel('Position')
        plt.title('Desired Position Trajectory')
        plt.legend()
        plt.grid(True)
        plt.savefig('/home/akshay/Pictures/Plot_1.png') 
        plt.show()

    def send_trajectory_goal(self, positions, velocities, timestamps):
        goal_msg = TrajectoryAction.Goal()
        goal_msg.client_id = self.client_id
        goal_msg.robot_id = self.robot_id
        goal_msg.position_list = positions
        goal_msg.velocity_list = velocities
        goal_msg.timestamp_list = timestamps
        
        # Ensure the action server is available
        if not self._trajectory_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Trajectory action server not available after waiting")
            return
        
        self.get_logger().info("Trajectory action server is available, sending goal.")
        
        send_goal_future = self._trajectory_action_client.send_goal_async(goal_msg)
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


    

    def plot_trajectories(self, actual_positions, actual_velocities):
        

        # Ensure all lists have the same length
        if len(self.timestamps) == len(self.positions) == len(actual_positions) and len(self.timestamps) == len(self.velocities) == len(actual_velocities):
            # Plot the desired and actual trajectories
            plt.figure(figsize=(12, 6))

            # Subplot for Position Comparison
            plt.subplot(2, 1, 1)
            plt.plot(self.timestamps, self.positions, 'r-', label='Desired Position')
            plt.plot(self.timestamps, actual_positions, 'b--', label='Actual Position')
            plt.title('Position Comparison')
            plt.xlabel('Time (s)')
            plt.ylabel('Position')
            plt.legend()
            plt.grid(True)

            # Subplot for Velocity Comparison
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
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
