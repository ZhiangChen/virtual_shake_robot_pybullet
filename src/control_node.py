#!/usr/bin/env python3
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from virtual_shake_robot_pybullet.action import TrajectoryAction
import numpy as np

class ControlNode(Node):
    def __init__(self, amplitude, frequency):
        super().__init__('control_node')
        self.amplitude = amplitude
        self.frequency = frequency
        self.control_frequency = self.declare_parameter('simulation_node.engineSettings.ControlFrequency', 1000).value
        self._trajectory_action_client = ActionClient(self, TrajectoryAction, 'trajectory_action')
        self.get_logger().info(f"Initialized with Amplitude: {amplitude} and Frequency: {frequency}")

        # Calculate trajectory and send it
        self.calculate_and_send_trajectory()

    def calculate_and_send_trajectory(self):
        positions, velocities, timestamps = self.generate_trajectory()
        self.send_trajectory_goal(positions, velocities, timestamps)

    def generate_trajectory(self):
        """Generate position and velocity trajectory based on A and F."""
        T = 1.0 / self.frequency
        num_samples = int(self.control_frequency * T)
        t = np.linspace(0, T, num_samples)
        positions = -self.amplitude * np.cos(2 * np.pi * self.frequency * t) + self.amplitude
        velocities = 2 * np.pi * self.amplitude * self.frequency * np.sin(2 * np.pi * self.frequency * t)
        self.get_logger().info(f"Position list:{positions}, Velocity list: {velocities}")
        timestamps = t.tolist()
        positions = positions.tolist()
        velocities = velocities.tolist()
        return positions, velocities, timestamps

    def send_trajectory_goal(self, positions, velocities, timestamps):
        goal_msg = TrajectoryAction.Goal()
        goal_msg.position_list = positions
        goal_msg.velocity_list = velocities
        goal_msg.timestamp_list = timestamps
        
        # Ensure the action server is available
        if not self._trajectory_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available after waiting")
            return
        
        self.get_logger().info("Action server is available, sending goal.")
        
        
        send_goal_future = self._trajectory_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
        else:
            self.get_logger().info('Goal accepted by action server.')
            
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.final_result_callback)

    def final_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Action succeeded!")
        else:
            self.get_logger().error("Action failed!")


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 3:
        print("Usage: control_node.py <amplitude> <frequency>")
        return
    amplitude = float(sys.argv[1])
    frequency = float(sys.argv[2])
    node = ControlNode(amplitude, frequency)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
