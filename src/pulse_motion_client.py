#!/usr/bin/env python3
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from virtual_shake_robot_pybullet.action import AF
import numpy as np
from math import pi
import matplotlib.pyplot as plt

class AFActionClient(Node):
    def __init__(self):
        super().__init__('af_action_client')
        self._action_client = ActionClient(self, AF, 'set_amplitude_frequency_manual')

    def send_goal(self, amplitude, frequency):
        goal_msg = AF.Goal()
        goal_msg.a = amplitude
        goal_msg.f = frequency

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed :(')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback}')

    def sampleMotionParam(self):
        PGA = np.linspace(0.1, 0.5, 3)
        PGV_2_PGA = np.linspace(0.1, 0.5, 3)
        Fs = 1.0 / (2 * pi * PGV_2_PGA)
        FA_data = []
        for F in Fs:
            for pga in PGA:
                A = 9.807 * pga / (4 * pi**2 * F**2)
                FA_data.append((F, A))
        return np.asarray(FA_data)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 3:
        print("Usage: amplitude_frequency_action_client.py <amplitude> <frequency>")
        return
    
    action_client = AFActionClient()
    
    # Generate and log the A and F values
    FA_data = action_client.sampleMotionParam()
    for F, A in FA_data:
        action_client.get_logger().info(f'Generated Frequency: {F}, Amplitude: {A}')
    
   
    # Send the command-line values as goals
    amplitude = float(sys.argv[1])
    frequency = float(sys.argv[2])
    action_client.send_goal(amplitude, frequency)

    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
