#!/usr/bin/env python3
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from virtual_shake_robot_pybullet.action import AF  

class AFActionClient(Node): 
    def __init__(self):
        super().__init__('af_action_client')  
        self._action_client = ActionClient(self, AF, 'set_amplitude_frequency')

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

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 3:
        print("Usage: amplitude_frequency_action_client.py <amplitude> <frequency>")
        return
    amplitude = float(sys.argv[1])
    frequency = float(sys.argv[2])

    action_client = AFActionClient()  # Updated to match the class name
    action_client.send_goal(amplitude, frequency)
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
