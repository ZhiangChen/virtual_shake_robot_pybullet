#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from virtual_shake_robot_pybullet.action import LoadPBR
import sys

class LoadPBRClient(Node):
    def __init__(self, structure_type):
        super().__init__('load_pbr_client')
        self.structure_type = structure_type
        self._action_client = ActionClient(self, LoadPBR, 'load_pbr_action')

    def send_goal(self, structure_name):
        goal_msg = LoadPBR.Goal()
        goal_msg.structure_name = structure_name
        goal_msg.structure_type = self.structure_type

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.status}')

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('PBR loading succeeded')
        else:
            self.get_logger().info('PBR loading failed')

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run your_package_name load_pbr.py [box|mesh]")
        return

    structure_type = sys.argv[1]

    if structure_type not in ['box', 'mesh']:
        print("Invalid argument. Use 'box' or 'mesh'.")
        return

    load_pbr_client = LoadPBRClient(structure_type)
    load_pbr_client.send_goal('rock')
    rclpy.spin(load_pbr_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
