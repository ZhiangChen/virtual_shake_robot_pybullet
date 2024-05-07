#!/usr/bin/env python3
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from virtual_shake_robot_pybullet.action import AF  
import numpy as np

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self._action_client = ActionClient(self, AF, 'execute_movement')
        self.get_logger().debug("Action client created.")

    def send_goal(self, a, f):
        goal_msg = AF.Goal()
        goal_msg.a = a
        goal_msg.f = f

        self.get_logger().debug("Waiting for action server...")
        self._action_client.wait_for_server()
        self.get_logger().debug("Action server available.")
        self.send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().debug("Goal sent.")

    def feedback_callback(self, feedback_msg):
        self.get_logger().debug("Feedback call Recieved!")
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Received feedback: {feedback}')

    def goal_response_callback(self, future):
        self.get_logger().debug("Goal response triggered")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().debug('Goal rejected :(')
            return

        self.get_logger().debug('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().debug('Result callback triggered')
        result = future.result().result
        if result.success:
            self.get_logger().debug('Goal succeeded!')
        else:
            self.get_logger().debug('Goal failed!')

   

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)  # Set log level to debug

    if len(sys.argv) == 3:
        a = float(sys.argv[1])
        f = float(sys.argv[2])
        print(f"received the values of {a} and {f}")
        node.send_goal(a, f)
    else:
        print("Usage: control_node.py <a> <f>")
        
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
