#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from virtual_shake_robot_pybullet.srv import ManageModel
from virtual_shake_robot_pybullet.action import RecordingAction
import numpy as np
from geometry_msgs.msg import PoseStamped
import transforms3d.euler as euler
import matplotlib.pyplot as plt
from math import pi
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.get_logger().info("Initializing Perception Node...")

        # Set up subscription for pbr_pose
        self.pbr_pose_subscription = self.create_subscription(
            PoseStamped,
            'pbr_pose_topic',
            self.pbr_pose_callback,
            10
        )

        self.recording = False
        self.trajectory = []
        self.toppling_data = []
        self.latest_pose = None

        # Set up ActionServer for recording management
        self._action_server = ActionServer(
            self,
            RecordingAction,
            'manage_recording',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("Perception Node initialized.")

    def pbr_pose_callback(self, msg):
        self.latest_pose = msg.pose
        if self.recording:
            # Append the pose information to the trajectory
            time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            position = msg.pose.position
            orientation = msg.pose.orientation
            self.trajectory.append([
                time_stamp,
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            ])

    def goal_callback(self, goal_request):
        self.get_logger().info('Received recording management request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel recording')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        pga = goal_handle.request.pga
        pgv = goal_handle.request.pgv
        test_no = goal_handle.request.test_no
        namespace = self.get_namespace().split('/')[-1]

        self.get_logger().info(f"Received PGA: {pga}, PGV: {pgv}, Test No: {test_no}, Namespace: {namespace}")

        if goal_handle.request.command == 'start':
            self.start_recording()
        elif goal_handle.request.command == 'stop':
            self.stop_recording(pga, pgv, test_no, namespace)
        goal_handle.succeed()

        result = RecordingAction.Result()
        result.success = True
        return result

    def start_recording(self):
        self.recording = True
        self.trajectory = []
        self.get_logger().info("Recording started...")

    def stop_recording(self, pga, pgv, test_no, namespace):
        self.recording = False
        self.get_logger().info(f"Recording stopped. PGA: {pga}, PGV: {pgv}, Test No: {test_no}, Namespace: {namespace}")

        # Save the trajectory to a numpy file
        if self.trajectory:
            trajectory_np = np.array(self.trajectory)
            file_name = f"trajectory_{namespace}_test_{int(test_no)}.npy"
            recordings_folder = os.path.join(os.path.expanduser('~'), 'recordings', f"trajectory_{namespace}")
            os.makedirs(recordings_folder, exist_ok=True)
            file_path = os.path.join(recordings_folder, file_name)
            np.save(file_path, trajectory_np)
            self.get_logger().info(f"Recording saved to {file_path}")

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()
    rclpy.spin(perception_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
