#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
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

        # Set up subscriptions
        self.pbr_pose_subscription = self.create_subscription(
            PoseStamped,
            'pbr_pose_topic',
            self.pbr_pose_callback,
            10
        )
        self.pga_subscription = self.create_subscription(
            Float64,
            'pga_topic',
            self.pga_callback,
            10
        )
        self.pgv_subscription = self.create_subscription(
            Float64,
            'pgv_topic',
            self.pgv_callback,
            10
        )

        self.recording = False
        self.trajectory = []
        self.latest_pose = None
        self.latest_pga = None
        self.latest_pgv = None

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
            self.record_data()

    def pga_callback(self, msg):
        self.latest_pga = msg.data
        if self.recording:
            self.record_data()

    def pgv_callback(self, msg):
        self.latest_pgv = msg.data
        if self.recording:
            self.record_data()

    def record_data(self):
        if self.latest_pose and self.latest_pga is not None and self.latest_pgv is not None:
            time_stamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
            position = self.latest_pose.position
            orientation = self.latest_pose.orientation
            self.trajectory.append([
                time_stamp,
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w,
                self.latest_pga, self.latest_pgv
            ])

    def goal_callback(self, goal_request):
        self.get_logger().info('Received recording management request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel recording')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        if goal_handle.request.command == 'start':
            self.start_recording()
        elif goal_handle.request.command == 'stop':
            self.stop_recording()
        goal_handle.succeed()

        result = RecordingAction.Result()
        result.success = True
        return result

    def start_recording(self):
        self.recording = True
        self.trajectory = []
        self.get_logger().info("Recording started...")

    def stop_recording(self):
        self.recording = False
        self.get_logger().info("Recording stopped.")

        # Save the trajectory to a numpy file
        if self.trajectory:
            trajectory_np = np.array(self.trajectory)
            file_name = f"trajectory_{time.time()}.npy"
            recordings_folder = os.path.join(os.path.expanduser('~'), 'recordings')
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
