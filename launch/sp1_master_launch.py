
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_0.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_1.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_2.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_3.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_4.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_5.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_6.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_7.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_8.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_9.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_10.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_11.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_12.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_13.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_14.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_15.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_16.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_17.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_18.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_19.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_20.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_21.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_22.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/launch/sp1_23.launch.py'))
        ),
        
    ])
    