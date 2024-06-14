# virtual_shake_robot_pybullet
Virtual shake robot v2 based on PyBullet for earthquake studies

# Project  Overview:
The Virtual Shake Robot (VSR) V2.0 project is a simulation tool designed to study the dynamics of Precariously Balanced Rocks (PBRs) during overturning and large-displacement processes. Utilizing the PyBullet physics engine, the VSR offers an approach to study the effects of physics parameters on the dynamics of PBRs, which help us understand seismic hazards, rockfall prediction, and the fate of toppled PBRs with realistic material properties and terrains. This project is developed with the integration of robotics and machine learning technologies, aiming to automate rock mapping and analysis effectively.
# Simulation Software:

## PyBullet ##
We use PyBullet Python bindings for improved support for robotics, reinforcement learning and VR. Use pip install pybullet and checkout the [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3).


# Installatiion Guide 
Before installing PyBullet, ensure you have the following prerequisites installed on your system:

- Python (3.6 or newer)
- pip (Python package installer)

## Steps ##

1 . Update pip
```
    pip install --upgrade pip
```
2 . Install PyBullet
```
    pip install pybullet
```
## Verifying the Installation ##
```
import pybullet as p
print(pb.__version__)

```
## Pybullet ROS2 Integration
 - Here we are using ROS2 since it has better dependenceies and the launch files in ROS2 provide more flexibility which can be very useful for the VSR 2.0

 For detailed info on the integration and the structure. Look [docs/VSR2_structure.md](docs/VSR2_structure.md)

 ## Building the Package 

 1 . Use the colcon package builder
 ```
 colcon build 

 ```
 2 . Source the package in the current directory

 ```
 source ~/install/setup.bash

 ```
 3 . Run the simulation_node.py for the pedestal mesh file using the launch file to launch the pybullet GUI for the VSR Structure.

 ```
 ros2 launch virtual_shake_robot_pybullet node.launch.py

 ```

 4 . For the box pedestal you need to launch the box_launch.py, which launches the control_node as well with it.

 ```
 ros2 launch virtual_shake_robot_pybullet box_launch.py

 ```
 5 . The action server in the control_node.py accpets Amplitude[A] and Frequency[F] values that are needed to determine the single_pulse consine motion for the pedestal

 ```
 ros2 run virtual_shake_robot_pytbullet 2.0 1.0

 ```
 This will send the A and F values for the motion of the pedestal to the control_node.py that generates the trajectory of the pedestal.

 Using the A and F values, we use this formula to calculate the target postions, velocity for the pedestal and the acceleration required for the pedestal


 ```
   pulse motion
        displacement function: d = -A*cos(2*pi*F*t) + A
        velocity function: v = 2*pi*A*F*sin(2*pi*F*t)
        acceleration function: a = 4*pi^2*F^2*A*cos(2*pi*F*t)
```
This values are calucalted and send as an goal using an action message for the Trajectory in the [TrajectoryAction.action](action/TrajectoryAction.action)


Then in the simulation_node these values are used in the built-in API of the pybullet GUI setJointMotorControl2 

For further details about the calculation refer the tutorial [Inertia.md](docs/Inertia.md)




 