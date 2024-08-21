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
 ros2 launch virtual_shake_robot_pybullet mesh_launch.py

 ```

 4 . For the box pedestal you need to launch the box_launch.py, which launches the control_node as well with it.

 ```
 ros2 launch virtual_shake_robot_pybullet box_launch.py

 ```
 5 . The action server in the control_node.py accpets Amplitude[A] and Frequency[F] values that are needed to determine the single_pulse consine motion for the pedestal
```
 ros2 run virtual_shake_robot_pytbullet pulse_motion_client.py 2.0 1.0
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

## Spawning PBR on top of the pedestal 

Now we have a good controller that can handle range of the A and F values, so we are ready for the spawning of the PBR, and carrying out the experiment.

```
ros2 run virtual_shake_robot_pybullet pbr_loader.py [box|mesh]
```

To spawn the PBR using a mesh file, you can utilize the command:

```
ros2 run virtual_shake_robot_pybullet pbr_loader.py mesh
```

Similarly for the box :

```
ros2 run virtual_shake_robot_pybullet pbr_loader.py box
```

Both the properties for the mesh file and box are calucalted accurately the tutorial for calculating them from Fusion360 is present in [Inertia.md](docs/Inertia.md). Please refer that if you have any doubt for calculating the inetia values accurately for the mesh file(.obj).

![Pbr mesh file on pedestal](docs/pbr.png)


## Running the Experiment in Different Modes

We have four distinct experiment modes that can be triggered by setting the motion_mode parameter. These modes dictate how the pedestal behaves during the experiment:

Single Cosine Mode: This mode applies a single cosine motion based on provided A and F values and is the default mode.

The implementation for this experiment is given above in the README.md

Grid Cosine Mode : 

This mode is similar to the single_pulse_cosine motion but it generates a range of A and F values from the defined PGV and PGV/PGA range of values.

So after triggering this experiment the continous experiment will get triggered for the bunch of A and F values .

To trigger the experiment :

```
ros2 launch virtual_shake_robot_pybullet box_launch.py motion_mode:=grid_cosine
```
Single Recording Mode:

This mode is run on the experiment data, the actual data that we have collected our lab to verify the virtual environment.

This has the range of displacement  for the pedestal given in the file, there are multiple tests  conducted on the rock.

There are two mesh models sp1 and sp2 mesh models on which the experiment were conducted.


In this mode, you can send the particular test no to conduct, and check the output  that is whether the rock has topppled or not.

To trigger this :

```
ros2 launch virtual_shake_robot_pybullet sp1_launch.py motion_mode:=single_recording
```

After this you also need to launch the perception_node.py where we record the trajectory data.

```
ros2 run virtual_shake_robot_pybullet perception_node.py

```

All Recording Mode :

If you want to run all the tests from the start there is a method for that as well. It is similar to the single_recording_mode.

To beign all the tests

```
ros2 launch virtual_shake_robot_pybullet sp1_launch.py motion_mode:=all_recordings
```
Similarly launch the perception_node.py



















 