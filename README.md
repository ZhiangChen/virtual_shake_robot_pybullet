# virtual_shake_robot_pybullet
Virtual shake robot v2 based on PyBullet 

## Project  Overview
The Virtual Shake Robot (VSR) V2.0 project is a simulation tool designed to study the dynamics of Precariously Balanced Rocks (PBRs) during overturning and large-displacement processes. Utilizing the PyBullet physics engine, the VSR offers an approach to study the effects of physics parameters on the dynamics of PBRs, which help us understand seismic hazards, rockfall prediction, and the fate of toppled PBRs with realistic material properties and terrains. 

## Installatiion Guide

### Option 1. Manaul Installation
#### PyBullet
We use PyBullet as the physics engine. Please checkout the [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3), and install PyBullet. Before installing PyBullet, ensure you have the following prerequisites installed on your system:

- Python (3.6 or newer)
- pip (Python package installer)

1. Update pip
```
pip install --upgrade pip
```
2. Install PyBullet
```
pip install pybullet
```
3.  Verify the Installation
```
import pybullet as p
print(pb.__version__)
```

#### ROS2
we develop VSR 2.0 based ROS2. Detailed info on the integration and the structure can be found on [docs/VSR2_structure.md](docs/VSR2_structure.md).

Please follow the instruction to install ROS2 and create a ROS2 workspace `~\ros2_ws`: https://docs.ros.org/en/foxy/Installation.html

This repository should not have compatability issues with many ROS2 versions. We have developed the package based on ROS2 Humble. Here is the installation guide for our setup [ROS2_humble_installation](docs/ROS_installation.md)

#### virtual_shake_robot_pybullet
Clone the repository into your workspace: 
```
cd ~/ros2_ws/src
https://github.com/ZhiangChen/virtual_shake_robot_pybullet.git
```
Build the Package: 
```
cd ~/ros2_ws/src
colcon build --packages-select virtual_shake_robot_pybullet
```

Source the package in the current directory:
```
source ~/ros2_ws/install/setup.bash
```
 
### Option 2. Docker Installation


### Option 3. Apptainer Installation (for HPC)
If you need to run the parallel simulations on HPC with more computational power, you can manually set up the environment using Apptainer, [VSR2_Apptainer_installation](docs/Apptainer_installation.md). 


## Basic Usage

### Start simulation environment
We provide two options for the pedestal, a box with flat surfaces and a mesh file from realistic mapping. 

1. Box pedestal: 

```
ros2 launch virtual_shake_robot_pybullet box_launch.py
```

To configure the box pedestal, you can modify the yaml file, `config/vsr_structure_box.yaml`.

2. Mesh pedestal:
```
ros2 launch virtual_shake_robot_pybullet mesh_launch.py
```

To configure the mesh pedestal, you can modify the yaml file, `config/vsr_structure_mesh.yaml`.

Note: The physics parameters about the pedestal and world can be modified on `config/physics_parameters.yaml`, and the physics engine parameters about the simulation solvers can be modified on `config/physics_engine_parameters.yaml`. 

### Running the Control Node    

The control node accepts amplitude (A) and frequency (F) values to simulate a single-pulse cosine displacement ground motion of the pedestal. To trigger the motion, run:
 
```
ros2 run virtual_shake_robot_pybullet pulse_motion_client.py 2.0 1.0
```

In this case, the ground motion amplitude is 2.0 meters and the frequence is 1 Hz. The single-pulse cosine displacement motion is calculated using the following equations:
```
displacement: d = -A*cos(2*pi*F*t) + A
velocity: v = 2*pi*A*F*sin(2*pi*F*t)
acceleration: a = 4*pi^2*F^2*A*cos(2*pi*F*t)
```

For further details about the calculation, refer to the tutorial [inertia.md](docs/inertia.md).

### Spawning PBR on top of the pedestal 

Once we have a controller that can handle range of the A and F values, we are ready to spawn a PBR and carry out the experiment. VSR 2.0 supports two PBR options, a box PBR or a realistic PBR from a mesh file 


To spawn the PBR using a mesh file, you can utilize the command:

```
ros2 run virtual_shake_robot_pybullet pbr_loader.py mesh
```

Similarly for the box :

```
ros2 run virtual_shake_robot_pybullet pbr_loader.py box
```

With different arguments (`mesh` or `box`), the program loads a PBR from `config/pbr_mesh.yaml` or `config/pbr_box.yaml`, respectively. Users can modify these two files to costumize the PBR loading. 

Note: Inertia is an important physics parameter for PBR dynamics simulation. For more details on calculating inertia, refer to [inertia.md](docs/inertia.md). 

![Pbr mesh file on pedestal](docs/pbr.png)


### Running the Experiment in Different Modes

There are four distinct experiment modes, which can be triggered by setting the motion_mode parameter.

Single Cosine Mode: This mode applies a single cosine motion based on provided A and F values and is the default mode.

The implementation for this experiment is given above in the README.md

Grid Cosine Mode : 

This mode generates a range of A and F values to simulate continuous experiments over a range of parameters.

```
ros2 launch virtual_shake_robot_pybullet box_launch.py motion_mode:=grid_cosine
```
Single Recording Mode:

This mode runs experiments based on real data collected in the lab. Each test is conducted on the SP1 or SP2 mesh models, and the result (whether the rock toppled) is verified.

You can specify a particular test number:

To trigger this :

```
ros2 launch virtual_shake_robot_pybullet sp1_launch.py motion_mode:=single_recording test_no:=11
```

After this you also need to launch the perception_node.py where we record the trajectory data.

```
ros2 run virtual_shake_robot_pybullet perception_node.py

```

All Recording Mode :

This mode runs all the recorded tests consecutively.

```
ros2 launch virtual_shake_robot_pybullet sp1_launch.py motion_mode:=all_recordings
```
Similarly launch the perception_node.py




## Parallel Simulation

The parallel simulation feature allows you to test different combinations of parameters simultaneously by leveraging multiple namespaces. This is particularly useful for running multiple instances of the experiment concurrently, reducing the time it takes to evaluate a wide range of physics parameters and their effects on PBR dynamics.

## Namespaced Simulations

By defining each simulation under its own namespace, we eliminate the need to generate separate YAML files for each experiment. This ensures that each simulation runs independently, even though the parameters and results are stored in shared directories. The use of ROS2 launch files enables seamless namespace management for multiple simulations.

To set up the parallel simulation:

1. Generate YAML and Launch Files for Each Namespace:
 
 The script_generator.py automatically generates the required YAML files for each parameter combination and creates corresponding launch files under different namespaces.

```
ros2 run virtual_shake_robot_pybullet script_generator.py
```
This script will:

- Generate YAML files with combinations of physics parameters for each experiment.
- Create corresponding launch files for each generated YAML file.
- Additionally, create a master launch file (e.g., {mesh_file_name}_master_launch.py) that launches all simulations in parallel.`

2. Run the Master Launch File:

```
ros2 launch virtual_shake_robot_pybullet {mesh_file_name}_master_launch.py
```

3. Monitor the results

Each simulation will log its results in separate files, organized by namespace. This allows you to later compare the behaviors of PBRs across different parameter combinations.




## Memory Leak Issue
During the VSR simulations, a memory leakage issue was observed, especially in long-running or parallel experiments. The memory leak was caused by several factors, including:

- Data Recording: Continuous data logging, where trajectory data was recorded over time, contributed to increased memory usage.
- Creating and Deleting Plots: The callback function included the creation and deletion of plots, which were not properly managed, leading to memory leaks.
- Loading and Deleting URDF Models: Repeated loading and deletion of URDF models contributed to memory fragmentation and memory leaks.

To mitigate these issues, changes were made to the simulation node to handle the data recording more efficiently, as well as moving the plotting function to a separate script (plotter.py) to avoid creating plots inside the callback function. Additionally, improvements were made in the management of URDF models to ensure they are loaded and deleted correctly.


For more technical details on the memory leak issue, refer to the full report here.
[memory_Issue_report](docs/memory_issue.md)








 