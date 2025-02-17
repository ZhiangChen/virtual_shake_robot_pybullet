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

Install PyBullet:
```
pip install --upgrade pip
pip install pybullet
```
Verify the Installation:
```
import pybullet as p
print(pb.__version__)
```

#### ROS2
We develop VSR 2.0 based ROS2. Detailed info on the integration and the structure can be found on [docs/VSR2_structure.md](docs/VSR2_structure.md).

Please follow the instruction to install ROS2 and create a ROS2 workspace `~\ros2_ws\src`: https://docs.ros.org/en/humble/Installation.html

This repository should not have compatability issues with many ROS2 versions. We have developed the package based on ROS2 Humble. Here is the installation guide for our setup [ROS2_humble_installation](docs/ROS_installation.md)

#### Dependencies

To install all the depencies, required to run this package we have a requirements.txt file which you can use to install them:

```
pip install -r requirements.txt

```

#### virtual_shake_robot_pybullet
Clone the repository into your workspace: 
```
cd ~/ros2_ws/src
https://github.com/ZhiangChen/virtual_shake_robot_pybullet.git
```
Build the Package: 
```
cd ~/ros2_ws
colcon build --packages-select virtual_shake_robot_pybullet
```

Source the package in the current directory:
```
source ~/ros2_ws/install/setup.bash
```
 
### Option 2. Docker Installation
Docker provides an isolated environment to run the VSR 2.0 without worrying about system dependencies. We have the steps to set up the VSR package using Docker in a tutorial, [Docker_Installtion](docs/docker_installation.md).


### Option 3. Apptainer Installation (for HPC)
If you need to run the parallel simulations on HPC with more computational power, you can set up the environment using Apptainer, [VSR2_Apptainer_installation](docs/Apptainer_installation.md). 


## Basic Usage

### Start simulation environment
We provide two options for the pedestal, a box with flat surfaces and a mesh file from realistic mapping. 

1. Box pedestal: 

```
ros2 launch virtual_shake_robot_pybullet box_launch.py
```

To configure the box pedestal, you can modify the yaml file, `config/vsr_structure_box.yaml`.

2. Mesh pedestal (in development):
```
ros2 launch virtual_shake_robot_pybullet mesh_launch.py
```

To configure the mesh pedestal, you can modify the yaml file, `config/vsr_structure_mesh.yaml`.

Note: The physics parameters about the pedestal and world can be modified on `config/physics_parameters.yaml`, and the physics engine parameters about the simulation solvers can be modified on `config/physics_engine_parameters.yaml`.


### Spawning PBR on top of the pedestal 

VSR 2.0 supports two PBR options, a box PBR or a realistic PBR from a mesh file.


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




## Advanced Usage

### Different Control Modes

VSR 2.0 supports four control modes for a box pedestal, which can be triggered by setting the `motion_mode` parameter in `control_node.py`.

1. **Single Cosine Mode**  
This mode applies a single cosine displacement motion based on provided A and F values and is the default mode.
```
ros2 launch virtual_shake_robot_pybullet box_launch.py
```

or 

```
ros2 launch virtual_shake_robot_pybullet box_launch.py motion_mode:=single_cosine
```


2. **Grid Cosine Mode** 

This mode generates a set of single cosine displacement motions based on a grid of PGA and PGV/PGA. The range of the grid is defined in the function `sample_motion_param` in `motion_node.py`.

```
ros2 launch virtual_shake_robot_pybullet box_launch.py motion_mode:=grid_cosine
```


3. **Single Recording Mode**

This mode runs experiments based on real data collected in the lab. Each test is conducted on the SP1 or SP2 mesh models.

You can specify a particular test number, `test_no` between 11 and 705:

```
ros2 launch virtual_shake_robot_pybullet sp1_launch.py motion_mode:=single_recording test_no:=600
```

4. **All Recording Mode**

This mode runs all the recorded tests consecutively.

```
ros2 launch virtual_shake_robot_pybullet sp1_launch.py motion_mode:=all_recordings
```

### Parallel Simulation

The parallel simulation feature allows you to test different combinations of parameters simultaneously by leveraging multiple namespaces. This is particularly useful for running multiple instances of the experiment concurrently, reducing the time it takes to evaluate a wide range of physics parameters and their effects on PBR dynamics.


To set up the parallel simulation:

1. Generate YAML and Launch files for parameter set:
 
 The script_generator.py automatically generates the required YAML files for each parameter combination and creates corresponding launch files under different namespaces.

```
ros2 launch virtual_shake_robot_pybullet script_generator.py
```
This script will:

- Generate YAML files with combinations of physics parameters for each experiment.
- Create corresponding launch files for each generated YAML file.
- Additionally, create a master launch file (e.g., {mesh_file_name}_master_launch.py) that launches all simulations in parallel.`

2. Run the master Launch file:

```
ros2 launch virtual_shake_robot_pybullet {mesh_file_name}_master_launch.py
```

Note that you need to `colcon build`  before running the launch file. 

3. Monitor the results

    Each simulation will log its results in separate files, organized by namespace. This allows you to later compare the behaviors of PBRs across different parameter combinations.



## Virtual Shake Robot Output Files

Regardless of the selected control mode or simulation type, the main output of the VSR simulations is an `.npy` file that records the detailed trajectory data. This file provides crucial insights into the dynamics of the PBR during the simulation.

### Content
The `.npy` file captures two key aspects of the simulation:
- **Actual Trajectory of the pedestal**: The position of the pedestal (x, y, z coordinates) recorded over the duration of the simulation. This data represents how the pedestal moved in response to the simulated ground motions or control inputs.
- **PBR Poses and orientation**: Includes detailed poses (position and orientation) of the PBR throughout the simulation, allowing for precise analysis of its stability and behavior under different conditions.

### File Naming
- For simulations that run single tests or experiments, the `.npy` files are named based on the test number or parameter set (e.g., `trajectory_test_600.npy` for test number 600).
- In parallel simulations, each `.npy` file is saved in a separate namespace to distinguish between different parameter combinations (e.g., `sim_96/trajectory.npy` for simulation 96).

### Analysis and Visualization
- Users can analyze the trajectory data using Python or other tools to study the dynamics of the PBR.
- The data can be plotted to compare the **actual vs. desired positions** of the pedestal, which helps to evaluate the accuracy of the simulation.
- A graph comparing actual and desired positions can be generated automatically if enabled, providing a visual representation of the Pedestal's movement relative to the expected motion.

For the output visualization, on how to use the .npy file we have a tutorial [Anaylsis.md](docs/analysis.md)

## Memory Leak Issue
During the VSR simulations, a memory leakage issue was observed, especially in long-running or parallel experiments. The memory leak was caused by several factors, including:

- Data Recording: Continuous data logging, where trajectory data was recorded over time, contributed to increased memory usage.
- Creating and Deleting Plots: The callback function included the creation and deletion of plots, which were not properly managed, leading to memory leaks.
- Loading and Deleting URDF Models: Repeated loading and deletion of URDF models contributed to memory fragmentation and memory leaks.

To mitigate these issues, changes were made to the simulation node to handle the data recording more efficiently, as well as moving the plotting function to a separate script (plotter.py) to avoid creating plots inside the callback function. Additionally, improvements were made in the management of URDF models to ensure they are loaded and deleted correctly.


For more technical details on the memory leak issue, refer to the full report here.
[memory_Issue_report](docs/memory_issue.md)


## Iterative Experiments on PBR

We wanted to do a parameter sweep on the parameters that we can tweak, so that we can have a clear which 
parameters yields us the highest accuracy when compared with the actual experiment data.

All the details about how the experiments  are conducted and the comparisons are done are in the 
[Iterative_Experiments](docs/Iterative_experiments.md)

Additonally we wanted to maximize the accuracy so we did iterative optimisation on the parameter set to find the best pair of the parameters.

Details about the script and results are in the [Iterative_Optimisation](docs/iterative_optimisation.md)








 
