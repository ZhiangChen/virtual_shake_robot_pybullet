# virtual_shake_robot_pybullet
Virtual shake robot v2 based on PyBullet for earthquake studies

# Project  Overview:
The Virtual Shake Robot (VSR) project is a sophisticated simulation tool designed to study the dynamics of Precariously Balanced Rocks (PBRs) during overturning and large-displacement processes. Utilizing the PyBullet physics engine, the VSR offers an innovative approach to understanding seismic hazards, rockfall prediction, and the fate of toppled PBRs with realistic material properties and terrains. This project is developed with the integration of advanced robotics and machine learning technologies, aiming to automate rock mapping and analysis effectively.

# Simulation Software:

## PyBullet ##
It is highly recommended to use PyBullet Python bindings for improved support for robotics, reinforcement learning and VR. Use pip install pybullet and checkout the [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3).


# Installatiion Guide 
Before installing PyBullet, ensure you have the following prerequisites installed on your system:

- Python (3.6 or newer)
- pip (Python package installer)

## Steps ##

1 . Update pip
```
    pip install --upgrade pip
```
2. Install PyBullet
```
    pip install pybullet
```
## Verifying the Installation ##
```
import pybullet as p
print(pb.__version__)

```
