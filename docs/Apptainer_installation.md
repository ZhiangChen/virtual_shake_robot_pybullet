# Running Parallel Simulations on a Server

If you need to run the parallel simulations on a server with more computational power, you can set up the environment using a base ROS2 Docker image or build your own Dockerfile. 

## Manual Setup

1. Start by pulling a base ROS2 Docker image that includes the ROS2 Humble release:
```bash
docker pull osrf/ros:humble-desktop
```
2. Once you have the image, you will need to install the following dependencies manually:

- pip (for Python package management)
- pybullet
- Any other dependencies required by your project.


3. Start the docker container :

```
docker run -it --name vsr_ros2_simulation osrf/ros:humble-desktop
```

## Convert from docker container

Apptainer (formerly Singularity): Apptainer is more suited for shared server environments where you might not have root or sudo access, as it runs containers in a user space.

1. Make sure that you have installed the docker container following [docs/docker_installation.md](docker_installation.md).

2. Convert the docker image to an Apptainer Image

```
apptainer build vsr_simulation.sif docker://vsr_simulation

```
3. Run the container

```
apptainer run vsr_simulation.sif
```
