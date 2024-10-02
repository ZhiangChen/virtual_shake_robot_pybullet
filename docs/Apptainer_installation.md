# Running Parallel Simulations on a Server (Manual Setup)

If you need to run the parallel simulations on a server with more computational power, you can manually set up the environment using a base ROS2 Docker image or build your own Dockerfile. Here's a step-by-step guide:

## 1. Using a Pre-Built ROS2 Docker Image

Start by pulling a base ROS2 Docker image that includes the ROS2 Humble release:
```bash
docker pull osrf/ros:humble-desktop
```
Manual Setup:

Once you have the image, you will need to install the following dependencies manually:

    pip (for Python package management)
    pybullet
    Any other dependencies required by your project.


start the docker container :

```
docker run -it --name vsr_ros2_simulation osrf/ros:humble-desktop
```

This will start the container with the dependencies.

So now follow the README.md for the vsr setup


Apptainer (formerly Singularity): Apptainer is more suited for shared server environments where you might not have root or sudo access, as it runs containers in a user space.

- Convert the docker image to an Apptainer Image

```
apptainer build vsr_simulation.sif docker://vsr_simulation

```
 - Run the container

```
apptainer run vsr_simulation.sif
```

Follow Similar steps as the docker container
