# Running Parallel Simulations on a Server

If you need to run the parallel simulations on a server with more computational power, you can set up an Apptainer environment using a Dockerfile. Apptainer (formerly Singularity): Apptainer is more suited for shared server environments where you might not have root or sudo access, as it runs containers in a user space.

## Convert from docker container


1. Make sure that you have installed the docker container following [docs/docker_installation.md](docker_installation.md).

2. Convert the docker image to an Apptainer Image

```
apptainer build vsr_simulation.sif docker://vsr_simulation
```
3. Run the container

```
apptainer run vsr_simulation.sif
```
