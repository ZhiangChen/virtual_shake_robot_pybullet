### Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed on your system.
- A working internet connection to pull Docker images and dependencies.

### Step-by-Step Instructions

1. **Clone the Repository**

   First, clone the repository to your local machine:

   ```
   git clone https://github.com/ZhiangChen/virtual_shake_robot_pybullet.git
   cd virtual_shake_robot_pybullet
   ```

2. **Build the Docker Image**

   Inside the cloned repository, you should have a \`Dockerfile\` that contains all the instructions to build the environment for the Virtual Shake Robot. Use the following command to build the Docker image:

   ```
   docker build -t vsr_pybullet .
   ```

   This will create a Docker image named \`vsr_pybullet\`. The \`Dockerfile\` will install all the dependencies, including PyBullet and ROS2 Humble.

3. **Run the Docker Container**

   Once the Docker image is built, you can run a container from it. Use the following command:

   ```
   docker run -it --rm --name vsr_pybullet_container vsr_pybullet
   ```

   This will start a container named \`vsr_pybullet_container\` from the \`vsr_pybullet\` image. The \`-it\` flag runs the container interactively, and the \`--rm\` flag ensures that the container is removed once you exit.

4. **Access the ROS2 Workspace**

   The ROS2 workspace is set up inside the Docker container. You can navigate to the workspace and build the package:

   ```
   cd ~/ros2_ws/src/virtual_shake_robot_pybullet
   colcon build --packages-select virtual_shake_robot_pybullet
   ```

5. **Source the ROS2 Environment**

   Before running any ROS2 commands, make sure to source the workspace:

   ```
   source ~/ros2_ws/install/setup.bash
   ```

6. **Run the Simulation**

   You can now run the Virtual Shake Robot simulations just like in a native setup.