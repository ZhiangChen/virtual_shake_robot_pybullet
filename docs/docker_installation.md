
# Virtual Shake Robot Setup with Docker and X11 Forwarding

### Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed on your system.
- A working internet connection to pull Docker images and dependencies.
- X11 server installed and configured for GUI-based applications. (For Linux users, this is typically installed by default; for Mac, consider using [XQuartz](https://www.xquartz.org/); for Windows, [VcXsrv](https://sourceforge.net/projects/vcxsrv/)).

### Step-by-Step Instructions

1. **Clone the Repository**

   Clone the repository to your local machine:

   ```bash
   git clone https://github.com/ZhiangChen/virtual_shake_robot_pybullet.git && cd virtual_shake_robot_pybullet
   ```

2. **Build the Docker Image**

   Inside the cloned repository, build the Docker image with the following command:

   ```bash
   docker build -t vsr_pybullet .
   ```

   This will create a Docker image named `vsr_pybullet`. The `Dockerfile` will install all the dependencies, including PyBullet and ROS2 Humble.

3. **Enable X11 Forwarding for GUI Applications**

   For GUI applications to work (e.g., viewing PyBullet's simulation environment), you need to enable X11 forwarding:

   - **Linux**:
     - Allow Docker to use your display by running:
       ```bash
       xhost +local:docker
       ```

   - **Mac (using XQuartz)**:
     - Start XQuartz from your Applications.
     - Allow connections from localhost by going to `XQuartz > Preferences`, navigating to the `Security` tab, and checking `Allow connections from network clients`.
     - Run the following in your terminal:
       ```bash
       xhost + 127.0.0.1
       ```

   - **Windows (using VcXsrv)**:
     - Start VcXsrv with the default configuration, making sure to enable the "Disable access control" option.

4. **Run the Docker Container**

   Now, run the Docker container with X11 forwarding:

   ```bash
   docker run -it --rm        --name vsr_pybullet_container        -e DISPLAY=$DISPLAY        -v /tmp/.X11-unix:/tmp/.X11-unix        vsr_pybullet
   ```

   - The `-e DISPLAY=$DISPLAY` flag ensures that the display is correctly forwarded to your host.
   - The `-v /tmp/.X11-unix:/tmp/.X11-unix` mounts the X11 socket to enable GUI support.

   This will start a container named `vsr_pybullet_container` from the `vsr_pybullet` image. The `-it` flag runs the container interactively, and the `--rm` flag ensures that the container is removed once you exit.

5. **Access the ROS2 Workspace**

   Once inside the Docker container, navigate to the ROS2 workspace and build the package:

   ```bash
   cd ~/ros2_ws/src/virtual_shake_robot_pybullet
   colcon build --packages-select virtual_shake_robot_pybullet
   ```

6. **Source the ROS2 Environment**

   Before running any ROS2 commands, source the workspace:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

7. **Run the Simulation**

   You can now run the Virtual Shake Robot simulations as you would on a native setup. For example:

   ```bash
   ros2 launch virtual_shake_robot_pybullet <launch_file>.launch.py
   ```

   Replace `<launch_file>` with the appropriate launch file name.

### Additional Notes

- If you encounter any display issues, make sure that your X11 server is running correctly and that the `xhost` settings are properly configured.
- This setup assumes that you are using the default display `:0`. If you are using a different display, adjust the `DISPLAY` environment variable accordingly.