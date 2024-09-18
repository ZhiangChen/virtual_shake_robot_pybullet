# Use the official ROS2 Humble base image
FROM osrf/ros:humble-desktop

# Set environment variables for ROS2 and Python
ENV ROS_DISTRO humble
ENV DEBIAN_FRONTEND noninteractive

# Install Python, pip, screen, and other required dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    screen \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies, including PyBullet
RUN pip3 install --upgrade pip && \
    pip3 install pybullet

# Clone the Virtual Shake Robot repository
WORKDIR /home/ros2_ws/src
RUN git clone https://github.com/Akshay6077/virtual_shake_robot_pybullet.git

# Install ROS2 and build the workspace
WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source the workspace in the bashrc for easy access
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

# Expose the necessary ports
EXPOSE 11311

# Set the entrypoint to start a bash shell
ENTRYPOINT ["/bin/bash"]
