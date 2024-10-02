# ROS2 Installation Guide

This guide provides step-by-step instructions for installing ROS2 Humble on Ubuntu 22.04, which is compatible with the Virtual Shake Robot (VSR) project.

## 1. System Update
Before starting the installation, update your system to ensure all packages are up to date:

```bash
sudo apt update
sudo apt upgrade
```
## 2 .Add the ROS2 apt Repository

First, you need to add the ROS2 apt repository to your system:

```
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```
## 3. Install ROS2 Humble

Once the repository is set up, install ROS2 Humble Desktop:

```
sudo apt update
sudo apt install ros-humble-desktop
```
## 4. Source ROS2 Setup Script 

```
source /opt/ros/humble/setup.bash
```

## 5. Install Colcon for Building ROS2 Packages

```
sudo apt install python3-colcon-common-extensions
```

## 6. Verify the installation

```
ros2 --version

```

## 7. Create a ROS2 workshpace
```
mkdir -p ~/ros2_ws
```
