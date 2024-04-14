# PyBullet ROS2 Architecture Overview



This project implements a robotics simulation framework using PyBullet and ROS2. The architecture is divided into three main nodes: Control, Perception, and Simulation, each with specific responsibilities. Additionally, it uses various configuration files to manage simulation parameters and robot specifications.

## Control Node

- Implements high-level control algorithms.
- Interacts with users to gather Planned Ground Velocity (PGV) and Planned Ground Acceleration (PGA) data or reads pre-defined acceleration-time series data from a file.
- Calculates and generates the desired trajectory for the simulation.
- Employs action servers to manage long-running processes, including trajectory planning and simulation task execution.

## Perception Node

- Listens for and subscribes to data topics published by the Simulation Node, focusing on the dynamic state of the simulation environment.
- Processes and analyzes simulation state data, potentially enhancing and republishing this information for use in detailed analysis or advanced visualization.
- Records experimental data, such as the status of Precariously Balanced Rock (PBR) and the trajectory followed during the simulation.

## Simulation Node

- Acts as the primary interface with the PyBullet simulation environment, operating as a dedicated PyBullet client.
- Receives and interprets trajectory data from the Control Node.
- Implements a PID controller to accurately follow the specified trajectory.
- Sends low-level control signals to the PyBullet server to guide the simulation process.
- Supports instantiation of multiple PyBullet clients in parallel for efficient execution of concurrent simulations.

## Launch File

- Initiates the system framework and establishes the ROS2 environment.
- Executes each node according to its specific configuration, ensuring seamless interaction and coordination.
- Manages the orchestration of node interactions for cohesive system operation.

## Configuration Files

- **Physics Parameter Files**: This file will contain the physics parameters managed by (`changeDynamics`), such as the coefficient of restitution and friction.
- **VSR Structure Parameter File**: Specifies the VSR (Virtual Shake Robot) structure, including geometric dimensions, mass distribution, inertia properties, and mesh file paths.
- **Simulation Parameter File**: This file will contain  the parameters that will control the Physics Engine Settings, through (`setPhysicsEngineParameter`).

This architecture facilitates detailed simulation and analysis of robotic systems, leveraging the power of ROS2 and PyBullet for comprehensive, realistic modeling.
