## How to Use the Output

### 1. Post-Simulation Analysis:
After running a simulation, you can load the `.npy` file using Python's NumPy library:

```python
import numpy as np

# Load the trajectory data
trajectory_data = np.load('path_to_output/trajectory_test_600.npy')

# Extract positions and poses
positions = trajectory_data[:, :3]  # x, y, z positions
poses = trajectory_data[:, 3:]      # orientation data
```

This allows you to analyze the recorded data for detailed insights into the motion of the PBR.

### 2. Comparing Simulations:
When using parallel simulations, you can compare the `.npy` files from different namespaces to evaluate how different physics parameters affect the stability and movement of the PBR.

### 3. Error Analysis:
The `.npy` data allows for error analysis between the **target** (desired) and **actual** positions. By plotting these differences, you can assess how closely the simulation followed the intended motion pattern.

## Example Visualization

The generated `.npy` file can be used to create plots of the PBR's motion over time. Below is an example of a simple visualization using Matplotlib:

```python
import numpy as np
import matplotlib.pyplot as plt

# Load the recorded trajectory
data = np.load('path_to_output/trajectory_test_600.npy')

# Extract time, x, y, z positions
time = data[:, 0]  # Assuming the first column represents time
x, y, z = data[:, 1], data[:, 2], data[:, 3]

# Plot the trajectory
plt.figure()
plt.plot(time, x, label='X Position')
plt.plot(time, y, label='Y Position')
plt.plot(time, z, label='Z Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('PBR Trajectory Over Time')
plt.legend()
plt.grid(True)
plt.show()
```

This code snippet will generate a plot of the PBR's position over time, allowing you to visualize how it moved during the simulation.
