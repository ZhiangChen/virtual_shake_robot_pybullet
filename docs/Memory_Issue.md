## Overview

In the Virtual Shake Robot (VSR) simulation, we have experienced memory leakage, especially during long-running or parallel simulations. This was mainly caused by the continuous recording of simulation data (such as positions and PBR poses) over time, leading to increased memory usage. To address this, we implemented an improved setup to manage memory more effectively, by integrating the recording process directly into the simulation_node.py instead of using the perception node.

## New Setup to Prevent Memory Leakage

To avoid memory leakage, we have made the following key changes:

- Recording Process in Simulation Node: The perception node is no longer responsible for recording trajectory data. Instead, the recording is now integrated into the simulation node's execute_position_trajectory_callback function. This reduces inter-node communication overhead and improves memory management.

Recording process in the simulation node:

```
np.save('trajectory_data.npy', np.array(simulation_data))
del simulation_data
gc.collect()

```
 - Manual Memory management:After saving data, we explicitly delete the list storing the data and invoke garbage collection (gc.collect()) to free up memory immediately.

 - Manual Cleanup:

    After saving data (e.g., .npy files), explicitly delete lists or other large variables to free up memory.
    Force garbage collection after data-saving steps using:

    ```
    import gc
    gc.collect()

    ```
- Separate Plotting Script: Previously, the plotting function was embedded within the execute_position_trajectory_callback, which contributed to memory leakage due to unclosed figures. To resolve this, we moved the plotting functionality to a separate script called plotter.py. This script can be used to compare the actual vs. desired trajectory after the simulation has been completed, freeing the simulation node from the memory overhead of handling plots.

You can run plotter.py to visualize and analyze the results once the simulation is finished.

- Memory Profiling: We recommend using memory profiling tools like tracemalloc or objgraph to monitor memory usage in long-running simulations.




