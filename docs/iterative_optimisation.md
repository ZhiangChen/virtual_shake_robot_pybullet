# Iterative Optimization for Virtual Shake Robot

## Overview
This script performs iterative optimization of simulation parameters for the Virtual Shake Robot using ROS2 and PyBullet. The objective is to determine the optimal values for various contact dynamics parameters by running simulations, analyzing results, and refining the parameters in multiple iterations.

## Process Flow
1. **Run the Simulation Experiment**
   - The generated launch files execute the simulation.
   - The simulation records results in `.npy` format.
   - All necessary `.npy` files are saved correctly for analysis.

2. **Analyze Results in `rock_analysis.py`**
   - The recorded `.npy` files are moved to the appropriate directory specified in the `rock_analysis.py` script.
   - Update the `recording_folder` path in `rock_analysis.py` to match the folder used when launching the simulation.
   - Run `rock_analysis.py` to process the `.npy` files.
   - The output of this analysis is a CSV file containing metrics such as accuracy, precision, and recall.

3. **Iterative Optimization**
   - The generated CSV file serves as input for the next iteration.
   - The script evaluates different values of the parameters using the objective function:
     
     \[Objective = w_1 \times Accuracy + w_2 \times Precision + w_3 \times Recall\]
     
     where the default weights are \(w_1 = 0.33\), \(w_2 = 0.33\), and \(w_3 = 0.33\).
   - The best-performing parameter value is selected for the next iteration.
   - This process repeats for each parameter in the optimization loop.
   - To continue to the next iteration, simply press **Enter** after updating the CSV file with the new results from `rock_analysis.py`.

4. **Results Storage**
   - Each iteration's results are stored in an Excel file (`optimization_results.xlsx`).
   - The Excel file logs:
     - Iteration number
     - Parameter values
     - Accuracy, precision, recall
     - Computed objective function value

## Running the Script
To execute the iterative optimization:
1. The ROS2 workspace and necessary dependencies are set up.
2. The script runs using:
   ```bash
   python iterative_optimization.py
   ```
3. After running the simulations, update the `recording_folder` path in `rock_analysis.py` and run it to process the `.npy` files.
4. The generated CSV file updates the optimization for further iterations. Press **Enter** to continue to the next iteration.

## File Structure
- **`data/`**: Stores generated YAML files, CSV files, and results.
- **`launch/`**: Stores dynamically generated launch files.
- **`rock_analysis.py`**: Analyzes `.npy` files to produce the CSV file used for optimization.
- **`optimization_results.xlsx`**: Logs all optimization iterations.

## Key Considerations
- The correct `.npy` files are recorded before running `rock_analysis.py`.
- The `recording_folder` path in `rock_analysis.py` is updated before running analysis.
- The latest CSV file generated from `rock_analysis.py` is always used for the next iteration.

This iterative approach refines simulation parameters, leading to optimal results for the Virtual Shake Robot.
