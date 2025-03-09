#!/usr/bin/env python3
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse

# Try to import cumtrapz and simps from scipy.integrate
try:
    from scipy.integrate import cumtrapz, simps
except ImportError:
    # Fallback for cumtrapz (simps is used in a loop below)
    def cumtrapz(y, x, initial=0):
        y = np.asarray(y)
        x = np.asarray(x)
        dt = np.diff(x)
        y_avg = (y[:-1] + y[1:]) / 2
        return np.concatenate(([initial], np.cumsum(dt * y_avg)))
    # Note: A proper simps fallback for cumulative integration is more involved;
    # here we assume SciPy is available for simps.

def compute_displacement_cumtrapz(timestamps, velocities):
    """
    Computes displacement using cumulative trapezoidal integration.
    
    Args:
        timestamps (list or array): Time values.
        velocities (list or array): Velocity values (m/s).
        
    Returns:
        tuple: (timestamps, cumulative displacement array)
    """
    ts = np.array(timestamps)
    vs = np.array(velocities)
    displacement = cumtrapz(vs, ts, initial=0)
    return ts, displacement

def compute_displacement_simps(timestamps, velocities):
    """
    Computes displacement using cumulative Simpson's rule integration.
    
    For each index i, Simpson's rule is applied to the sub-array ts[:i+1] and vs[:i+1].
    
    Args:
        timestamps (list or array): Time values.
        velocities (list or array): Velocity values (m/s).
        
    Returns:
        tuple: (timestamps, cumulative displacement array)
    """
    ts = np.array(timestamps)
    vs = np.array(velocities)
    cumulative_disp = [0]  # initial displacement is 0
    # Simpson's rule requires at least two intervals so we start from index 1.
    for i in range(1, len(ts)):
        disp = simps(vs[:i+1], ts[:i+1])
        cumulative_disp.append(disp)
    return ts, np.array(cumulative_disp)

def process_file(file_path, time_step=0.005):
    """
    Processes a single earthquake .tab file: reads the velocity data,
    generates timestamps, computes displacement using three methods,
    and plots the resulting displacement curves for comparison.
    
    Args:
        file_path (str): Full path to the .tab file.
        time_step (float): Time step between samples.
    """
    # Read the file skipping the first two header rows
    df = pd.read_csv(file_path, sep='\t', skiprows=2, header=None)
    velocities = df.iloc[:, 0].values.tolist()
    num_samples = len(velocities)
    
    # Use linspace to ensure exactly num_samples timestamps from 0 to (num_samples-1)*time_step
    timestamps = np.linspace(0, (num_samples - 1) * time_step, num_samples)
    print(f"Using time_step: {time_step} with {num_samples} samples")
    print(f"First 10 timestamps: {timestamps[:10]}")
    
    # Compute displacement using cumulative trapezoidal integration
    ts_trapz, disp_trapz = compute_displacement_cumtrapz(timestamps, velocities)
    # Compute displacement using cumulative Simpson's rule
    ts_simps, disp_simps = compute_displacement_simps(timestamps, velocities)
    # Compute displacement using a simple cumulative sum (Riemann sum)
    disp_cumsum = np.cumsum(velocities) * time_step

    # Plot the displacement curves for comparison
    plt.figure(figsize=(12, 8))
    plt.plot(ts_trapz, disp_trapz, 'r-', label="Cumtrapz (Trapezoidal)")
    plt.plot(ts_simps, disp_simps, 'b--', label="Cumulative Simpson's")
    plt.plot(timestamps, disp_cumsum, 'g:', label="Cumsum (Riemann Sum)")
    plt.xlabel("Time (s)")
    plt.ylabel("Displacement (m)")
    plt.title("Displacement Computation Comparison")
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Compare Displacement Integration Methods")
    parser.add_argument("--file", type=str,
                        help="Full path to a single .tab file to process")
    parser.add_argument("--time_step", type=float, default=0.005,
                        help="Time step between samples in seconds (default: 0.005)")
    args = parser.parse_args()

    if args.file:
        if not os.path.isfile(args.file):
            print(f"Error: The specified file does not exist: {args.file}")
            return
        print(f"Processing specified file: {args.file}")
        process_file(args.file, time_step=args.time_step)
    else:
        print("No file specified. Please use the --file argument to specify a .tab file.")
    
if __name__ == '__main__':
    main()
