import os
import pandas as pd
import matplotlib.pyplot as plt

def load_and_process(file_paths):
    """
    Given a list of CSV file paths, load them into DataFrames,
    convert timestamp columns to datetime, concatenate, sort by 'end_time',
    and compute a cumulative finish time relative to the earliest finish time.
    Returns the processed DataFrame.
    """
    dfs = []
    for fp in file_paths:
        df = pd.read_csv(fp)
        df['start_time'] = pd.to_datetime(df['start_time'])
        df['end_time'] = pd.to_datetime(df['end_time'])
        dfs.append(df)
    # Combine all DataFrames
    df_combined = pd.concat(dfs, ignore_index=True)
    # Sort by finish time
    df_combined = df_combined.sort_values(by='end_time')
    # Compute cumulative finish time relative to the earliest finish time
    baseline = df_combined['end_time'].min()
    df_combined['cumulative_finish_time'] = (df_combined['end_time'] - baseline).dt.total_seconds()
    # Reset index to have a sequential test number for plotting
    df_combined = df_combined.reset_index(drop=True)
    df_combined['combined_test_no'] = df_combined.index
    return df_combined

# Define file paths for each simulation scenario

# Single-core CSV file
csv_file_single = "/home/akshay/Downloads/experiment_times_sim_1 (1).csv"

# 2-core simulation CSV files
csv_files_2core = [
    "/home/akshay/Downloads/experiment_times_sim_1.csv",
    "/home/akshay/Downloads/experiment_times_sim_2.csv"
]

# 4-core simulation CSV files (inside folder "exp_4_core")
csv_files_4core = [
    "/home/akshay/Downloads/exp_4_core/experiment_times_sim_1.csv",
    "/home/akshay/Downloads/exp_4_core/experiment_times_sim_2.csv",
    "/home/akshay/Downloads/exp_4_core/experiment_times_sim_3.csv",
    "/home/akshay/Downloads/exp_4_core/experiment_times_sim_4.csv"
]

# 8-core simulation CSV files (inside folder "exp_8_core")
csv_files_8core = [
    "/home/akshay/Downloads/exp_8_core/experiment_times_sim_1.csv",
    "/home/akshay/Downloads/exp_8_core/experiment_times_sim_2.csv",
    "/home/akshay/Downloads/exp_8_core/experiment_times_sim_3.csv",
    "/home/akshay/Downloads/exp_8_core/experiment_times_sim_4.csv",
    "/home/akshay/Downloads/exp_8_core/experiment_times_sim_5.csv",
    "/home/akshay/Downloads/exp_8_core/experiment_times_sim_6.csv",
    "/home/akshay/Downloads/exp_8_core/experiment_times_sim_7.csv",
    "/home/akshay/Downloads/exp_8_core/experiment_times_sim_8.csv"
]

# Load and process each scenario
df_single = load_and_process([csv_file_single])
df_2core = load_and_process(csv_files_2core)
df_4core = load_and_process(csv_files_4core)
df_8core = load_and_process(csv_files_8core)

# Plot all datasets on the same graph using lines only (without markers).
plt.figure(figsize=(12, 6))

plt.plot(df_single['combined_test_no'].to_numpy(),
         df_single['cumulative_finish_time'].to_numpy(),
         linestyle='-', color='red', label='Single-Core')

plt.plot(df_2core['combined_test_no'].to_numpy(),
         df_2core['cumulative_finish_time'].to_numpy(),
         linestyle='-', color='blue', label='2-Core')

plt.plot(df_4core['combined_test_no'].to_numpy(),
         df_4core['cumulative_finish_time'].to_numpy(),
         linestyle='-', color='green', label='4-Core')

plt.plot(df_8core['combined_test_no'].to_numpy(),
         df_8core['cumulative_finish_time'].to_numpy(),
         linestyle='-', color='purple', label='8-Core')

plt.xlabel("Test Number (sorted by finish time)")
plt.ylabel("Cumulative Finish Time (seconds)")
plt.title("Cumulative Finish Time Comparison: Single, 2-Core, 4-Core, 8-Core Simulation")
plt.grid(True)
plt.legend()
plt.show()
