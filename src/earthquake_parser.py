#!/usr/bin/env python3
import os
import numpy as np
import argparse
import pickle  # <-- New import for pickle

def parse_all_earthquake_records(base_dir, time_step=0.005):
    """
    Parses the entire Earthquake_records directory structure by manually reading each file.

    Expected directory structure:
        base_dir/
          ├── Folder1 - PGVA X
          │     └── Subfolder (e.g., C02090)
          │            ├── Earthquake-0.2g.tab
          │            ├── Earthquake-0.3g.tab
          │            └── ...
          ├── Folder2 - PGVA Y
          │     └── Subfolder (e.g., C02111)
          │            └── ...
          └── ...

    For each .tab file:
      - The first two header lines are skipped.
      - The remaining lines are converted to floats (velocity values in m/s).
      - A corresponding timestamp array is generated using np.linspace.

    Args:
        base_dir (str): Path to the top-level Earthquake_records directory.
        time_step (float): Time step between samples in seconds (default 0.005 sec).

    Returns:
        dict: A nested dictionary in the form:
              {
                "Folder1 - PGVA X": {
                    "Subfolder": {
                        "Earthquake-0.2g.tab": ([t0, t1, ...], [v0, v1, ...]),
                        "Earthquake-0.3g.tab": ([t0, t1, ...], [v0, v1, ...]),
                        ...
                    }
                },
                ...
              }
    """
    all_data = {}

    # Loop over each top-level folder in the base directory.
    for top_folder in os.listdir(base_dir):
        top_folder_path = os.path.join(base_dir, top_folder)
        if not os.path.isdir(top_folder_path):
            continue  # Skip non-folders

        # Assume each top-level folder has exactly one subfolder (adjust if needed).
        subfolders = [d for d in os.listdir(top_folder_path)
                      if os.path.isdir(os.path.join(top_folder_path, d))]
        if not subfolders:
            continue

        subfolder_name = subfolders[0]
        subfolder_path = os.path.join(top_folder_path, subfolder_name)
        tab_data = {}

        # Process each .tab file within the subfolder.
        for file_name in os.listdir(subfolder_path):
            if file_name.endswith(".tab"):
                file_path = os.path.join(subfolder_path, file_name)
                print(f"DEBUG: Reading raw file: {file_path}")

                # Read the entire file.
                with open(file_path, 'r') as f:
                    raw_lines = f.readlines()

                # Print the first 10 lines to confirm the file structure.
                for i, line in enumerate(raw_lines[:10], 1):
                    print(f"Line {i}: {line.strip()}")

                # Skip the first two header lines (title and sample count/time step).
                data_lines = raw_lines[2:]
                velocities = []
                for line in data_lines:
                    line = line.strip()
                    if line:  # ignore empty lines
                        try:
                            velocities.append(float(line))
                        except Exception as e:
                            print(f"DEBUG: Error converting line to float: '{line}' | Error: {e}")
                            continue

                num_samples = len(velocities)
                # Generate timestamps: exactly one per velocity sample.
                timestamps = np.linspace(0, (num_samples - 1) * time_step, num_samples)

                
                tab_data[file_name] = (timestamps.tolist(), velocities)

        if tab_data:
            if top_folder not in all_data:
                all_data[top_folder] = {}
            all_data[top_folder][subfolder_name] = tab_data

    return all_data

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parse Earthquake Records")
    parser.add_argument("base_dir", help="Path to the top-level Earthquake_records directory")
    parser.add_argument("--time_step", type=float, default=0.005,
                        help="Time step between samples in seconds (default 0.005)")
    parser.add_argument("--pkl_file", type=str, default="earthquake_data.pkl",
                        help="Output pickle file to save the parsed data (default: earthquake_data.pkl)")
    args = parser.parse_args()

    data = parse_all_earthquake_records(args.base_dir, args.time_step)
    
    # Print a summary of parsed files.
    print("\nParsed earthquake records:")
    for top_folder, subfolders in data.items():
        print(f"Folder: {top_folder}")
        for subfolder, files in subfolders.items():
            print(f"  Subfolder: {subfolder}")
            for file, (timestamps, velocities) in files.items():
                print(f"    File: {file} -> {len(timestamps)} samples")
    
    # Save the parsed data to a pickle file.
    with open(args.pkl_file, "wb") as pkl_out:
        pickle.dump(data, pkl_out)
    print(f"\nData successfully saved to pickle file: {args.pkl_file}")
