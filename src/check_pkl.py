#!/usr/bin/env python3
import pickle
import argparse
import os

def print_summary(data, indent=0):
    """Recursively prints the structure of the parsed data."""
    prefix = " " * indent
    if isinstance(data, dict):
        for key, value in data.items():
            print(f"{prefix}{key}:")
            print_summary(value, indent=indent+4)
    elif isinstance(data, (list, tuple)):
        print(f"{prefix}List/Tuple with {len(data)} elements")
    else:
        # In our case, the leaf is a tuple of (timestamps, velocities)
        if isinstance(data, tuple) and len(data) == 2:
            timestamps, velocities = data
            print(f"{prefix}File with {len(timestamps)} samples")
        else:
            print(f"{prefix}{data}")

def main(pkl_file):
    if not os.path.isfile(pkl_file):
        print(f"Pickle file not found: {pkl_file}")
        return
    with open(pkl_file, "rb") as f:
        data = pickle.load(f)
    
    print("Parsed Earthquake Records structure:")
    print_summary(data)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check Pickle File Structure")
    parser.add_argument("pkl_file", type=str, help="Path to the pickle file")
    args = parser.parse_args()
    
    main(args.pkl_file)
