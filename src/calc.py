import os
import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt

# Constants for conversion and filtering
SCALE_FACTOR = 5.9797 * 10**-4  # Convert voltage to inches
INCH_TO_METER = 0.0254  # Convert inches to meters
FILTER_ORDER = 3  # Butterworth filter order
D_time = 0.00125 # delta_t in the time data

def apply_bandpass_filter(data, sampling_frequency, filter_order):
    """
    Applies a Butterworth bandpass filter to the data.
    """
    if sampling_frequency is None or np.isnan(sampling_frequency):
        raise ValueError("Invalid sampling frequency. Cannot apply bandpass filter.")

    nyquist = 0.5 * sampling_frequency
    low = 0.1 / nyquist  # Low cutoff frequency
    high = 25 / nyquist  # High cutoff frequency

    if low >= high or low <= 0 or high >= 1:
        raise ValueError(f"Invalid filter cutoff frequencies: low={low}, high={high}")

    b, a = butter(filter_order, [low, high], btype='band')
    filtered_data = filtfilt(b, a, data)
    # Bring displacement to zero at the start
    filtered_data = filtered_data - filtered_data[0]
    return filtered_data

def calculate_pgv_pga(time_data, displacement_voltage, apply_filter, sampling_frequency=None):
    """
    Calculates the Peak Ground Velocity (PGV) and Peak Ground Acceleration (PGA).
    """
    displacement_voltage = np.array(displacement_voltage, dtype=float)
    displacement_inches = displacement_voltage * SCALE_FACTOR

    if apply_filter:
        displacement_inches = apply_bandpass_filter(displacement_inches, sampling_frequency, FILTER_ORDER)

    displacement_meters = displacement_inches * INCH_TO_METER

    if len(time_data) < 2:
        raise ValueError("Insufficient time data for differentiation.")
    
    print(time_data)
    print(np.diff(time_data))
    print(np.mean(np.diff(time_data)))
    print("Indicate a new line")

    velocity = np.gradient(displacement_meters, D_time)
    acceleration = np.gradient(velocity, D_time)

    PGV = np.max(np.abs(velocity))
    PGA = np.max(np.abs(acceleration))

    return {"PGV": PGV, "PGA": PGA}

def process_all_tests(folder_path, reference_xlsx, output_csv):
    # Load reference data
    reference_data = pd.read_excel(reference_xlsx)
    reference_data.rename(columns={'Test Number': 'Test'}, inplace=True)

    # Convert PGA in reference data to g units
    reference_data['PGA (g)'] = reference_data['PGA']
    reference_data['PGVA_ref'] = reference_data['PGVA']

    results = []

    for test_no in range(11, 706):  # Test numbers range from 011 to 705
        file_name = f"Test{test_no:03d}.txt"
        file_path = os.path.join(folder_path, file_name)

        if not os.path.exists(file_path):
            print(f"File not found: {file_name}. Skipping...")
            continue

        try:
            data = pd.read_csv(file_path, sep='\s+', comment='#', header=None, skiprows=1, low_memory=False)

            time_data = pd.to_numeric(data.iloc[:, 4], errors='coerce').dropna().to_numpy(dtype=float)
            displacement_voltage = pd.to_numeric(data.iloc[:, 5], errors='coerce').dropna().to_numpy(dtype=float)

            if len(time_data) > 1:
                sampling_intervals = np.diff(time_data)
                sampling_frequency = 1 / np.mean(sampling_intervals)
            else:
                print(f"Invalid time data in {file_name}. Skipping...")
                continue
            

            # Calculate filtered PGV and PGA
            filtered_result = calculate_pgv_pga(time_data, displacement_voltage, apply_filter=True, sampling_frequency=sampling_frequency)
            filtered_pgv = filtered_result["PGV"]
            filtered_pga = filtered_result["PGA"]
            filtered_pga_g = filtered_result["PGA"] / 9.81 # Convert to g
            filtered_pgva = filtered_pgv / filtered_pga
            # Calculate unfiltered PGV and PGA
            unfiltered_result = calculate_pgv_pga(time_data, displacement_voltage, apply_filter=False, sampling_frequency=sampling_frequency)
            unfiltered_pgv = unfiltered_result["PGV"]
            unfiltered_pga = unfiltered_result["PGA"]
            unfiltered_pga_g = unfiltered_result["PGA"] / 9.81  # Convert tfiltered_result["PGV"]o g
            unfiltered_pgva = unfiltered_pgv / unfiltered_pga


            # Append results for this test
            results.append({
                "Serial Number": len(results) + 1,  # Assign serial numbers incrementally
                "Test": test_no,
                "PGA (g)": round(reference_data.loc[reference_data['Test'] == test_no, 'PGA (g)'].values[0], 4),
                "PGVA_ref": round(reference_data.loc[reference_data['Test'] == test_no, 'PGVA_ref'].values[0], 4),
                "PGA_unfiltered": round(unfiltered_pga_g, 4),
                "Error_PGA_unfiltered": round(abs(unfiltered_pga_g - reference_data.loc[reference_data['Test'] == test_no, 'PGA (g)'].values[0]), 4),
                "PGVA_unfiltered": round(unfiltered_pgva, 4),
                "Error_PGVA_unfiltered": round(abs(unfiltered_pgva - reference_data.loc[reference_data['Test'] == test_no, 'PGVA_ref'].values[0]), 4),
                "PGA_filtered": round(filtered_pga_g, 4),
                "Error_PGA_filtered": round(abs(filtered_pga_g - reference_data.loc[reference_data['Test'] == test_no, 'PGA (g)'].values[0]), 4),
                "PGVA_filtered": round(filtered_pgva, 4),
                "Error_PGVA_filtered": round(abs(filtered_pgva - reference_data.loc[reference_data['Test'] == test_no, 'PGVA_ref'].values[0]), 4)
            })

        except Exception as e:
            print(f"Error processing {file_name}: {e}. Skipping...")

    # Create DataFrame from results
    calculated_data = pd.DataFrame(results)

    # Reorder columns based on professor's request
    final_data = calculated_data[[
        "Serial Number",          # Serial Number
        "Test",                   # Test Number
        "PGA (g)",                # Reference PGA
        "PGVA_ref",               # Reference PGVA
        "PGA_unfiltered",         # Calculated PGA (unfiltered)
        "Error_PGA_unfiltered",   # Error for PGA (unfiltered)
        "PGVA_unfiltered",        # Calculated PGVA (unfiltered)
        "Error_PGVA_unfiltered",  # Error for PGVA (unfiltered)
        "PGA_filtered",           # Calculated PGA (filtered)
        "Error_PGA_filtered",     # Error for PGA (filtered)
        "PGVA_filtered",          # Calculated PGVA (filtered)
        "Error_PGVA_filtered"     # Error for PGVA (filtered)
    ]]

    # Save to CSV with the requested order
    final_data.to_csv(output_csv, index=False)
    print(f"Results saved to {output_csv}")

def main():
    folder_path = '/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/data/Shake_Table_Response'
    reference_xlsx = '/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/data/Shake_Table_Response/PGVA-zchen.xlsx'
    output_csv = '/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/data/PGVA_Error_Comparison.csv'

    if not os.path.exists(folder_path):
        raise FileNotFoundError(f"Folder not found: {folder_path}")

    process_all_tests(folder_path, reference_xlsx, output_csv)

if __name__ == '__main__':
    main()
