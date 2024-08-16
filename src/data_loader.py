#!/usr/bin/env python3
import os
import pandas as pd
import pickle
import numpy as np
from scipy.signal import butter, filtfilt

class DataLoader:
    def __init__(self, excel_file_path, folder_path, pickle_file_path):
        self.excel_file_path = excel_file_path
        self.folder_path = folder_path
        self.pickle_file_path = pickle_file_path
        self.scale_factor = 5.9797 * 10**-4  # Scale factor to convert voltage to inches
        self.inch_to_meter = 0.0254  # Conversion factor from inches to meters
        self.sampling_interval = 0.001250  # Time step from the MATLAB code
        self.sampling_frequency = 1 / self.sampling_interval  # Sampling frequency
        self.filter_order = 3  # Filter order

        # Check if the pickle file exists
        if os.path.exists(self.pickle_file_path):
            self.combined_data = self._load_from_pickle()
        else:
            self.combined_data = self._load_and_combine_data()
            self._save_to_pickle()  # Save the data for future quick access

    def _load_excel_data(self):
        df = pd.read_excel(self.excel_file_path, header=1)
        df.columns = df.columns.str.strip()  # Strip any leading/trailing spaces in column names
        return df

    def _load_txt_data(self, file_path):
        """Load the time vs displacement data from the text file."""
        data = pd.read_csv(file_path, sep='\s+')

        # Extract time from the first column and displacement from the sixth column
        time_data = data.iloc[:, 0].tolist()  # Time is in the first column
        displacement_data = data.iloc[:, 5].tolist()  # Displacement is in the sixth column

        # Convert displacement from voltage to inches
        displacement_data = [d * self.scale_factor for d in displacement_data]

        # Apply bandpass filter
        displacement_data_filtered = self._apply_bandpass_filter(displacement_data)

        # Convert filtered displacement from inches to meters
        displacement_data_filtered_meters = [d * self.inch_to_meter for d in displacement_data_filtered]

        return pd.DataFrame({
            'Time': time_data,
            'Displacement': displacement_data_filtered_meters
        })

    def _apply_bandpass_filter(self, data):
        """Apply a Butterworth bandpass filter to the data."""
        nyquist = 0.5 * self.sampling_frequency
        low = 0.1 / nyquist
        high = 25 / nyquist
        b, a = butter(self.filter_order, [low, high], btype='band')
        filtered_data = filtfilt(b, a, data)
        # Bring displacement to zero at the start
        filtered_data = filtered_data - filtered_data[0]
        return filtered_data

    def _load_and_combine_data(self):
        """Combine data from Excel and corresponding text files."""
        combined_data = {}
        excel_df = self._load_excel_data()

        for index, row in excel_df.iterrows():
            test_no = row['Test No.']
            pgv_pga = row['PGV/PGA']
            scaled_pga = row['Scaled PGA (g)']

            txt_file_path = os.path.join(self.folder_path, f'Test{test_no:03d}.txt')

            if os.path.exists(txt_file_path):
                time_displacement_data = self._load_txt_data(txt_file_path)
                combined_data[test_no] = {
                    'PGV/PGA': pgv_pga,
                    'Scaled PGA': scaled_pga,
                    'Time vs Displacement': time_displacement_data
                }
            else:
                print(f"File not found: {txt_file_path}")

        return combined_data

    def _save_to_pickle(self):
        with open(self.pickle_file_path, 'wb') as f:
            pickle.dump(self.combined_data, f)
        print(f"Data saved to pickle file: {self.pickle_file_path}")

    def _load_from_pickle(self):
        with open(self.pickle_file_path, 'rb') as f:
            combined_data = pickle.load(f)
        print(f"Data loaded from pickle file: {self.pickle_file_path}")
        return combined_data

    def get_combined_data(self):
        return self.combined_data

def main():
    excel_file_path = '/home/akshay/ASU_ Shared_ Scans/Shake_ Table_ Response/Earthquake Records Info.xlsx'
    folder_path = '/home/akshay/ASU_ Shared_ Scans/Shake_ Table_ Response'
    pickle_file_path = 'combined_data.pkl'

    # Initialize DataLoader
    data_loader = DataLoader(excel_file_path, folder_path, pickle_file_path)

    # Retrieve combined data
    combined_data = data_loader.get_combined_data()

    # Print out the data for the first few test cases
    for test_no, data in combined_data.items():
        print(f"Test No: {test_no}")
        print(f"PGV/PGA: {data['PGV/PGA']}")
        print(f"Scaled PGA: {data['Scaled PGA']}")
        print(f"Time vs Displacement Data:\n{data['Time vs Displacement'].head()}\n")

if __name__ == '__main__':
    main()
