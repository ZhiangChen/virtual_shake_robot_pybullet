#!/usr/bin/env python3
import os
import pandas as pd
import pickle
import numpy as np
from scipy.signal import butter, filtfilt
from ament_index_python.packages import get_package_share_directory

class DataLoader:
    def __init__(self, excel_file_name, folder_name, pickle_file_name):
        """
        Initializes the DataLoader, loading data from Excel and text files or from a cached pickle file.

        Args:
            excel_file_name (str): The name of the Excel file containing metadata.
            folder_name (str): The name of the folder containing the text data files.
            pickle_file_name (str): The name of the pickle file for cached combined data.

        Returns:
            None
        """
        # Get the package share directory
        package_share_directory = get_package_share_directory('virtual_shake_robot_pybullet')

        # Construct the paths relative to the package's data directory
        self.excel_file_path = os.path.join(package_share_directory, 'data', folder_name, excel_file_name)
        self.folder_path = os.path.join(package_share_directory, 'data', folder_name)
        self.pickle_file_path = os.path.join(package_share_directory, 'data', pickle_file_name)

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
        """
        Loads the Excel metadata file.

        Returns:
            pd.DataFrame: A DataFrame containing the Excel data.
        """
        df = pd.read_excel(self.excel_file_path, header=1)
        df.columns = df.columns.str.strip()  # Strip any leading/trailing spaces in column names
        return df

    def _load_txt_data(self, file_path):
        """
        Loads the time vs displacement data from a text file.

        Args:
            file_path (str): The path to the text file.

        Returns:
            pd.DataFrame: A DataFrame containing time and displacement data.
        """
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
        """
        Applies a Butterworth bandpass filter to the data.

        Args:
            data (list): The displacement data to filter.

        Returns:
            np.array: The filtered data.
        """
        nyquist = 0.5 * self.sampling_frequency
        low = 0.1 / nyquist
        high = 25 / nyquist
        b, a = butter(self.filter_order, [low, high], btype='band')
        filtered_data = filtfilt(b, a, data)
        # Bring displacement to zero at the start
        filtered_data = filtered_data - filtered_data[0]
        return filtered_data

    def _load_and_combine_data(self):
        """
        Combines data from Excel and corresponding text files.

        Returns:
            dict: A dictionary with combined data from the Excel and text files.
        """
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
        """
        Saves the combined data to a pickle file for faster future loading.

        Returns:
            None
        """
        with open(self.pickle_file_path, 'wb') as f:
            pickle.dump(self.combined_data, f)
        print(f"Data saved to pickle file: {self.pickle_file_path}")

    def _load_from_pickle(self):
        """
        Loads the combined data from a pickle file.

        Returns:
            dict: The combined data loaded from the pickle file.
        """
        with open(self.pickle_file_path, 'rb') as f:
            combined_data = pickle.load(f)
        print(f"Data loaded from pickle file: {self.pickle_file_path}")
        return combined_data

    def get_combined_data(self):
        """
        Retrieves the combined data.

        Returns:
            dict: The combined data.
        """
        return self.combined_data

def main():
    # Initialize the DataLoader with relative paths
    data_loader = DataLoader(
        excel_file_name='Earthquake Records Info.xlsx',
        folder_name='ASU_ Shared_ Scans/Shake_ Table_ Response',
        pickle_file_name='combined_data.pkl'
    )

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
