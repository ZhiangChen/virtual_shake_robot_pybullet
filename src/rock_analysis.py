import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.lines as mlines
from transforms3d import euler
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score
import tkinter as tk
from pandastable import Table


class TopplingAnalysis:
    def __init__(self):
        # Dynamically resolve paths based on the workspace
        ros2_ws = os.getenv('ROS2_WS', default=os.path.expanduser('~/ros2_ws'))
        package_share_directory = os.path.join(ros2_ws, 'src', 'virtual_shake_robot_pybullet')

        # Set relative paths
        self.combined_pickle_path = os.path.join(package_share_directory, 'data', 'real_experiments_combined_data.pkl')
        self.npy_folder_path = os.path.join(package_share_directory, 'recordings', 'iterative_test_lat')
        self.ground_truth_path = os.path.join(package_share_directory, 'data', 'Results_Granite.xlsx')
        self.simulation_data_path = os.path.join(package_share_directory, 'data')
        self.simulation_parameters_path = os.path.join(package_share_directory, 'config', 'iteration_results_restitution.csv')

        # Ensure the simulation data path exists
        os.makedirs(self.simulation_data_path, exist_ok=True)

        # Load data
        self.combined_data = self._load_combined_data()
        self.ground_truth_data = self._load_ground_truth_data()

    def _load_combined_data(self):
        with open(self.combined_pickle_path, 'rb') as f:
            combined_data = pickle.load(f)
        return combined_data

    def _load_ground_truth_data(self):
        df = pd.read_excel(self.ground_truth_path)
        df = df[['Test Number', 'Experimental Results for response-Specimen 1']]
        df.columns = ['Test Number', 'Toppling Status']
        return df.set_index('Test Number')

    def _load_npy_data(self, sim_no, test_no):
        filename = f'trajectory_sim_{sim_no}_test_{test_no}.npy'
        file_path = os.path.join(self.npy_folder_path, filename)
        if not os.path.exists(file_path):
            return None
        return np.load(file_path)

    def check_toppled(self, orientation):
        w, x, y, z = orientation
        roll, pitch, _ = euler.quat2euler([w, x, y, z])
        return (abs(roll) + abs(pitch)) >= 0.1

    def analyze_all_simulations(self):
        metrics_list = []
        all_sim_data_list = []
        for sim_no in range(1, 25):
            sim_no_str = str(sim_no)
            actual_status = []
            predicted_status = []
            sim_data_list = []

            for test_no in sorted(self.ground_truth_data.index):
                if test_no not in self.combined_data:
                    continue

                npy_data = self._load_npy_data(sim_no_str, test_no)
                if npy_data is None or len(npy_data) == 0:
                    continue

                scaled_pga = self.combined_data[test_no].get('Scaled PGA', None)
                pgv_pga = self.combined_data[test_no].get('PGV/PGA', None)
                if scaled_pga is None or pgv_pga is None:
                    continue

                ground_truth_status = self.ground_truth_data.loc[test_no, 'Toppling Status']
                last_orientation = npy_data[-1, 6:10]
                toppled = self.check_toppled(last_orientation)
                predicted_status_value = 1 if toppled else 0

                actual_status.append(ground_truth_status)
                predicted_status.append(predicted_status_value)

                sim_data_list.append({
                    "Simulation Number": sim_no_str,
                    "Test Number": test_no,
                    "Scaled PGA": scaled_pga,
                    "PGV/PGA": pgv_pga,
                    "Toppling Status": predicted_status_value,
                    "Ground Truth Status": ground_truth_status
                })

            if actual_status and predicted_status:
                accuracy = accuracy_score(actual_status, predicted_status)
                precision = precision_score(actual_status, predicted_status, zero_division=0)
                recall = recall_score(actual_status, predicted_status, zero_division=0)
                f1 = f1_score(actual_status, predicted_status, zero_division=0)

                metrics_list.append({
                    "Simulation": sim_no_str,
                    "Accuracy": accuracy,
                    "Precision": precision,
                    "Recall": recall,
                    "F1 Score": f1
                })

                sim_df = pd.DataFrame(sim_data_list)
                csv_file_path = os.path.join(self.simulation_data_path, f'simulation_{sim_no_str}.csv')
                sim_df.to_csv(csv_file_path, index=False)
                print(f"Saved CSV for Simulation {sim_no_str}: {csv_file_path}")

                # Collect data for the combined CSV
                all_sim_data_list.extend(sim_data_list)

        # Save the combined simulation data to a single CSV file
        all_sim_data_df = pd.DataFrame(all_sim_data_list)
        combined_csv_file_path = os.path.join(self.simulation_data_path, 'combined_simulation_data.csv')
        all_sim_data_df.to_csv(combined_csv_file_path, index=False)
        print(f"Saved combined CSV file: {combined_csv_file_path}")

        # Load simulation parameters and merge with metrics
        if os.path.exists(self.simulation_parameters_path):
            parameters_df = pd.read_csv(self.simulation_parameters_path)
            parameters_df['Simulation'] = parameters_df['sim_no'].astype(str)
            metrics_df = pd.DataFrame(metrics_list)
            merged_metrics_df = pd.merge(parameters_df, metrics_df, on='Simulation', how='left')
            merged_metrics_df = merged_metrics_df.drop(columns=['Simulation'])
            metrics_csv_file_path = os.path.join(self.simulation_data_path, 'simulation_metrics.csv')
            merged_metrics_df.to_csv(metrics_csv_file_path, index=False)
            print(f"Saved metrics CSV file: {metrics_csv_file_path}")
            self.display_dataframe(merged_metrics_df)
        else:
            print(f"Simulation parameters CSV file not found: {self.simulation_parameters_path}")
            metrics_df = pd.DataFrame(metrics_list)
            self.display_dataframe(metrics_df)
            metrics_csv_file_path = os.path.join(self.simulation_data_path, 'simulation_metrics.csv')
            metrics_df.to_csv(metrics_csv_file_path, index=False)
            print(f"Saved metrics CSV file: {metrics_csv_file_path}")

        # Determine the best simulation number based on accuracy
        best_sim_no = metrics_df.loc[metrics_df['Recall'].idxmax(), 'Simulation']
        self.plot_graph(best_sim_no)
        self.plot_graph_analysis(best_sim_no)

    def display_dataframe(self, df):
        root = tk.Tk()
        root.title("Simulation Metrics")
        frame = tk.Frame(root)
        frame.pack(fill=tk.BOTH, expand=1)
        table = Table(frame, dataframe=df, showtoolbar=True, showstatusbar=True)
        table.show()
        root.mainloop()

    def _load_simulation_csv(self, sim_no):
        csv_file_path = os.path.join(self.simulation_data_path, f'simulation_{sim_no}.csv')
        if not os.path.exists(csv_file_path):
            print(f"CSV file not found: {csv_file_path}")
            return None
        return pd.read_csv(csv_file_path)

    def plot_graph(self, best_sim_no):
        sim_data = self._load_simulation_csv(best_sim_no)
        if sim_data is None:
            return

        x_vals_overturned = []
        y_vals_overturned = []
        x_vals_non_overturned = []
        y_vals_non_overturned = []

        for _, row in sim_data.iterrows():
            pga = row['Scaled PGA']
            pgv_pga = row['PGV/PGA']
            toppling_status = row['Toppling Status']

            if toppling_status == 1:  # Overturned
                x_vals_overturned.append(pga)
                y_vals_overturned.append(pgv_pga)
            else:  # Not overturned
                x_vals_non_overturned.append(pga)
                y_vals_non_overturned.append(pgv_pga)

        plt.figure(figsize=(8, 6))
        plt.scatter(x_vals_non_overturned, y_vals_non_overturned, color='black', alpha=0.8, label='No Overturn')
        plt.scatter(x_vals_overturned, y_vals_overturned, color='lightgray', alpha=0.8, label='Overturn')
        plt.xlabel("Scaled PGA (g)", fontsize=12, weight='bold')
        plt.ylabel("PGV/PGA (s)", fontsize=12, weight='bold')
        plt.title(f"(A) Overturn vs No Overturn - Best Simulation {best_sim_no}", fontsize=14, weight='bold')
        plt.xlim(0, 1)
        plt.ylim(0, 0.3)
        plt.legend(loc='upper right', fontsize=10, frameon=False)
        plt.grid()
        plt.show()

    def plot_graph_analysis(self, best_sim_no):
        sim_data = self._load_simulation_csv(best_sim_no)
        if sim_data is None:
            return

        # Extract values
        actual_status = sim_data['Ground Truth Status'].values
        predicted_status = sim_data['Toppling Status'].values

        tp_indices = []
        tn_indices = []
        fp_indices = []
        fn_indices = []

        for i, (actual, predicted) in enumerate(zip(actual_status, predicted_status)):
            if actual == 1 and predicted == 1:
                tp_indices.append(i)
            elif actual == 0 and predicted == 0:
                tn_indices.append(i)
            elif actual == 0 and predicted == 1:
                fp_indices.append(i)
            elif actual == 1 and predicted == 0:
                fn_indices.append(i)

        # Separate data for plotting
        x_tp = sim_data.iloc[tp_indices]['Scaled PGA']
        y_tp = sim_data.iloc[tp_indices]['PGV/PGA']

        x_tn = sim_data.iloc[tn_indices]['Scaled PGA']
        y_tn = sim_data.iloc[tn_indices]['PGV/PGA']

        x_fp = sim_data.iloc[fp_indices]['Scaled PGA']
        y_fp = sim_data.iloc[fp_indices]['PGV/PGA']

        x_fn = sim_data.iloc[fn_indices]['Scaled PGA']
        y_fn = sim_data.iloc[fn_indices]['PGV/PGA']

        # Plot data
        plt.figure(figsize=(8, 6))
        plt.scatter(x_tn, y_tn, color='#A9A9A9', marker='o', alpha=0.8, label='TN', s=10)  # TN (dark gray)
        plt.scatter(x_fp, y_fp, color='blue', marker='s', alpha=0.8, label='FP', s=40)  # FP
        plt.scatter(x_fn, y_fn, color='red', marker='^', alpha=0.8, label='FN', s=40)  # FN
        plt.scatter(x_tp, y_tp, color='gray', marker='o', alpha=0.8, label='TP', s=10)  # TP

        # Add legend
        legend_handles = [
            mlines.Line2D([], [], color='gray', marker='o', linestyle='None', markersize=8, label='TP'),
            mlines.Line2D([], [], color='#A9A9A9', marker='o', linestyle='None', markersize=8, label='TN'),
            mlines.Line2D([], [], color='red', marker='^', linestyle='None', markersize=8, label='FN'),
            mlines.Line2D([], [], color='blue', marker='s', linestyle='None', markersize=8, label='FP'),
        ]
        plt.legend(handles=legend_handles, loc='upper right', fontsize=10, frameon=False)

        # Set axis labels and title
        plt.xlabel("Scaled PGA (g)", fontsize=12, weight='bold')
        plt.ylabel("PGV/PGA (s)", fontsize=12, weight='bold')
        plt.title(f"(C) TP, TN, FP, FN - Simulation {best_sim_no}", fontsize=14, weight='bold')
        plt.xlim(0, 1)  # X-axis limit
        plt.ylim(0, 0.3)  # Y-axis limit
        plt.grid()

        # Show plot
        plt.show()
    
    def plot_analysis_with_best_or_specific_simulation(self, sim_no=None):
        """
        Plots the simulation data for either the best simulation (based on Recall) or a specific simulation number.

        :param sim_no: Optional. Specific simulation number to plot. If None, uses the best simulation based on Recall.
        """
        metrics_csv_file_path = os.path.join(self.simulation_data_path, 'simulation_metrics.csv')

        if not os.path.exists(metrics_csv_file_path):
            print(f"Metrics CSV file not found: {metrics_csv_file_path}")
            return

        metrics_df = pd.read_csv(metrics_csv_file_path)

        if sim_no is None:
            # Determine the best simulation number based on Recall
            sim_no = metrics_df.loc[metrics_df['Recall'].idxmax(), 'Simulation']
            print(f"Best simulation number based on Recall: {sim_no}")

        sim_data = self._load_simulation_csv(sim_no)
        if sim_data is None:
            print(f"Simulation data for Simulation {sim_no} not found.")
            return

        x_vals_overturned = []
        y_vals_overturned = []
        x_vals_non_overturned = []
        y_vals_non_overturned = []

        for _, row in sim_data.iterrows():
            pga = row['Scaled PGA']
            pgv_pga = row['PGV/PGA']
            toppling_status = row['Toppling Status']

            if toppling_status == 1:  # Overturned
                x_vals_overturned.append(pga)
                y_vals_overturned.append(pgv_pga)
            else:  # Not overturned
                x_vals_non_overturned.append(pga)
                y_vals_non_overturned.append(pgv_pga)

        # Plot data
        plt.figure(figsize=(8, 6))
        plt.scatter(x_vals_non_overturned, y_vals_non_overturned, color='black', alpha=0.8, label='No Overturn')
        plt.scatter(x_vals_overturned, y_vals_overturned, color='lightgray', alpha=0.8, label='Overturn')
        plt.xlabel("Scaled PGA (g)", fontsize=12, weight='bold')
        plt.ylabel("PGV/PGA (s)", fontsize=12, weight='bold')
        plt.title(f"Simulation Results - Simulation {sim_no}", fontsize=14, weight='bold')
        plt.xlim(0, 1)
        plt.ylim(0, 0.3)
        plt.legend(loc='upper right', fontsize=10, frameon=False)
        plt.grid()
        plt.show()


# Command-line argument handling
if __name__ == '__main__':
    analysis = TopplingAnalysis()
    analysis.analyze_all_simulations()
    analysis.plot_analysis_with_best_or_specific_simulation(sim_no=10)