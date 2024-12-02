import pickle
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Define the file paths
combine_data_path = '/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/data/combined_data.pkl'  
excel_file_path = '/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/data/Results_Granite.xlsx'  

def load_combine_data(pickle_file):
    """
    Load the combined data from the pickle file.
    
    Args:
        pickle_file (str): Path to the pickle file.
        
    Returns:
        dict: Combined data loaded from the pickle file.
    """
    with open(pickle_file, 'rb') as f:
        combined_data = pickle.load(f)
    return combined_data

def plot_single_pga_pgv_vs_toppling_status(pickle_file, ground_truth_file, num_tests_to_plot=100):
    """
    Loads the test data and toppling status from the Excel file and combined data from the pickle file,
    and plots PGA/PGV vs toppling status in a format similar to the provided example image.
    
    Args:
        pickle_file (str): The path to the pickle file containing combined data.
        ground_truth_file (str): The path to the Excel file containing the test numbers and toppling status.
        num_tests_to_plot (int): The number of tests to plot (default is 100).
    """
    # Load the combined data from pickle
    combined_data = load_combine_data(pickle_file)
    
    # Load the ground truth data from Excel
    df = pd.read_excel(ground_truth_file)

    # Extract test numbers and toppling status, limiting to the number of tests to plot
    df = df.head(num_tests_to_plot)  # Limit the number of tests
    test_numbers = df['Test Number'].tolist()
    topple_status = df.iloc[:, 1].tolist()

    # Extract PGA/PGV for each test number from combined_data and plot the status
    pga_values = []
    pgv_to_pga_values = []
    
    for test_no in test_numbers:
        if test_no in combined_data:
            pga_values.append(combined_data[test_no]['Scaled PGA'])
            pgv_to_pga_values.append(combined_data[test_no]['PGV/PGA'])
        else:
            pga_values.append(None)
            pgv_to_pga_values.append(None)
    
    # Create the plot
    plt.figure(figsize=(8, 6))

    # Adjust font properties for titles, axis, and legends
    font_properties = {'fontsize': 12, 'weight': 'bold'}
    
    # Scatter plot: Toppled vs Not Toppled
    overturn_plotted = False
    no_overturn_plotted = False
    
    for i, test_no in enumerate(test_numbers):
        if pga_values[i] is not None and pgv_to_pga_values[i] is not None:
            if topple_status[i] == 1:  # Overturn (light gray)
                if not overturn_plotted:  # Plot label only once
                    plt.scatter(pga_values[i], pgv_to_pga_values[i], color='lightgray', marker='o', s=30, alpha=0.8, label='Overturn')
                    overturn_plotted = True
                else:
                    plt.scatter(pga_values[i], pgv_to_pga_values[i], color='lightgray', marker='o', s=30, alpha=0.8)
            else:  # No overturn (dark gray)
                if not no_overturn_plotted:  # Plot label only once
                    plt.scatter(pga_values[i], pgv_to_pga_values[i], color='black', marker='o', s=30, alpha=0.8, label='No Overturn')
                    no_overturn_plotted = True
                else:
                    plt.scatter(pga_values[i], pgv_to_pga_values[i], color='black', marker='o', s=30, alpha=0.8)

    # Set axis limits
    plt.xlim(0, 1)  # PGA (g) goes from 0 to 1
    plt.ylim(0, 0.3)  # PGV/PGA (s) goes from 0 to 0.3

    # Titles, labels, and grid
    plt.title('(A) Overturn vs No Overturn', fontsize=14, weight='bold')
    plt.xlabel('PGA (g)', fontsize=12, weight='bold')
    plt.ylabel('PGV/PGA (s)', fontsize=12, weight='bold')
    plt.grid(True)
    plt.legend(loc='upper right', fontsize=10, frameon=False)

    # Final adjustments for consistent layout and show plot
    plt.tight_layout()
    plt.show()

# Call the function to plot the graph for the first 100 tests
plot_single_pga_pgv_vs_toppling_status(combine_data_path, excel_file_path, num_tests_to_plot=700)
