import numpy as np
import matplotlib.pyplot as plt

def load_trajectory(file_path):
    """Load the trajectory data from a .npy file and print the first few elements"""
    trajectory_data = np.load(file_path)
    
    # Print the first few elements (let's print first 5 rows as an example)
    print("First 5 elements of the trajectory data:")
    print(trajectory_data[:5])
    
    return trajectory_data

def plot_pedestal_trajectory(trajectory, labels):
    """Plot the X, Y, Z positions from the pedestal trajectory"""
    time = trajectory[:, 0]

    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    # Plot Target position
    axs[0].plot(time, trajectory[:, 1], label=labels[0])
    axs[0].set_title('Pedestal Target Position')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Position (m)')
    axs[0].legend()

    # Plot Actual position
    axs[1].plot(time, trajectory[:, 2], label=labels[1])
    axs[1].set_title('Pedestal Actual Position')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Position (m)')
    axs[1].legend()

    plt.tight_layout()
    plt.show()

def plot_pbr_position(trajectory, labels):
    """Plot the X, Y, Z positions of the PBR"""
    time = trajectory[:, 0]

    fig, axs = plt.subplots(3, 1, figsize=(10, 8))

    # Plot PBR X position
    axs[0].plot(time, trajectory[:, 3], label=labels[0])
    axs[0].set_title('PBR X Position')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('X Position (m)')
    axs[0].legend()

    # Plot PBR Y position
    axs[1].plot(time, trajectory[:, 4], label=labels[1])
    axs[1].set_title('PBR Y Position')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Y Position (m)')
    axs[1].legend()

    # Plot PBR Z position
    axs[2].plot(time, trajectory[:, 5], label=labels[2])
    axs[2].set_title('PBR Z Position')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Z Position (m)')
    axs[2].legend()

    plt.tight_layout()
    plt.show()

def main():
    # Load the trajectory from the .npy file
    trajectory_file = '/home/akshay/ros2_ws/virtual_shake_robot_pybullet/recordings/trajectory_sim_96_test_160.npy'
    trajectory = load_trajectory(trajectory_file)

    length = len(trajectory)
    print(f"Trajectory length: {length}")
    print(f"Trajectory shape: {trajectory.shape}")

    # Plot pedestal trajectory (target vs actual)
    labels_pedestal = ['Target Position', 'Actual Position']
    plot_pedestal_trajectory(trajectory, labels_pedestal)

    # Plot PBR position
    labels_pbr_position = ['PBR X Position', 'PBR Y Position', 'PBR Z Position']
    plot_pbr_position(trajectory, labels_pbr_position)

if __name__ == "__main__":
    main()
