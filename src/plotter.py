import numpy as np
import matplotlib.pyplot as plt

def load_trajectory(file_path):
    """Load the trajectory data from a .npy file."""
    trajectory_data = np.load(file_path)
    return trajectory_data

def plot_pedestal_trajectory(actual_trajectory, desired_trajectory, labels, title, color_actual, color_desired, linestyle_actual, linestyle_desired):
    """Plot the actual vs desired positions from the pedestal trajectory with improved visualization."""
    fig, ax = plt.subplots(figsize=(10, 8))

    time = actual_trajectory[:, 0]
    
    # Plot actual and desired positions with different colors and linestyles
    ax.plot(time, actual_trajectory[:, 1], label=labels[0], color=color_actual, linestyle=linestyle_actual, linewidth=2)  # Actual position (dotted or dashed)
    ax.plot(time, desired_trajectory[:, 1], label=labels[1], color=color_desired, linestyle=linestyle_desired, linewidth=2)  # Desired position (solid)

    # Customize the plot
    ax.set_title(title)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.legend()

    plt.tight_layout()
    plt.show()

def plot_pedestal_actual_comparison(trajectories, mode_labels, colors, linestyles):
    """Plot the actual positions from different modes on a single graph with improved line styles."""
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot actual positions for each mode
    for i, trajectory in enumerate(trajectories):
        time = trajectory[:, 0]
        ax.plot(time, trajectory[:, 1], label=mode_labels[i], color=colors[i], linestyle=linestyles[i], linewidth=2)

    # Customize the plot
    ax.set_title('Pedestal Actual Position Comparison (RTF On vs RTF Off vs GUI Off)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.legend()

    plt.tight_layout()
    plt.show()

def plot_pbr_position(trajectories, labels, mode_labels, colors, linestyles):
    """Plot the X, Y, Z positions of the PBR for multiple modes with distinct patterns."""
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))

    # Plot for each mode with different line styles and colors
    for i, trajectory in enumerate(trajectories):
        time = trajectory[:, 0]
        axs[0].plot(time, trajectory[:, 3], label=f'{mode_labels[i]} - {labels[0]}', color=colors[i], linestyle=linestyles[i], linewidth=2)
        axs[1].plot(time, trajectory[:, 4], label=f'{mode_labels[i]} - {labels[1]}', color=colors[i], linestyle=linestyles[i], linewidth=2)
        axs[2].plot(time, trajectory[:, 5], label=f'{mode_labels[i]} - {labels[2]}', color=colors[i], linestyle=linestyles[i], linewidth=2)

    axs[0].set_title('PBR X Position')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('X Position (m)')
    axs[0].legend()

    axs[1].set_title('PBR Y Position')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Y Position (m)')
    axs[1].legend()

    axs[2].set_title('PBR Z Position')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Z Position (m)')
    axs[2].legend()

    plt.tight_layout()
    plt.show()

def plot_pbr_orientation(trajectories, mode_labels, colors, linestyles):
    """Plot the quaternion orientations (Qx, Qy, Qz, Qw) of the PBR with distinct patterns."""
    fig, axs = plt.subplots(4, 1, figsize=(10, 10))

    for i, trajectory in enumerate(trajectories):
        time = trajectory[:, 0]
        axs[0].plot(time, trajectory[:, 6], label=f'{mode_labels[i]} - Qx', color=colors[i], linestyle=linestyles[i], linewidth=2)
        axs[1].plot(time, trajectory[:, 7], label=f'{mode_labels[i]} - Qy', color=colors[i], linestyle=linestyles[i], linewidth=2)
        axs[2].plot(time, trajectory[:, 8], label=f'{mode_labels[i]} - Qz', color=colors[i], linestyle=linestyles[i], linewidth=2)
        axs[3].plot(time, trajectory[:, 9], label=f'{mode_labels[i]} - Qw', color=colors[i], linestyle=linestyles[i], linewidth=2)

    axs[0].set_title('PBR Quaternion X (Qx)')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Qx')
    axs[0].legend()

    axs[1].set_title('PBR Quaternion Y (Qy)')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Qy')
    axs[1].legend()

    axs[2].set_title('PBR Quaternion Z (Qz)')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Qz')
    axs[2].legend()

    axs[3].set_title('PBR Quaternion W (Qw)')
    axs[3].set_xlabel('Time (s)')
    axs[3].set_ylabel('Qw')
    axs[3].legend()

    plt.tight_layout()
    plt.show()

def main():
    # Load trajectories for GUI On, RTF On; GUI On, RTF Off; and GUI Off, RTF Off
    gui_on_rtf_on_file = '/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/recordings_new/trajectory_sim_1_test_605.npy'
    gui_on_rtf_off_file = '/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/recordings_realtime_off/trajectory_sim_1_test_605.npy'
    gui_off_rtf_off_file = '/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/recordings_gui_off_rtf_off/trajectory_sim_1_test_605.npy'

    trajectory_gui_on_rtf_on = load_trajectory(gui_on_rtf_on_file)
    trajectory_gui_on_rtf_off = load_trajectory(gui_on_rtf_off_file)
    trajectory_gui_off_rtf_off = load_trajectory(gui_off_rtf_off_file)

    # List of trajectories for actual comparison
    actual_trajectories = [trajectory_gui_on_rtf_on, trajectory_gui_on_rtf_off, trajectory_gui_off_rtf_off]

    # Labels for the different modes
    mode_labels = ['GUI On, RTF On', 'GUI On, RTF Off', 'GUI Off, RTF Off']

    # Define colors and line styles for each mode
    colors = ['blue', 'green', 'red']
    linestyles = ['-', '--', ':']  # Solid, Dashed, Dotted

    # Plot GUI On, RTF On (actual vs desired) with improved visualization
    plot_pedestal_trajectory(trajectory_gui_on_rtf_on, trajectory_gui_on_rtf_on, ['Actual Position', 'Desired Position'], 
                             'GUI On, RTF On (Actual vs Desired)', 'blue', 'orange', ':', '-')

    # Plot GUI On, RTF Off (actual vs desired) with improved visualization
    plot_pedestal_trajectory(trajectory_gui_on_rtf_off, trajectory_gui_on_rtf_off, ['Actual Position', 'Desired Position'], 
                             'GUI On, RTF Off (Actual vs Desired)', 'green', 'purple', '--', '-')

    # Plot GUI Off, RTF Off (actual vs desired) with improved visualization
    plot_pedestal_trajectory(trajectory_gui_off_rtf_off, trajectory_gui_off_rtf_off, ['Actual Position', 'Desired Position'], 
                             'GUI Off, RTF Off (Actual vs Desired)', 'red', 'cyan', ':', '-')

    # Plot comparison of actual positions from all modes with improved patterns
    plot_pedestal_actual_comparison(actual_trajectories, mode_labels, colors, linestyles)

    # Plot PBR position (X, Y, Z) for all modes with improved patterns
    plot_pbr_position(actual_trajectories, ['PBR X Position', 'PBR Y Position', 'PBR Z Position'], mode_labels, colors, linestyles)

    # Plot PBR orientation (quaternion) for all modes with improved patterns
    plot_pbr_orientation(actual_trajectories, mode_labels, colors, linestyles)

if __name__ == "__main__":
    main()
