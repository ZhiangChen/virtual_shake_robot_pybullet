import pybullet as p
import pybullet_data  # Import pybullet_data for access to built-in URDFs
import time
import psutil
import os
import matplotlib.pyplot as plt
from collections import deque

# Initialize PyBullet and set up physics
def setup_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the data path to pybullet_data
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)  # Disable real-time simulation for controlled step simulation

# Function to load the plane and SP1_PBRmodel URDF
def simulate_urdf(plane_path, sp1_urdf_path):
    plane = p.loadURDF(plane_path, [0, 0, 0])  # Load the plane at the origin
    model = p.loadURDF(sp1_urdf_path, [0, 0, 1])  # Load the model 1 meter above the plane
    return model, plane

# Monitor CPU and memory usage
def monitor_performance(cpu_usage, memory_usage, timestamps):
    pid = os.getpid()
    process = psutil.Process(pid)

    cpu = process.cpu_percent(interval=0.1)
    memory = process.memory_info().rss / (1024 * 1024)  # Memory in MB

    cpu_usage.append(cpu)
    memory_usage.append(memory)
    timestamps.append(time.time())

# Function to update the plot in real-time
def update_plot(cpu_usage, memory_usage, timestamps, start_time):
    plt.clf()  # Clear the plot
    time_elapsed = [t - start_time for t in timestamps]

    plt.subplot(2, 1, 1)
    plt.plot(time_elapsed, cpu_usage, label='CPU Usage (%)', color='r')
    plt.xlabel('Time (seconds)')
    plt.ylabel('CPU Usage (%)')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(time_elapsed, memory_usage, label='Memory Usage (MB)', color='b')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Memory Usage (MB)')
    plt.legend()

    plt.tight_layout()
    plt.pause(0.01)  # Pause to update the plot

# Main function to run the simulation in a loop with real-time plotting
def run_simulation(num_loops=1000, max_records=30, sp1_urdf_path="/home/akshay/ros2_ws/virtual_shake_robot_pybullet/models/SP1_PBRmodel/sp1.urdf"):
    setup_simulation()

    # Fixed length deques to store performance data
    cpu_usage = deque(maxlen=max_records)
    memory_usage = deque(maxlen=max_records)
    timestamps = deque(maxlen=max_records)

    plt.ion()  # Enable interactive mode for real-time plotting
    start_time = time.time()
    
    # Load the plane and your model
    plane_path = "plane.urdf"  # Use built-in plane URDF from pybullet_data
    model, plane = simulate_urdf(plane_path, sp1_urdf_path)

    for i in range(num_loops):
        for _ in range(240):  # Monitoring performance during simulation steps
            p.stepSimulation()
            monitor_performance(cpu_usage, memory_usage, timestamps)
            update_plot(cpu_usage, memory_usage, timestamps, start_time)
            

    # Clean up after the simulation
    p.removeBody(model)  # Remove the SP1_PBRmodel from the simulation
    p.removeBody(plane)  # Remove the plane
    p.disconnect()
    plt.ioff()  # Disable interactive mode
    plt.show()  # Final display of the plot

if __name__ == "__main__":
    run_simulation(num_loops=200, max_records=30)
