import numpy as np
import math
import matplotlib.pyplot as plt
import argparse
import os

# Define a class for generating ground motion velocities.
class GroundMotion:
    def __init__(self, PGA, PGVA):
        """
        Initialize the ground motion with given PGA (in g) and PGVA.
        
        Args:
            PGA (float): Peak ground acceleration in units of g.
            PGVA (float): The ratio PGV/PGA.
        """
        self.PGA = PGA  # in g
        self.PGVA = PGVA
        
        # Convert PGA from g to m/s².
        self.PGA_m = self.PGA * 9.807
        
        # Calculate frequency and amplitude using:
        #   PGV = 2π*A*F and PGA = 4π²*F²*A, with PGV/PGA = PGVA.
        # From 1/(2πF) = PGVA, we get F = 1/(2π * PGVA).
        # And then A = PGA / (4π²*F²) which simplifies to A = PGA * PGVA².
        # Here, PGA used in amplitude calculation is PGA in m/s².
        self.frequency = 1 / (2 * math.pi * self.PGVA)
        self.amplitude = self.PGA_m * (self.PGVA ** 2)

    def generate_velocity(self, dt=0.005):
        """
        Generates a cosine-based velocity trajectory using a fixed timestep.
        
        Args:
            dt (float): The time step interval (in seconds).
        
        Returns:
            tuple: A tuple containing the list of velocity values and the corresponding timestamps.
        """
        # Compute the period T based on frequency.
        T = 1.0 / self.frequency
        # Create a time vector with a fixed step dt.
        t = np.arange(0, T, dt)
        # Calculate velocity: derivative of the cosine-based displacement.
        velocities = 2 * math.pi * self.amplitude * self.frequency * np.sin(2 * math.pi * self.frequency * t)
        return velocities.tolist(), t.tolist()

def write_velocity_file(PGA, PGVA, velocities, dt=0.005):
    """
    Writes velocity data to a tab-delimited file with a name based on PGA and PGVA.
    The first line contains the number of velocity points and the time step (dt),
    followed by one column of velocity values.
    """
    filename = f"pga_{PGA}_pgva_{PGVA}.tab"
    with open(filename, "w") as f:
        # Write header line: number of points and dt value.
        f.write(f"# number of velocity points: {len(velocities)}, dt: {dt}\n")
        # Write each velocity value on its own line.
        for vel in velocities:
            f.write(f"{vel:.6f}\n")
    print(f"File '{filename}' has been written.")

def plot_velocity_file(filename):
    """
    Reads a file with velocity values and plots the velocity vs. time.
    Assumes the first line of the file is a header with the number of points and dt.
    """
    # Check if file exists.
    if not os.path.exists(filename):
        print(f"File '{filename}' not found!")
        return

    velocities = []
    dt = 0.005  # Default dt value.
    with open(filename, "r") as f:
        # Read header and extract dt if needed.
        header = f.readline().strip()
        # Attempt to parse dt from the header.
        if "dt:" in header:
            try:
                dt_str = header.split("dt:")[1].strip()
                dt = float(dt_str)
            except Exception as e:
                print("Could not parse dt from header. Using default dt=0.005.")
        for line in f:
            if line.strip():
                velocities.append(float(line.strip()))
                
    # Create a time vector based on the number of velocity points and dt.
    t = [i * dt for i in range(len(velocities))]

    # Plot the velocity vs. time.
    plt.figure()
    plt.plot(t, velocities, marker="o", linestyle="-")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title(f"Velocity vs Time from '{filename}'")
    plt.grid(True)
    plt.show()

def generate_all_ground_motions():
    """
    Generates velocity files for a 3x3 grid of PGA and PGVA values.
    """
    # Define the grid values.
    PGA_values = [0.25, 0.50, 0.75]  # in g
    PGVA_values = [0.2, 0.4, 0.6]
    
    for PGA in PGA_values:
        for PGVA in PGVA_values:
            gm = GroundMotion(PGA, PGVA)
            velocities, _ = gm.generate_velocity(dt=0.005)
            write_velocity_file(PGA, PGVA, velocities, dt=0.005)
            print(f"Generated velocities for PGA={PGA}g, PGVA={PGVA} with frequency={gm.frequency:.4f} Hz and amplitude={gm.amplitude:.4f} m.\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate ground motion velocity files or plot an existing file.")
    parser.add_argument("--plot", type=str, help="Filename of the velocity file to plot")
    args = parser.parse_args()

    if args.plot:
        # If --plot option is provided, plot the provided file.
        plot_velocity_file(args.plot)
    else:
        # Otherwise, generate all ground motion velocity files.
        generate_all_ground_motions()
