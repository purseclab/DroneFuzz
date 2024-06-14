import sys
import ast
import matplotlib.pyplot as plt
import numpy as np

# Define DEPTH_RANGE for the sensor
DEPTH_RANGE = [0.3, 12.0]  # Example range values


# Function to calculate distances and angles
def calculate_distances_angles(data):
    distances = np.sqrt(np.square(data[0]) + np.square(data[1]))
    angles = np.arctan2(data[1], data[0])
    angles = np.mod(angles, 2 * np.pi)
    return distances, angles


# Initialize a list to store all datasets
datasets = []

# Read data from each file provided as argument
if len(sys.argv) > 1:
    for arg in sys.argv[1:]:
        with open(arg, "r") as f:
            for line in f:
                if line.startswith("R"):
                    value = []
                    for val in line[2:-2].split("|"):
                        value.append(ast.literal_eval(val))
                    datasets.append(value)

# Create a polar plot
fig, ax = plt.subplots(subplot_kw={"projection": "polar"})

# Define colors for each dataset
colors = ["b", "g", "r", "c", "m", "y", "k"]

# Plot each dataset
for i, data in enumerate(datasets):
    distances, angles = calculate_distances_angles(data)
    color = colors[i % len(colors)]  # Cycle through colors
    ax.plot(angles, distances, f"{color}-", label=f"Mutated point{i+1}")
    ax.fill(angles, distances, color, alpha=0.3)

# Set the title and labels
ax.set_title("OBSTACLE_DISTANCE_3D on a 360-degree Plane", va="bottom")
ax.set_theta_zero_location("N")  # Set 0 degrees to be at the top
ax.set_theta_direction(-1)  # Clockwise direction

# Set the radius limit based on sensor's depth range
ax.set_ylim(DEPTH_RANGE[0], DEPTH_RANGE[1])

# Add legend
ax.legend()

# Display the plot
plt.show()
