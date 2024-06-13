import matplotlib
import sys
import ast

matplotlib.use("module://drawilleplot")
import matplotlib.pyplot as plt
import numpy as np

# Sample data for demonstration purposes. Replace these with your actual values.
# value = [
#     np.random.uniform(6 / 8, 5, 9),  # X coordinates
#     np.random.uniform(5, 6 / 8, 9),  # Y coordinates
#     np.random.uniform(-12, 12, 9),  # Z coordinates (not used in 2D plot)
# ]

# Define DEPTH_RANGE for the sensor
DEPTH_RANGE = [0.3, 12.0]  # Example range values
# Read sample data from a file if provided as an argument
# It will be in the following format
# R [0.2627943417975285, 0.8903173689955001, 0.0827894378731136, -0.36790945497635363, 0.4303650305912299, 0.5151043120186867, -0.4589361121023835, 0.827381385566705, 1.4402370718890722]|[0.2627943417975285, 0.8903173689955001, 0.0827894378731136, -0.36790945497635363, 0.4303650305912299, 0.5151043120186867, -0.4589361121023835, 0.827381385566705, 1.4402370718890722]|[0.2627943417975285, 0.8903173689955001, 0.0827894378731136, -0.36790945497635363, 0.4303650305912299, 0.5151043120186867, -0.4589361121023835, 0.827381385566705, 1.4402370718890722]|
# C MAV_CMD_CONDITION_YAW 8,32,30,5,27,54,51
# Only take the values that start with R
value = []
if len(sys.argv) > 1:
    for arg in sys.argv[1:]:
        # Open the file and read the data
        with open(arg, "r") as f:
            # Check if starts with R
            for line in f:
                if line.startswith("R"):
                    # Store the value as a array of floats
                    for val in line[2:-2].split(
                        "|"
                    ):  # Remove the R and the last character
                        value.append(ast.literal_eval(val))


# Calculate the 2D distances and angles
distances = np.sqrt(np.square(value[0]) + np.square(value[1]))
angles = np.arctan2(value[1], value[0])

# Normalize angles to be between 0 and 2*pi
angles = np.mod(angles, 2 * np.pi)

# Create a polar plot
fig, ax = plt.subplots(subplot_kw={"projection": "polar"})

# Plot the obstacle distances
ax.plot(angles, distances, "r-")

# Fill the area under the plot
ax.fill(angles, distances, "r", alpha=0.3)

# Set the title and labels
ax.set_title("OBSTACLE_DISTANCE_3D on a 360-degree Plane", va="bottom")
ax.set_theta_zero_location("N")  # Set 0 degrees to be at the top
ax.set_theta_direction(-1)  # Clockwise direction

# Set the radius limit based on sensor's depth range
ax.set_ylim(DEPTH_RANGE[0], DEPTH_RANGE[1])

# Display the plot
plt.show()

plt.close()
