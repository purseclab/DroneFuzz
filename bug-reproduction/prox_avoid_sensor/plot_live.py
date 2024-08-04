import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# File name
# NOTE: Make sure that you run this in the same folder as PGFUZZ.py
filename = "sma.log"

# Initialize empty lists for data
x_data, y_data = [], []

# Set up the figure and axis
fig, ax = plt.subplots()
(line1,) = ax.plot([], [], label="Column 1")
(line2,) = ax.plot([], [], label="Column 2")
ax.xaxis.set_tick_params(labelbottom=False)
ax.set_title("Pitch and Roll Over Time")
# ax.legend()


# Function to read data from file
def read_data():
    with open(filename, "r") as file:
        data = file.readlines()
    return [list(map(float, line.strip().split(","))) for line in data]


# Update function for the animation
def update(frame):
    data = read_data()
    x_data = [row[0] for row in data]
    y_data = [row[1] for row in data]

    line1.set_data(range(len(x_data)), x_data)
    line2.set_data(range(len(y_data)), y_data)

    # ax.relim()
    # ax.autoscale_view()
    # Limit the x-axis to last 10 points only
    ax.set_xlim(max(0, len(x_data) - 10), len(x_data))
    # Set limit for y-axis to 0.0015
    ax.set_ylim(0, 0.0015)

    return line1, line2


# Check if sma.log exists and has some data else just sleep
while True:
    try:
        with open(filename, "r") as file:
            data = file.readlines()
            if len(data) == 0:
                print("Nop, no data in the file, sleeping for 1 second")
                time.sleep(1)
            else:
                break
    except FileNotFoundError:
        print(f"File {filename} not found, sleeping for 1 second")
        time.sleep(1)

# Create the animation
ani = FuncAnimation(
    fig, update, frames=range(100), blit=False, interval=1000, repeat=True
)

plt.show()
