import re
import csv
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np

# Define the log file path
log_file_path = "fuzzing.log"

# Regular expression to match the log entries
log_pattern = re.compile(
    r"\[(?P<timestamp>.*?)\] \[Thread: .*?\] \[Process: .*?\] SMA pitch:(?P<pitch>[\d\.\-e]+) roll:(?P<roll>[\d\.\-e]+)"
)

# Prepare lists to hold the extracted data
timestamps = []
pitches = []
rolls = []

# Open the CSV file for writing
with open("log_data.csv", "w", newline="") as csvfile:
    fieldnames = ["timestamp", "pitch", "roll"]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()

    # Read the log file
    with open(log_file_path, "r") as logfile:
        for line in logfile:
            match = log_pattern.match(line)
            if match:
                # Extract the relevant data
                log_entry = match.groupdict()
                writer.writerow(log_entry)

                # Convert timestamp to datetime object
                timestamp = datetime.strptime(
                    log_entry["timestamp"], "%Y-%m-%d %H:%M:%S"
                )
                timestamps.append(timestamp)
                pitches.append(float(log_entry["pitch"]))
                rolls.append(float(log_entry["roll"]))

print("CSV file 'log_data.csv' created successfully.")

# Convert lists to numpy arrays for easier processing
timestamps = np.array(timestamps)
pitches = np.array(pitches)
rolls = np.array(rolls)


# Function to detect dips in data
def find_dips(data, threshold=0.01):
    dips = []
    diffs = np.diff(data)
    for i in range(1, len(diffs)):
        if diffs[i] < -threshold:
            dips.append(i)
    return dips


# Find dips in pitch and roll data
pitch_dips = find_dips(pitches, threshold=0.0005)
print(len(pitch_dips))
roll_dips = find_dips(rolls, threshold=0.0005)
print(len(roll_dips))

# Plot the data and highlight dips
plt.figure(figsize=(10, 5))

plt.plot(timestamps, pitches, label="Pitch")
plt.plot(timestamps, rolls, label="Roll")

# Highlight dips
plt.plot(timestamps[pitch_dips], pitches[pitch_dips], "ro", label="Pitch Dips")
plt.plot(timestamps[roll_dips], rolls[roll_dips], "go", label="Roll Dips")

# Format x-axis to show only the time
plt.gca().xaxis.set_major_formatter(plt.matplotlib.dates.DateFormatter("%H:%M:%S"))

plt.xlabel("Time")
plt.ylabel("Value")
plt.title("Pitch and Roll Over Time with Dips Highlighted")
plt.legend()
plt.grid(True)
plt.xticks(rotation=45)

plt.tight_layout()
plt.show()
