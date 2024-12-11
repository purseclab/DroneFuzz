#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-License-Identifier: BSD-3-Clause or GPL-3.0-or-later
# SPDX-FileCopyrightText: (C) 2024 silipwn (Ashwin)
# Basically plot data via UDP and Mavlink 
# Need to export float data over 5005
# Also expose 1339 from AP
import matplotlib.pyplot as plt
from pymavlink import mavutil
import socket
import time
import threading

# Connect to the MAVLink stream
mavlink_connection = mavutil.mavlink_connection("udpin:localhost:1339")

# Set up UDP socket for the second message
UDP_IP = "127.0.0.1"  # localhost
UDP_PORT = 5005  # Choose an appropriate port
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind((UDP_IP, UDP_PORT))

# Initialize data arrays
timestamps1 = []
values1 = []
timestamps2 = []
values2 = []

# Set up the plot
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots(figsize=(10, 6))
(line1,) = ax.plot([], [], "b-", label="Yaw [Attitude]")
(line2,) = ax.plot([], [], "r-", label="Requested Yaw")

ax.set_xlabel("Time (s)")
ax.set_ylabel("Yaw (degrees)")
ax.set_title("Real-time Data over MAVLink")
ax.legend()

start_time = time.time()


# Function to receive UDP messages
def receive_udp():
    global timestamps2, values2
    while True:
        data, addr = udp_socket.recvfrom(1024)  # Buffer size is 1024 bytes
        current_time = time.time() - start_time
        value = float(data.decode())  # Assuming the UDP message is a float
        time.sleep(3)  # Simulate processing delay
        timestamps2.append(current_time)
        values2.append(value)


# Start UDP receiving thread
udp_thread = threading.Thread(target=receive_udp)
udp_thread.daemon = True
udp_thread.start()

# Main loop
try:
    while True:
        # Wait for MAVLink messages
        msg = mavlink_connection.recv_match(blocking=True)

        if msg is not None and msg.get_type() == "ATTITUDE":
            # Get current time
            current_time = time.time() - start_time

            # Extract value from the MAVLink message
            value1 = msg.yaw

            # Convert to degrees
            value1 = value1 * 180.0 / 3.14159

            # Append data to arrays
            timestamps1.append(current_time)
            values1.append(value1)

            # Update the plot lines
            line1.set_xdata(timestamps1)
            line1.set_ydata(values1)
            line2.set_xdata(timestamps2)
            line2.set_ydata(values2)

            # Adjust the plot limits
            ax.relim()
            ax.autoscale_view()

            # Redraw the plot
            plt.draw()
            plt.pause(0.01)

            # Optional: Limit data points to prevent memory issues
            max_points = 250
            if len(timestamps1) > max_points:
                timestamps1 = timestamps1[-max_points:]
                values1 = values1[-max_points:]
            if len(timestamps2) > max_points:
                timestamps2 = timestamps2[-max_points:]
                values2 = values2[-max_points:]

except KeyboardInterrupt:
    print("Exiting...")
finally:
    udp_socket.close()
