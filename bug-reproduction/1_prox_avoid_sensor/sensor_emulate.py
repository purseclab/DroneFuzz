from typing import final
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import sys
import time
import datetime
import random
from pymavlink import mavutil
import os

sys.path.append("/usr/local/lib/")

# Connect to the UAV's MAVLink interface over UDP
conn = mavutil.mavlink_connection("localhost:1337", autoreconnect=True)

print("Connected to Mavlink interface")

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

# get time in correct format
start_time = int(round(time.time() * 1000))
# current_milli_time = lambda: int(round(time.time() * 1000) - start_time)
DEPTH_RANGE = [0.3, 12]  # depth range, to be changed as per requirements
MAX_DEPTH = 9999  # arbitrary large number

obstacle_bring = False

# Open a file to log the sensor values
f = open("sensor_values.txt", "w")


def current_milli_time():
    return int(round(time.time() * 1000) - start_time)


def random_normal_value():
    full_range = DEPTH_RANGE[1]
    # Uniformly send values
    x = y = z = [0.00] * 9
    for i in range(9):
        x[i] = random.uniform(0, full_range)
        y[i] = random.uniform(0, full_range)
        z[i] = random.uniform(0, full_range)
    return x, y, z


def random_attack_value():
    half_range = DEPTH_RANGE[1] / 2

    # Uniformly send values
    x = y = z = [0.00] * 9
    for i in range(9):
        x[i] = random.uniform(half_range / 8, 5)
        y[i] = random.uniform(5, half_range / 8)
        z[i] = DEPTH_RANGE[1]
    return x, y, z


# Function to send a simulated sensor input packet
def send_sensor_input(value):
    time = current_milli_time()
    for i in range(9):
        msg = mavlink2.MAVLink_obstacle_distance_3d_message(
            # conn.mav.obstacle_distance_3d_send(
            time,  # us Timestamp (UNIX time or time since system boot)
            0,  # not implemented in ArduPilot
            12,  # Set the frame to MAV_FRAME_BODY_FRD
            65535,  # unknown ID of the object. We are not really detecting the type of obstacle
            float(value[0][i]),  # X in NEU body frame
            float(value[1][i]),  # Y in NEU body frame
            float(value[2][i]),  # Z in NEU body frame
            float(DEPTH_RANGE[0]),  # min range of sensor
            float(DEPTH_RANGE[1]),  # max range of sensor
        )
        # print(msg)
        conn.mav.send(msg)


# Normal scenario
def normal_exec():
    itr = 0
    hz = 25
    while itr <= hz:
        vals = random_normal_value()
        if itr == 0:
            final_str = "R [" + str(vals[0]) + "]|["
            final_str += str(vals[1]) + "]|["
            final_str += str(vals[2]) + "]"
            final_str += "\n"
            f.write(final_str)
        send_sensor_input(vals)
        time.sleep(1 / hz)
        itr += 1


# Attack mode function
def obstacle_exec():
    itr = 0
    hz = 10
    while itr <= hz:
        vals = random_attack_value()
        if itr == 0:
            final_str = "R [" + str(vals[0]) + "]|["
            final_str += str(vals[1]) + "]|["
            final_str += str(vals[2]) + "]"
            final_str += "\n"
            f.write(final_str)
        send_sensor_input(vals)
        time.sleep(1 / hz)
        itr += 1


# Function to check if the copter is in the desired state
def is_copter_ready():
    msg = conn.recv_match(type="HEARTBEAT", blocking=True)
    is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    is_auto_mode = (
        msg.custom_mode == mavutil.mavlink.COPTER_MODE_LOITER
    )  # To check if we can detect in mission mode
    # Check if the copter is actually in the air and on a waypoint
    return is_armed and is_auto_mode


# Main loop
while True:
    normal_exec()
    if is_copter_ready():
        print("Copter is ready....")
        normal_exec()
        time.sleep(0.15)
        # if not obstacle_bring:
        print(f"{datetime.datetime.now()} Moving obstacle")
        f.write("Obstacle\n")
        obstacle_exec()
        # obstacle_bring = True
        # else:
        f.write("Done\n")

    time.sleep(0.25)
