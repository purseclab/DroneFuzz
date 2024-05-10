from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import sys
import time
import random
from pymavlink import mavutil
import os

sys.path.append("/usr/local/lib/")
import time

# Connect to the UAV's MAVLink interface over TCP
conn = mavutil.mavlink_connection('localhost:14551',source_system=1,source_component=93,baud=921600)

os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

# get time in correct format
start_time =  int(round(time.time() * 1000))
current_milli_time = lambda: int(round(time.time() * 1000) - start_time)
DEPTH_RANGE = [0.3, 12] # depth range, to be changed as per requirements
MAX_DEPTH = 9999 # arbitrary large number

obstacle_bring = False


def random_normal_value():
    full_range = DEPTH_RANGE[1]
    # Uniformly send values
    x = y = z = [0.00] * 9
    for i in range(9):
        x[i] = random.uniform(-full_range,full_range)
        y[i] = random.uniform(-full_range,full_range)
        z[i] = random.uniform(-full_range,full_range)
    return x,y,z

def random_attack_value():
    half_range = DEPTH_RANGE[1]/2

    # Uniformly send values
    x = y = z = [0.00] * 9
    for i in range(9):
        x[i] = random.uniform(-half_range/2,0)
        y[i] = random.uniform(0,half_range/2)
        z[i] = random.uniform(0,half_range)
    return x,y,z

# Function to send a simulated sensor input packet
def send_sensor_input(value):
    time = current_milli_time()
    for i in range(9):
        msg = mavlink2.MAVLink_obstacle_distance_3d_message(
        # conn.mav.obstacle_distance_3d_send(
            time,    # us Timestamp (UNIX time or time since system boot)
            0,       # not implemented in ArduPilot            
            12,      # Set the frame to MAV_FRAME_BODY_FRD
            65535,   # unknown ID of the object. We are not really detecting the type of obstacle           
            float(value[0][i]),	   # X in NEU body frame 
            float(value[1][i]),     # Y in NEU body frame  
            float(value[2][i]),	   # Z in NEU body frame  
            float(DEPTH_RANGE[0]), # min range of sensor
            float(DEPTH_RANGE[1])  # max range of sensor
        )
        conn.mav.send(msg)

# Normal scenario
def normal_exec():
    itr = 0
    hz = 25
    while(itr <= hz):
        send_sensor_input(random_normal_value())
        time.sleep(1/hz)
        itr += 1

# Attack mode function
def obstacle_exec():
    itr = 0
    hz = 10
    while(itr <= hz):
        send_sensor_input(random_attack_value())
        time.sleep(1/hz)
        itr += 1

# Function to check if the copter is in the desired state
def is_copter_ready():
    msg = conn.recv_match(type='HEARTBEAT', blocking=True)
    is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    is_loiter_mode = msg.custom_mode == mavutil.mavlink.COPTER_MODE_LOITER
    return is_armed and is_loiter_mode

# Main loop
while True:
    normal_exec()
    if is_copter_ready():
        print("Copter is ready....")
        normal_exec()
        time.sleep(0.15)
        if not obstacle_bring:
            print("Moving obstcale")
            obstacle_exec()
            obstacle_bring = True
        else:
            print("Already done")

    time.sleep(0.25)
