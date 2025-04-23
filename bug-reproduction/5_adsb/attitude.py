from pymavlink import mavutil
import time
import sys
from math import radians

vehicle = mavutil.mavlink_connection("localhost:14551")
vehicle.wait_heartbeat()

def wait_for_gps_fix():
    """Wait for a GPS fix."""
    print("Waiting for GPS fix...")
    while True:
        msg = vehicle.recv_match(type="STATUSTEXT", blocking=True)
        if "is using GPS" in msg.text:
            print("GPS obtained")
            time.sleep(1)
            break

def set_attitude():
    """Set vehicle attitude."""
    mask = 7
    q0 = q1 = q2 = q3 = 0.1
    att_target = [q0, q1, q2, q3]
    thrust = 1.0
    roll_rate = 0.0
    pitch_rate = 0.0
    yaw_rate = 0.0
    vehicle.mav.set_attitude_target_send(
        0,  # system time in milliseconds
        0,  # target system
        0,  # target component
        mask,  # type mask
        att_target,  # quaternion attitude
        radians(roll_rate),  # body roll rate
        radians(pitch_rate),  # body pitch rate
        radians(yaw_rate),  # body yaw rate
        thrust,
    )


# Get all the available modes
if len(sys.argv) != 2:
    print("Assuming normal mode")
    attack_mode = False
else:
    attack_mode = (sys.argv[1] == "attack")
print(f"Running eval script in Attack config: {attack_mode}")

# Wait till we are in air 
wait_for_gps_fix()
time.sleep(20)
adsb_mode = vehicle.mode_mapping()["AVOID_ADSB"]
auto_mode = vehicle.mode_mapping()["AUTO"]
# Send in the command to change to AvoidADSB mode
vehicle.set_mode(adsb_mode)
# Send the attitude command
if attack_mode:
    set_attitude()
time.sleep(10)
vehicle.set_mode(auto_mode)
