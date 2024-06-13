import time
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect

# Connect to the vehicle
master = mavutil.mavlink_connection("udp:127.0.0.1:1337")
master.wait_heartbeat()

TAKEOFF_ALTITUDE = 100

# FIXME: Ideally wait will IMU is using GPS data
# Wait till copter has position for EKF
# master.mav.request_data_stream_send(
#     master.target_system, master.target_component,
#     mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
#     2, 1)  # Rate and start/stop

# # Wait for the EXTENDED_SYS_STATE message
# msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True)

# # Check if the EKF is using GPS and has converged
# if msg.fault_flag & mavutil.mavlink.ESTIMATOR_GPS_PRIMARY_FAULT == 0 and msg.fault_flag & mavutil.mavlink.ESTIMATOR_ATT_FAULT == 0:
#     print("EKF is ready and using GPS data.")

# Arm the motors
print("Arming motors...")
print(f"Check the {master.target_system} and {master.target_component}")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
)

arm_msg = master.recv_match(type="COMMAND_ACK", blocking=True)
if arm_msg.to_dict()["result"] != mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("AP returned command not accpeted")
    print(f"{arm_msg.to_dict()}")
    print("Something is not right, exiting")
    exit(-1)

# For setup purposes, wait for 10 seconds
time.sleep(4)

# Ensure we are guided mode
# Change mode to Guided
print("Changing mode to Guided...")
mode_id = master.mode_mapping()["GUIDED"]

# https://github.com/Intelligent-Quads/iq_pymavlink_tutorial/blob/master/takeoff.py
master.mav.command_long_send(
    target_system=master.target_system,
    target_component=master.target_component,
    command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=mode_id,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0,
)

arm_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
time.sleep(1)
# Take off to 50 meters
print("Taking off to 100 meters...")
takeoff_command = dialect.MAVLink_command_long_message(
    target_system=master.target_system,
    target_component=master.target_component,
    command=dialect.MAV_CMD_NAV_TAKEOFF,
    confirmation=0,
    param1=0,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=TAKEOFF_ALTITUDE,
)
master.mav.send(takeoff_command)

# Wait for the vehicle to reach the desired altitude
while True:
    msg = master.recv_match(type=["GLOBAL_POSITION_INT"], blocking=True)
    if msg is not None:
        altitude = msg.relative_alt / 1000.0  # Altitude in meters
        if altitude >= 98:
            break

# Change mode to Loiter
time.sleep(2)
mode_id = master.mode_mapping()["LOITER"]
master.mav.command_long_send(
    target_system=master.target_system,
    target_component=master.target_component,
    command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=mode_id,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0,
)

# Wait for 40 seconds
print("Waiting for 60 seconds...")
time.sleep(60)

# Return to Land
print("Returning to Land...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
)

# Wait for the vehicle to land
while True:
    msg = master.recv_match(type=["GLOBAL_POSITION_INT"], blocking=True)
    if msg is not None:
        altitude = msg.relative_alt / 1000.0  # Altitude in meters
        if altitude <= 0.3:
            break

# Disarm the motors
print("Disarming motors...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
)

print("Changing mode to Stabilize...")
mode_id = master.mode_mapping()["STABILIZE"]

# https://github.com/Intelligent-Quads/iq_pymavlink_tutorial/blob/master/takeoff.py
master.mav.command_long_send(
    target_system=master.target_system,
    target_component=master.target_component,
    command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=mode_id,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0,
)
