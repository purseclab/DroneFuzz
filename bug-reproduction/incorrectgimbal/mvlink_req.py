import os
import sys
from time import sleep

from pymavlink import mavutil, mavwp
# from pymavlink.dialects.v20 import ardupilotmega as mavlink2
os.environ['MAVLINK20'] = '1'

# message COMMAND_LONG 0 0 512 0 286 0 0 0 0 0 0

connection =  mavutil.mavlink_connection("localhost:1337")

# XXX: Maybe can ignore?
connection.wait_heartbeat()

hz = 2
while True:
    connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0, # Confirmation
    mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE,
        0, 0, 0, 0, 0, 0)
    # _ = connection.recv_match(type="AUTOPILOT_STATE_FOR_GIMBAL_DEVICE",blocking=True).to_dict()
    sleep(1/hz)
