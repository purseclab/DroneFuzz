# from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import time
import random
from pymavlink import mavutil
import sys


def send_cmd(vehicle,attack_mode):
    # NOTE: 2024-05-11 14:29 Should be enough to send a DO_MOUNT_CONTROL to trigger the message
    # message COMMAND_LONG 0 0 205 0 0 0 0 0 0 0 2
    #

    # conn.mav.command_long_send(
    #     0, 0, 205,
    #     0,
    #     0, 0, 0, 0, 0, 0, 2
    if attack_mode:
        yaw = 180
    else:
        yaw = random.randint(0, 180)
    vehicle.mav.command_long_send(
        0,
        # self.settings.target_system,
        154,
        # self.settings.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        0,  # confirmation
        0,
        0,
        yaw,  # yaw
        0,  # param4
        0,  # lat
        0,  # lon
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
    )  # param7
    print("Command sent!")
    ack_msg = vehicle.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if ack_msg:
        ack_msg = ack_msg.to_dict()
        print(ack_msg)
    else:
        print("No message received")


def is_copter_ready(vehicle):
    msg = vehicle.recv_match(type="HEARTBEAT", blocking=True)
    is_in_air = False
    is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    is_auto_mode = (
        msg.custom_mode == mavutil.mavlink.COPTER_MODE_AUTO
    )  # To check if we can detect in mission mode
    # Also check if we have reached required altitude
    msg = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    alt = msg.relative_alt / 1000.0
    if alt > 45:
        is_in_air = True
    return is_armed and is_auto_mode and is_in_air


if __name__ == "__main__":
    # Connect to the UAV's MAVLink interface over TCP
    vehicle = mavutil.mavlink_connection("localhost:1337")

    print("Sending message for configuration")
    vehicle.wait_heartbeat()
    if len(sys.argv) < 2:
        print("WARNING: No value in for mode")
        attack_mode = False
    else:
        attack_mode = (sys.argv[1] == 'attack')

    while True:
        if is_copter_ready(vehicle):
            for _ in range(3):
                send_cmd(vehicle,attack_mode)
                time.sleep(1)
            break
        else:
            print("Copter not ready")
            time.sleep(1)
