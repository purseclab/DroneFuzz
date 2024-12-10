# from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import time
from pymavlink import mavutil


def send_cmd(vehicle):
    # NOTE: 2024-05-11 14:29 Should be enough to send a DO_MOUNT_CONTROL to trigger the message
    # message COMMAND_LONG 0 0 205 0 0 0 0 0 0 0 2
    #

    # conn.mav.command_long_send(
    #     0, 0, 205,
    #     0,
    #     0, 0, 0, 0, 0, 0, 2
    #
    vehicle.mav.command_long_send(
        0,
        # self.settings.target_system,
        154,
        # self.settings.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        0,  # confirmation
        0,
        0,
        90,  # yaw
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
    is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    is_auto_mode = (
        msg.custom_mode == mavutil.mavlink.COPTER_MODE_LOITER
    )  # To check if we can detect in mission mode
    return is_armed and is_auto_mode


if __name__ == "__main__":
    # Connect to the UAV's MAVLink interface over TCP
    vehicle = mavutil.mavlink_connection("localhost:1337")

    print("Sending message for configuration")
    vehicle.wait_heartbeat()

    while True:
        if is_copter_ready(vehicle):
            for _ in range(3):
                send_cmd(vehicle)
                time.sleep(1)
            break
        else:
            print("Copter not ready")
            time.sleep(1)
