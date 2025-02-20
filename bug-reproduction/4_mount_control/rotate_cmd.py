# from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from pymavlink import mavutil
import random
import time

# Connect to the UAV's MAVLink interface over TCP
master = mavutil.mavlink_connection("localhost:1337")

# os.environ["MAVLINK20"] = "1"
# os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

print("Sending message for configuration")
master.wait_heartbeat()
# master.mav.mount_configure_send(
#     0, 0, mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING, 1, 1, 1
# )
# ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True)
# ack_msg = ack_msg.to_dict()
# conn.mav.send(config_msg
# , timeout=3
#
# NOTE: 2024-05-11 14:29 Should be enough to send a DO_MOUNT_CONTROL to trigger the message
# message COMMAND_LONG 0 0 205 0 0 0 0 0 0 0 2
#
while True:
    angle = random.randint(0,360)
    master.mav.command_long_send(
        0,
        # self.settings.target_system,
        154,
        # self.settings.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        0,  # confirmation
        0,
        0,
        angle,  # yaw
        0,  # param4
        0,  # lat
        0,  # lon
        0,
    )  # param7
    # conn.mav.command_long_send(
    #     0, 0, 205,
    #     0,
    #     0, 0, 0, 0, 0, 0, 2
    #
    print("Command sent!")
    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if ack_msg:
        ack_msg = ack_msg.to_dict()
        print(ack_msg)
    else:
        print("No message man")
    time.sleep(1)
