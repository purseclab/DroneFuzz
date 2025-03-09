#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import time
from pymavlink import mavutil
import mission_control

def wait_for_gps(master):
    print("Waiting for GPS lock (looking for 'is using GPS')...")
    while True:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if not msg:
            continue
        if "is using GPS" in msg.text:
            print("Vehicle is using GPS, proceeding.")
            time.sleep(2) # Wait a bit more to ensure GPS lock
            break

def apply_throttle(master, throttle_pwm=1500, duration=1.0):
    end_time = time.time() + duration
    while time.time() < end_time:
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0,  # chan1
            0,  # chan2
            throttle_pwm,  # chan3
            0,  # chan4
            0,  # chan5
            0,  # chan6
            0,  # chan7
            0   # chan8
        )
        time.sleep(0.1)
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def arm_and_set_mode(master, mode="AUTO"):
    # https://ardupilot.org/copter/docs/auto-mode.html#starting-a-mission
    mode_id = master.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Mode {mode} not found in mode mapping.")
        return

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming vehicle...")
    master.set_mode(mode_id)
    print(f"Setting vehicle mode to {mode}.")
    apply_throttle(master, 1500, 1.0)

def wait_for_mission_end(master):
    print("Monitoring mission progress...")
    while True:
        msg = master.recv_match(type=["HEARTBEAT", "SYS_STATUS"], blocking=True, timeout=1)
        if not msg:
            continue
        if msg.get_type() == "HEARTBEAT":
            base_mode = msg.base_mode
            custom_mode = msg.custom_mode
            if base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                ext_state = master.recv_match(type="EXTENDED_SYS_STATE", blocking=False)
                if ext_state and ext_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                    print("Mission finished and vehicle landed.")
                    break

def main():
    parser = argparse.ArgumentParser(
        description="Upload mission and monitor until vehicle lands"
    )
    parser.add_argument(
        "connection_string",
        type=str,
        help="Connection string for the vehicle",
        default="udp:localhost:14550",
        nargs="?"
    )
    parser.add_argument("--upload", type=str, help="Path to mission file to upload")
    parser.add_argument("--skip_timeout", action="store_true", help="Skip upload timeout")
    args = parser.parse_args()

    master = mission_control.connect_vehicle(args.connection_string)

    wait_for_gps(master)

    if args.upload:
        mission_control.upload_mission(master, args.upload, args.skip_timeout)
        arm_and_set_mode(master, mode="AUTO")
        wait_for_mission_end(master)
    else:
        print("No upload file provided.")

if __name__ == "__main__":
    main()