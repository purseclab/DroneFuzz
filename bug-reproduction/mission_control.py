#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#  mission_control.py Author "silipwn <contact at as-hw.in>" Description "This file is responsible for extracting and uploading missions" Date 2024-08-02T16:10:26-0400

import argparse
import os
import time
import json
from pymavlink import mavutil, mavwp


def connect_vehicle(connection_string):
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print(
        "Heartbeat from system (system %u component %u)"
        % (master.target_system, master.target_component)
    )
    return master


def download_mission(master, filename):
    master.mav.mission_request_list_send(master.target_system, master.target_component)

    mission_list = []

    while True:
        message = master.recv_match(blocking=True)
        if message.get_type() == "MISSION_COUNT":
            mission_count = message.count
            print(f"Number of mission items: {mission_count}")
            break

    for i in range(mission_count):
        master.mav.mission_request_int_send(
            master.target_system, master.target_component, i
        )
        message = master.recv_match(
            type=["MISSION_ITEM", "MISSION_ITEM_INT"], blocking=True
        )
        if message:
            mission_list.append(message.to_dict())
            print(f"Received mission item {i + 1}/{mission_count}")

    with open(filename, "w") as f:
        json.dump(mission_list, f, indent=4)

    print(f"Mission saved to {filename}")


def upload_mission(master, filename, skip_timeout=False):
    """
    Upload a mission from a waypoint file using MAVProxy's waypoint module

    Args:
        master: MAVLink connection
        filename: Path to the mission file (.waypoints format)
        skip_timeout: Not used with MAVProxy but kept for compatibility
    """
    if not os.path.exists(filename):
        print(f"Mission file {filename} not found!")
        return

    print(f"Loading mission from {filename} using MAVProxy...")

    waypoints = mavwp.MAVWPLoader()
    _ = waypoints.load(filename.strip('"'))

    # Clear any existing mission
    master.waypoint_clear_all_send()
    time.sleep(1)

    # Send waypoint count
    master.waypoint_count_send(waypoints.count())

    # Respond to mission requests
    for i in range(waypoints.count()):
        try:
            # Wait for mission request message
            msg = master.recv_match(type=["MISSION_REQUEST"], blocking=True, timeout=5)
            if not msg:
                print("No mission request received")
                return False

            print(f"Received MISSION_REQUEST for sequence {msg.seq}")

            # Send the requested waypoint
            master.mav.send(waypoints.wp(msg.seq))
            print(f"Sending waypoint {msg.seq}")

        except Exception as e:
            print(f"Error in mission upload: {e}")
            return False

    # Wait for mission ACK
    msg = master.recv_match(type=["MISSION_ACK"], blocking=True, timeout=5)
    if not msg:
        print("No mission ACK received")
        return False

    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("Mission upload successful")
        return True
    else:
        print(f"Mission upload failed with error: {msg.type}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Download or upload mission plans from/to copter"
    )
    parser.add_argument(
        "connection_string",
        type=str,
        help="Connection string for the vehicle",
        default="udp:localhost:14550",
    )
    parser.add_argument("--download", type=str, help="Download mission to file")
    parser.add_argument("--upload", type=str, help="Upload mission from file")
    parser.add_argument("--skip_timeout", action="store_true", help="Skip timeout")
    args = parser.parse_args()

    master = connect_vehicle(args.connection_string)

    if args.download:
        download_mission(master, args.download)

    if args.upload:
        upload_mission(master, args.upload, args.skip_timeout)


if __name__ == "__main__":
    main()
