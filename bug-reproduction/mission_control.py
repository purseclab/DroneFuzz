#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#  mission_control.py Author "silipwn <contact at as-hw.in>" Description "This file is responsible for extracting and uploading missions" Date 2024-08-02T16:10:26-0400

import argparse
import os
import json
from pymavlink import mavutil


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


def upload_mission(master, filename, skip_timeout):
    if not os.path.exists(filename):
        print(f"Mission file {filename} not found!")
        return

    with open(filename, "r") as f:
        mission_list = json.load(f)

    mission_count = len(mission_list)
    print(
        "Connection details {0} {1}".format(
            master.target_system, master.target_component
        )
    )
    master.mav.mission_count_send(
        master.target_system, master.target_component, mission_count
    )
    # Check for mission_request_int
    if not skip_timeout:
        # NOTE: 2024-08-02T16:06:14-0400: silipwn: For some reason we don't see this packet coming at all
        message = master.recv_match(type="MISSION_REQUEST_INT", blocking=True)
        print(message)

    for i, item in enumerate(mission_list):
        item["target_system"] = master.target_system
        item["target_component"] = master.target_component
        item["seq"] = i
        # Ignore these fields
        # mavpackettype
        item.pop("mavpackettype", None)
        master.mav.send(mavutil.mavlink.MAVLink_mission_item_int_message(**item))

    # Wait for mission_ack
    message = master.recv_match(type="MISSION_ACK", blocking=True)
    if message.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("Mission upload complete.")
    else:
        print("Mission upload failed.")
        print(message)


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
