#!/usr/bin/env python3

import json
import time
from pymavlink import mavutil


def load_waypoints(filename):
    with open(filename, "r") as f:
        return json.load(f)


def send_waypoint_command(master, lat, lon, alt):
    """
    Send MAV_CMD_NAV_WAYPOINT command
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0,  # confirmation
        0,  # Hold time in seconds
        2,  # Acceptance radius in meters
        0,  # Pass through waypoint
        0,  # Desired yaw angle
        lat / 10000000.0,  # Latitude
        lon / 10000000.0,  # Longitude
        alt,  # Altitude
    )


def main():
    # Connect to the vehicle
    master = mavutil.mavlink_connection("udpin:localhost:14550")

    # Wait for heartbeat
    master.wait_heartbeat()
    print("Connected to vehicle")

    # Load waypoints
    waypoints = load_waypoints("simple_movement_AU.json")

    # Send each waypoint command
    for wp in waypoints:
        if wp["command"] == 16:  # MAV_CMD_NAV_WAYPOINT
            print(
                f"Sending waypoint: lat={wp['x']/10000000.0}, lon={wp['y']/10000000.0}, alt={wp['z']}"
            )
            send_waypoint_command(master, wp["x"], wp["y"], wp["z"])

            # Wait for command acknowledgment
            ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
            if ack:
                print(f"Command acknowledged with result: {ack.result}")
            else:
                print("Command acknowledgment timed out")

            time.sleep(2)  # Small delay between commands


if __name__ == "__main__":
    main()
