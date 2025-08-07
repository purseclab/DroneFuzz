import argparse
import time
import threading
import sys
import os
import ast
from pymavlink import mavutil, mavwp

PREARM_CHECK = 0x10000000
EKF_POS_HORIZ = 0x8
EKF_POS_VERT = 0x10

replay_event = threading.Event()
replay_event.clear()  # Clear the event initially
start_time = 0


def dummy_inputs(vehicle):
    """ Just send harcoded inputs for sensor health"""
    if not replay_event.is_set():
        # Create obstacle distance array (72 values for 5-degree increments)
        distances = [65535] * 72  # 65535 means no obstacle detected
        vehicle.mav.obstacle_distance_send(
            int((time.time() - start_time) * 1000000),  # time_usec (microseconds)
            0,  # sensor_type (0 = generic)
            distances,  # distances array
            0,  # increment (0 = 5 degrees)
            500,  # min_distance (50cm)
            10000,  # max_distance (1000cm)
            0.0,  # increment_f (not used when increment is 0)
            0.0,  # angle_offset
            0   # frame (0 = MAV_FRAME_GLOBAL)
        )
        time.sleep(1/15) # Simulate 15Hz rate

def vehicle_ready(vehicle, timeout=30):
    """
    Checks if the vehicle is ready for mission.
    """
    # Monitor the following messages
    # 1. Heartbeat
    # 2. GPS_RAW_INT
    # 3. SYS_STATUS
    prearm_check = False
    gps_check = False
    ekf_check = False
    init_time = time.time()
    try:
        while not (prearm_check and gps_check and ekf_check):
            remaining = (init_time + timeout) - time.time()
            if remaining <= 0:
                print("Timeout waiting for vehicle to be ready.")
                return False
            msg = vehicle.recv_match(type=["HEARTBEAT", "GPS_RAW_INT", "SYS_STATUS"], blocking=True, timeout=remaining)
            if not msg:
                continue
            if msg.get_type() == "HEARTBEAT" and not prearm_check:
                if hasattr(msg, "onboard_control_sensors") and (msg.onboard_control_sensors & PREARM_CHECK):
                    print("Prearm check passed.")
                    prearm_check = True
            elif msg.get_type() == "GPS_RAW_INT" and not gps_check:
                if hasattr(msg, "fix_type") and msg.fix_type >= 3:  # 3D fix
                    print("GPS check passed.")
                    gps_check = True
            elif msg.get_type() == "SYS_STATUS" and not ekf_check:
                if hasattr(msg, "onboard_control_sensors_health") and \
                   (msg.onboard_control_sensors_health & EKF_POS_HORIZ) and \
                   (msg.onboard_control_sensors_health & EKF_POS_VERT):
                    ekf_check = True
                    print("EKF check passed.")
        print("Vehicle is ready for mission.")
        return True
    except Exception as e:
        print(f"Error while waiting for vehicle to be ready: {e}")
        return False

def connect_to_vehicle(connection_string, retries=5):
    """
    Connects to the vehicle with retries.
    """
    for i in range(retries):
        print(f"Connecting to vehicle... (Attempt {i + 1}/{retries})")
        try:
            vehicle = mavutil.mavlink_connection(connection_string, baud=115200)
            vehicle.wait_heartbeat()
            print(
                "Heartbeat from system (system %u component %u)"
                % (vehicle.target_system, vehicle.target_component)
            )
            return vehicle
        except Exception as e:
            print(f"Failed to connect: {e}")
            time.sleep(5)
    print("Could not connect to vehicle. Exiting.")
    sys.exit(1)


def upload_mission(vehicle, mission_file):
    """
    Uploads a mission from a waypoint file.
    """
    print(f"Uploading mission from {mission_file}")
    wploader = mavwp.MAVWPLoader()
    try:
        with open(mission_file, "r") as f:
            for i, line in enumerate(f):
                if i == 0:
                    if "QGC WPL 110" not in line:
                        print("Invalid mission file format. Expected 'QGC WPL 110'.")
                        return False
                    continue
                line_parts = line.strip().split("\t")
                if len(line_parts) < 12:
                    continue

                p = mavutil.mavlink.MAVLink_mission_item_message(
                    vehicle.target_system,
                    vehicle.target_component,
                    int(line_parts[0]),
                    int(line_parts[2]),
                    int(line_parts[3]),
                    float(line_parts[4]),
                    float(line_parts[5]),
                    float(line_parts[6]),
                    float(line_parts[7]),
                    float(line_parts[8]),
                    float(line_parts[9]),
                    float(line_parts[10]),
                    int(line_parts[1]),
                )
                wploader.add(p)

    except Exception as e:
        print(f"Error loading mission file: {e}")
        return False

    vehicle.waypoint_clear_all_send()
    vehicle.waypoint_count_send(wploader.count())

    for i in range(wploader.count()):
        msg = vehicle.recv_match(type=["MISSION_REQUEST"], blocking=True, timeout=5)
        if not msg:
            print("Timeout waiting for MISSION_REQUEST")
            return False
        print(f"Sending waypoint {msg.seq}")
        vehicle.mav.send(wploader.wp(msg.seq))

    msg = vehicle.recv_match(type=["MISSION_ACK"], blocking=True, timeout=5)
    if msg and msg.type == 0:  # MAV_MISSION_ACCEPTED
        print("Mission uploaded successfully.")
        return True
    else:
        print("Mission upload failed.")
        return False


def parse_peripheral_inputs(peripheral_file):
    """
    Parses the peripheral input file.
    """
    if not os.path.exists(peripheral_file):
        print(f"Error: peripheral input file not found at {peripheral_file}")
        sys.exit(1)

    inputs = []
    try:
        with open(peripheral_file, "r") as f:
            for line in f:
                try:
                    # Using ast.literal_eval for safety
                    data = ast.literal_eval(line.strip())
                    if isinstance(data, list) and len(data) == 4:
                        inputs.append(data)
                    else:
                        raise ValueError("Invalid format")
                except (ValueError, SyntaxError) as e:
                    print(f"Could not parse line: {line.strip()} - {e}")
                    sys.exit(1)
    except Exception as e:
        print(f"Error reading peripheral file: {e}")
        sys.exit(1)

    # Sort by timestamp
    inputs.sort(key=lambda x: x[0])
    return inputs


def replay_peripherals(vehicle, peripheral_inputs):
    """
    Replays peripheral messages in a separate thread.
    """
    print("Starting peripheral replay thread.")
    replay_event.set()  # Signal that we are ready to replay
    for timestamp, msg_name, msg_id, field_values in peripheral_inputs:
        # Calculate when to send the message
        current_time = time.time() - start_time
        delay = timestamp - current_time
        if delay > 0:
            time.sleep(delay)

        try:
            # For PARAM_SET, use a specific function
            if msg_name == "PARAM_SET":
                param_id = field_values.get("param_id")
                param_value = field_values.get("param_value")
                if param_id and param_value is not None:
                    print(f"Sending {msg_name}: {param_id} = {param_value}")
                    vehicle.param_set_send(param_id.encode("utf-8"), param_value)
                else:
                    print(f"Skipping malformed PARAM_SET: {field_values}")

            # For other messages, construct and send them
            else:
                # This requires message definitions. Pymavlink can construct messages by name.
                msg_obj_class = getattr(
                    mavutil.mavlink, f"MAVLink_{msg_name.lower()}_message", None
                )

                if msg_obj_class:
                    # Filter field_values to only include valid fields for the message
                    valid_fields = {
                        k: v
                        for k, v in field_values.items()
                        if k in msg_obj_class.fieldnames
                    }

                    # Pymavlink message constructors need all fields.
                    # We will fill missing ones with 0.
                    args = [
                        valid_fields.get(field, 0)
                        for field in msg_obj_class.fieldnames
                        if field != "mavlink_version"
                    ]

                    try:
                        msg = msg_obj_class(*args)
                        print(f"Sending {msg_name}: {valid_fields}")
                        vehicle.mav.send(msg)
                    except Exception as e:
                        print(
                            f"Error creating message {msg_name} with fields {field_values}: {e}"
                        )
                else:
                    print(f"Unknown message name: {msg_name}")

        except Exception as e:
            print(f"Error sending message: {e}")

    print("Finished replaying all peripheral inputs.")


def main():
    """
    Main function to drive the replay script.
    """
    parser = argparse.ArgumentParser(
        description="Reproducer script for peripheral messages."
    )
    parser.add_argument(
        "-s",
        "--peripheral-inputs",
        required=True,
        help="Path to peripheral input file.",
    )
    parser.add_argument(
        "-m",
        "--mission-file",
        required=True,
        help="Path to mission file (QGC WPL 110 format).",
    )
    parser.add_argument(
        "-c",
        "--connect",
        default="tcp:127.0.0.1:5760",
        help="Connection string for SITL.",
    )

    args = parser.parse_args()

    # Parse peripheral inputs first to fail early
    peripheral_inputs = parse_peripheral_inputs(args.peripheral_inputs)

    # Connect to the vehicle
    vehicle = connect_to_vehicle(args.connect)

    # Upload mission
    if not upload_mission(vehicle, args.mission_file):
        print("Exiting due to mission upload failure.")
        sys.exit(1)
    
    # Wait till the vehicle is ready
    print("Waiting for vehicle to be ready...")
    if not vehicle_ready(vehicle):
        print("Vehicle is not ready. Exiting.")
        exit(1)

    global start_time; start_time = time.time()  # Set the start time for the replay

    # Start the dummy inputs thread
    dummy_thread = threading.Thread(target=dummy_inputs, args=(vehicle,))
    dummy_thread.start()
    # ARM the vehicle to start the mission
    print("Arming vehicle")
    vehicle.arducopter_arm()

    # Set mode to AUTO 
    print("Setting mode to AUTO")
    vehicle.set_mode_auto()

    # Now apply throttle to start the mission
    print("Applying throttle to start the mission")
    # Using the MAVLINK_RC_CHANNELS_OVERRIDE message to apply throttle
    # Send this for 1 second to ensure the vehicle starts moving
    finish_time = time.time() + 1
    while time.time() < finish_time:  # Send every 0.5 seconds
        vehicle.mav.rc_channels_override_send(
            vehicle.target_system,
            vehicle.target_component,
            0,  # Channel 1 (Roll)
            0,  # Channel 2 (Pitch)
            1500,  # Channel 3 (Throttle) - Neutral position
            0,  # Channel 4 (Yaw)
            0,  # Channel 5
            0,  # Channel 6
            0,  # Channel 7
            0   # Channel 8
        )
        time.sleep(0.1)
    # Reset the throttle to neutral position
    vehicle.mav.rc_channels_override_send(
        vehicle.target_system,
        vehicle.target_component,
        0,  # Channel 1 (Roll)
        0,  # Channel 2 (Pitch)
        0,  # Channel 3 (Throttle) - Neutral position
        0,  # Channel 4 (Yaw)
        0,  # Channel 5
        0,  # Channel 6
        0,  # Channel 7
        0   # Channel 8
    )

    # Start peripheral replay in a new thread
    replay_thread = threading.Thread(
        target=replay_peripherals, args=(vehicle, peripheral_inputs)
    )
    replay_thread.start()

    # Wait for the replay to finish
    replay_thread.join()

    print("Script finished.")


if __name__ == "__main__":
    main()
