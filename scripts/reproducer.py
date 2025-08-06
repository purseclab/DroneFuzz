import argparse
import time
import threading
import sys
import os
import ast
from pymavlink import mavutil, mavwp

def connect_to_vehicle(connection_string, retries=5):
    """
    Connects to the vehicle with retries.
    """
    for i in range(retries):
        print(f"Connecting to vehicle... (Attempt {i + 1}/{retries})")
        try:
            vehicle = mavutil.mavlink_connection(connection_string, baud=115200)
            vehicle.wait_heartbeat()
            print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))
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
        with open(mission_file, 'r') as f:
            for i, line in enumerate(f):
                if i == 0:
                    if 'QGC WPL 110' not in line:
                        print("Invalid mission file format. Expected 'QGC WPL 110'.")
                        return False
                    continue
                line_parts = line.strip().split('\t')
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
                    int(line_parts[1])
                )
                wploader.add(p)

    except Exception as e:
        print(f"Error loading mission file: {e}")
        return False

    vehicle.waypoint_clear_all_send()
    vehicle.waypoint_count_send(wploader.count())

    for i in range(wploader.count()):
        msg = vehicle.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=5)
        if not msg:
            print("Timeout waiting for MISSION_REQUEST")
            return False
        print(f"Sending waypoint {msg.seq}")
        vehicle.mav.send(wploader.wp(msg.seq))

    msg = vehicle.recv_match(type=['MISSION_ACK'], blocking=True, timeout=5)
    if msg and msg.type == 0: # MAV_MISSION_ACCEPTED
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
        with open(peripheral_file, 'r') as f:
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
    start_time = time.time()
    
    for timestamp, msg_name, msg_id, field_values in peripheral_inputs:
        # Calculate when to send the message
        current_time = time.time() - start_time
        delay = timestamp - current_time
        if delay > 0:
            time.sleep(delay)
        
        try:
            # For PARAM_SET, use a specific function
            if msg_name == "PARAM_SET":
                param_id = field_values.get('param_id')
                param_value = field_values.get('param_value')
                if param_id and param_value is not None:
                    print(f"Sending {msg_name}: {param_id} = {param_value}")
                    vehicle.param_set_send(param_id.encode('utf-8'), param_value)
                else:
                    print(f"Skipping malformed PARAM_SET: {field_values}")

            # For other messages, construct and send them
            else:
                # This requires message definitions. Pymavlink can construct messages by name.
                msg_obj_class = getattr(mavutil.mavlink, f"MAVLink_{msg_name.lower()}_message", None)

                if msg_obj_class:
                    # Filter field_values to only include valid fields for the message
                    valid_fields = {k: v for k, v in field_values.items() if k in msg_obj_class.fieldnames}
                    
                    # Pymavlink message constructors need all fields.
                    # We will fill missing ones with 0.
                    args = [valid_fields.get(field, 0) for field in msg_obj_class.fieldnames if field != 'mavlink_version']

                    try:
                        msg = msg_obj_class(*args)
                        print(f"Sending {msg_name}: {valid_fields}")
                        vehicle.mav.send(msg)
                    except Exception as e:
                        print(f"Error creating message {msg_name} with fields {field_values}: {e}")
                else:
                    print(f"Unknown message name: {msg_name}")

        except Exception as e:
            print(f"Error sending message: {e}")

    print("Finished replaying all peripheral inputs.")

def main():
    """
    Main function to drive the replay script.
    """
    parser = argparse.ArgumentParser(description="Reproducer script for peripheral messages.")
    parser.add_argument('-s', '--peripheral-inputs', required=True, help="Path to peripheral input file.")
    parser.add_argument('-m', '--mission-file', required=True, help="Path to mission file (QGC WPL 110 format).")
    parser.add_argument('-c', '--connect', default='tcp:127.0.0.1:5762', help="Connection string for SITL.")
    
    args = parser.parse_args()

    # Parse peripheral inputs first to fail early
    peripheral_inputs = parse_peripheral_inputs(args.peripheral_inputs)

    # Connect to the vehicle
    vehicle = connect_to_vehicle(args.connect)

    # Upload mission
    if not upload_mission(vehicle, args.mission_file):
        print("Exiting due to mission upload failure.")
        sys.exit(1)

    # Start peripheral replay in a new thread
    replay_thread = threading.Thread(target=replay_peripherals, args=(vehicle, peripheral_inputs))
    replay_thread.start()

    # Set mode to AUTO and ARM the vehicle to start the mission
    print("Setting mode to AUTO")
    vehicle.set_mode_auto()
    
    print("Arming vehicle")
    vehicle.arducopter_arm()
    
    # Wait for the replay to finish
    replay_thread.join()

    print("Script finished.")

if __name__ == "__main__":
    main()
