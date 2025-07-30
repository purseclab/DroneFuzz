#!/usr/bin/env python3
"""
param_comm_tester.py

Script to test parameter changes over MAVLink serial connection and detect communication loss.
- Reads parameters from a CSV file (NAME,VALUE,Desc)
- Sets each parameter on the board
- Checks for communication loss after each set
- Logs parameters that cause comm loss to a separate file
- Waits for user confirmation after comm loss or initial hardfault

Requires: pymavlink
"""
import csv
import time
import sys
import os
from pymavlink import mavutil

# User-configurable serial port (default: /dev/ttyACM0)
DEFAULT_SERIAL = '/dev/ttyACM0'
DEFAULT_CSV = 'pgfuzz_ardupilot_params.csv'
COMM_LOSS_LOG = 'comm_loss_log.csv'


def get_serial_port():
    """
    Prompt the user to enter the serial port for MAVLink connection.
    Returns the user input or the default serial port if left blank.
    """
    port = input(f"Enter serial port for MAVLink connection [{DEFAULT_SERIAL}]: ").strip()
    return port if port else DEFAULT_SERIAL


def get_csv_file():
    """
    Prompt the user to enter the parameter CSV file name.
    Returns the user input or the default CSV file if left blank.
    """
    csv_file = input(f"Enter parameter CSV file [{DEFAULT_CSV}]: ").strip()
    return csv_file if csv_file else DEFAULT_CSV


def wait_for_user(msg):
    """
    Display a message and wait for the user to press Enter to continue.
    """
    input(f"{msg} Press Enter to continue...")


def connect_mavlink(port):
    """
    Connect to the MAVLink device over the specified serial port.
    Waits for a heartbeat to confirm connection.
    Returns the MAVLink connection object.
    Raises RuntimeError if connection fails.
    """
    print(f"Connecting to {port} (baudrate auto-detect)...")
    master = mavutil.mavlink_connection(port, autoreconnect=True, baud=0)
    # Runtime type check for MAVLink connection
    if not isinstance(master, mavutil.mavfile):
        raise RuntimeError("Failed to create MAVLink connection. Check the serial port and try again.")
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat received from system {master.target_system}")
    return master


def set_param(master, name, value):
    """
    Set a parameter on the MAVLink device.
    Sends a PARAM_SET message and waits for an acknowledgment.
    """
    print(f"Setting parameter {name} to {value}...")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode('utf-8'),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    # Wait for param ack
    ack = master.recv_match(type='PARAM_VALUE', condition=f'PARAM_VALUE.param_id=="{name}"', timeout=5)
    if ack:
        print(f"Parameter {name} set to {ack.param_value}")
    else:
        print(f"No ack for {name}, continuing...")


def check_comm(master, timeout=5):
    """
    Check if the MAVLink communication link is still active by waiting for a heartbeat.
    Returns True if heartbeat is received, False otherwise.
    """
    print("Checking for heartbeat...")
    try:
        master.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        print("Heartbeat OK.")
        return True
    except Exception:
        print("No heartbeat detected!")
        return False

def trigger_hardfault(master):
    """ 
    Creates a long loop condition by sending a specific command.
    Specific to the 4.0.3 version of ArduPilot. The version under test with PGFUZZ.
    """
    print("Sending hardfault ....")
    master.mav.command_long_send(
            0,
            0,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
            42,
            24,
            71,
            93,
            0,
            0,
            0)

def log_comm_loss(param_row):
    """
    Log the parameter row to the comm loss log file if communication is lost.
    Adds a header if the log file does not exist.
    """
    file_exists = os.path.isfile(COMM_LOSS_LOG)
    with open(COMM_LOSS_LOG, 'a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(['NAME', 'VALUE', 'Desc'])
        writer.writerow(param_row)
    print(f"Logged comm loss for parameter: {param_row[0]}")


def main():
    """
    Main function to run the parameter communication tester.
    Handles user prompts, connection, parameter setting, comm loss detection, and logging.
    """
    print("=== MAVLink Parameter Communication Tester ===")
    port = get_serial_port()
    csv_file = get_csv_file()

    # Connect to board before triggering hardfault
    while True:
        try:
            master = connect_mavlink(port)
            break
        except Exception as e:
            print(f"Connection failed: {e}")
            change = input("Would you like to change the serial port? [y/N]: ").strip().lower()
            if change == 'y':
                port = get_serial_port()
            else:
                wait_for_user("Reconnect the board and try again.")

    print("\nAbout to trigger a hardfault via MAVLink command.\n")
    wait_for_user("Ensure board is ready. Press Enter to trigger hardfault.")
    trigger_hardfault(master)
    print("Hardfault command sent. Board should reboot or become unresponsive.")
    wait_for_user("After hardfault, reset/reboot the board and press Enter to continue.")

    # Reconnect after hardfault
    while True:
        try:
            master = connect_mavlink(port)
            break
        except Exception as e:
            print(f"Connection failed: {e}")
            change = input("Would you like to change the serial port? [y/N]: ").strip().lower()
            if change == 'y':
                port = get_serial_port()
            else:
                wait_for_user("Reconnect the board and try again.")

    # Read parameters from CSV
    with open(csv_file, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            name, value, desc = row
            set_param(master, name, value)
            if not check_comm(master):
                log_comm_loss(row)
                wait_for_user("Comm link lost. Please reboot the board and press Enter to continue.")
                # Reconnect after reboot
                while True:
                    try:
                        master = connect_mavlink(port)
                        break
                    except Exception as e:
                        print(f"Connection failed: {e}")
                        change = input("Would you like to change the serial port? [y/N]: ").strip().lower()
                        if change == 'y':
                            port = get_serial_port()
                        else:
                            wait_for_user("Reconnect the board and try again.")
            time.sleep(1)  # Small delay between params
    print("\nTesting complete.")

if __name__ == '__main__':
    main()
