# Basically a singular file to do the fuzzing loop
# Start the SITL binary
# Establish the TCP connection
# Set the guided mission
# Fuzz some messages based on the XML loading
# Check the status after the mission finishes
import argparse
import struct
import time
import re
import yaml
import os
import random
import tempfile
import logging
import datetime
import pickle
from lxml import etree
from enum import Enum
import heapq
from dataclasses import dataclass, field
from contextlib import redirect_stdout
import numpy as np
from tqdm import tqdm
from typing import Any, Dict, Optional

# Models import
from dtw import dtw
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import MinMaxScaler
import torch
import torch.nn as nn
import torch.optim as optim


# Set the mavlink version to 2
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil, mavwp
import subprocess
from queue import Queue
import threading


# Exceptions
class InternalError(Exception):
    pass


# Setup logging
def setup_logging(file_dir=None):
    """Setup logging with timestamp in filename"""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"dronefuzz_{timestamp}.log"
    if file_dir:
        log_filename = os.path.join(file_dir, log_filename)

    # Create logger
    logger = logging.getLogger("dronefuzz")
    logger.setLevel(logging.DEBUG)
    logger.propagate = False

    # Create file handler for all logs
    file_handler = logging.FileHandler(log_filename)
    file_handler.setLevel(logging.DEBUG)

    # Create console handler for important logs
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.WARNING)

    # Create formatters
    file_formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(lineno)d - %(threadName)s - %(levelname)s - %(message)s"
    )
    console_formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )

    # Apply formatters
    file_handler.setFormatter(file_formatter)
    console_handler.setFormatter(console_formatter)

    # Add handlers to logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    print(f"Logging initialized. Log file: {log_filename}")
    return logger


# Initialize logger

mavlink_timeout = 5
approx_threshold = 0.00005  # Threshold for approximate location matching
altitude_threshold = 0.1  # Threshold for altitude matching
PREARM_CHECK = 0x10000000
EKF_POS_HORIZ = 0x8
EKF_POS_VERT = 0x10

# Global error queue
error_queue = Queue()

RANDOM_SEED = 42  # For reproducibility

# Supported models
detection_models = ["dtw", "lstm"]


class LSTMAE(nn.Module):
    def __init__(self, seq_len, n_features, hidden_size_enc=64, hidden_size_dec=64):
        super(LSTMAE, self).__init__()
        self.seq_len = seq_len
        self.n_features = n_features
        self.hidden_size_enc = hidden_size_enc
        self.hidden_size_dec = hidden_size_dec

        # Encoder
        self.lstm1_enc = nn.LSTM(
            input_size=n_features, hidden_size=128, batch_first=True
        )
        self.dropout1_enc = nn.Dropout(0.2)
        self.lstm2_enc = nn.LSTM(
            input_size=128, hidden_size=hidden_size_enc, batch_first=True
        )

        # Decoder
        # The decoder takes the last hidden state of the encoder (hidden_size_enc)
        # and "repeats" it for each timestep in the sequence.
        # This is implicitly handled by the LSTM's initial hidden state or by
        # feeding the last encoder output repeatedly. Here, we feed a Linear
        # layer that maps the bottleneck to the decoder's input size.
        self.linear_dec = nn.Linear(
            hidden_size_enc, hidden_size_dec
        )  # Mapping bottleneck to decoder hidden size
        self.lstm1_dec = nn.LSTM(
            input_size=hidden_size_dec, hidden_size=128, batch_first=True
        )
        self.dropout1_dec = nn.Dropout(0.2)
        self.lstm2_dec = nn.LSTM(
            input_size=128, hidden_size=n_features, batch_first=True
        )  # Output layer

    def forward(self, x):
        # Encoder
        # Input shape: (batch_size, seq_len, n_features)
        # hidden_state and cell_state are initialized to zeros by default if not provided
        x, (hidden_state, cell_state) = self.lstm1_enc(x)
        x = self.dropout1_enc(x)
        x, (hidden_state, cell_state) = self.lstm2_enc(
            x
        )  # hidden_state[-1] contains the last hidden state for the last layer.

        # Keras's RepeatVector takes the last output of the previous layer and
        # repeats it for `seq_len` times. In PyTorch, we can use `expand` or `repeat`
        # on the last hidden state/output to create the sequence for the decoder.
        # We'll use the last hidden state of the second encoder LSTM as the "bottleneck".
        # We need the hidden state for the last layer of the encoder.
        # hidden_state shape: (num_layers * num_directions, batch, hidden_size_enc)
        # We take the last layer's hidden state: hidden_state[-1, :, :]
        # Then unsqueeze it to (batch, 1, hidden_size_enc) and expand to (batch, seq_len, hidden_size_enc)

        # Take the hidden state from the last layer of the encoder LSTM
        # (num_layers, batch_size, hidden_size) -> (batch_size, hidden_size)
        bottleneck = hidden_state[-1, :, :]

        # "RepeatVector" equivalent:
        # Expand the bottleneck output to match the sequence length for the decoder
        # (batch_size, hidden_size_enc) -> (batch_size, 1, hidden_size_enc) -> (batch_size, seq_len, hidden_size_enc)
        bottleneck_repeated = bottleneck.unsqueeze(1).expand(-1, self.seq_len, -1)

        # Apply the linear transformation before feeding to decoder LSTM
        x = self.linear_dec(bottleneck_repeated)

        # Decoder
        # Input to decoder is (batch_size, seq_len, hidden_size_dec)
        x, _ = self.lstm1_dec(x)
        x = self.dropout1_dec(x)
        x, _ = self.lstm2_dec(
            x
        )  # Output is (batch_size, seq_len, n_features) for TimeDistributed Dense

        return x


# Mutation helpers
def float_to_bits(f):
    """Convert float to its bit representation as integer"""
    return struct.unpack(">I", struct.pack(">f", f))[0]


def bits_to_float(bits):
    """Convert bit representation back to float"""
    return struct.unpack(">f", struct.pack(">I", bits))[0]


def flip_float_bit(f, position):
    """Flip a specific bit in a float"""
    bits = float_to_bits(f)
    flipped_bits = bits ^ (1 << position)
    return bits_to_float(flipped_bits)


class TCPConn:
    def __init__(self):
        # Python inits
        self.shutdown_requested = False
        self.connected = threading.Event()
        self.connected.clear()
        self.msg_queue = Queue()
        self.loc_queue = Queue()
        self.rcou_queue = Queue()
        self.mission_msg_queue = Queue()
        # GPS and Drone Status
        self.drone_ready = False  # Drone ready
        self.gps_ready = False  # GPS lock
        self.ekf_ready = False  # EKF lock
        self.drone_in_air = False
        self.rc_monitor = False  # Flag to monitor RC channel (ideally we want only after takeoff/and before landing)
        # 2025-06-18T13:35:06-0400: silipwn: Not sure if we actually are using this, so disabling for now
        # self.internal_error = False
        self.drone_state = mavutil.mavlink.MAV_STATE_UNINIT  # Initial state
        # Connection details
        with open(os.devnull, "w") as fnull:
            with redirect_stdout(fnull):
                self.conn = mavutil.mavlink_connection(
                    "tcp:localhost:5760", autoreconnect=True, retries=3
                )  # type: ignore
        self.wait_for_connection()
        # self.conn.wait_heartbeat()
        # self.connected.set()

    def setup_threads(self):
        # self.conn.wait_heartbeat()
        self.location_waiting = threading.Condition()
        # Start a thread to keep sending heartbeats
        threading.Thread(target=self.send_heartbeat, daemon=True).start()
        # Start a thread to monitor communications
        self.setup_streams()
        threading.Thread(target=self.monitor_comms, daemon=True).start()

    def setup_streams(self):
        self.conn.mav.request_data_stream_send(
            self.conn.target_system,  # target system  # type: ignore
            self.conn.target_component,  # target component  # type: ignore
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  # Stream ID
            4,  # Rate in Hz
            1,  # Start/Stop (1=start, 0=stop)
        )  # type: ignore
        # Add checks to make sure we get the data
        self.conn.recv_match(blocking=True)  # type: ignore

    # TODO: Maybe make this modular
    def apply_throttle(self, throttle_pwm=1500, duration=1.0):
        end_time = time.time() + duration
        while time.time() < end_time:
            self.conn.mav.rc_channels_override_send(
                self.conn.target_system,  # type: ignore
                self.conn.target_component,  # type: ignore
                0,  # chan1
                0,  # chan2
                throttle_pwm,  # chan3
                0,  # chan4
                0,  # chan5
                0,  # chan6
                0,  # chan7
                0,  # chan8
            )  # type: ignore
            time.sleep(0.1)
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system, self.conn.target_component, 0, 0, 0, 0, 0, 0, 0, 0  # type: ignore
        )  # type: ignore

    def wait_for_connection(self):
        # Check if we reconnected and received a heartbeat
        try:
            self.conn.wait_heartbeat()  # type: ignore
            self.connected.set()
            logger.info("Connected to the vehicle and received heartbeat.")
        except Exception as e:
            raise ConnectionError("Failed to connect to the vehicle: " + str(e))

    def reboot_and_wait_for_ack(self):
        """Reboot the vehicle by sending a command to reboot."""
        # Based on the code from the pymavlink documentation
        # We only need the normal reboot, don't care about the bootloader reboot
        param2 = 1  # To ensure we reboot normally (the autopilot only)
        self.conn.mav.command_long_send(
            self.conn.target_system,  # type: ignore
            self.conn.target_component,  # type: ignore
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            1,  # Confirmation
            param2,
            0,
            0,
            0,
            0,
            0,
            0,
        )  # type: ignore
        logger.info("Reboot command sent to the vehicle.")
        # Wait for the COMMAND_ACK message to confirm the reboot
        msg = self.conn.recv_match(type="COMMAND_ACK", blocking=True)  # type: ignore
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:  # type: ignore
            logger.info("Reboot command acknowledged by the vehicle.")

    def send_heartbeat(self):
        while self.connected.is_set() and not self.shutdown_requested:
            try:
                self.conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,  # Ground Control Station
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )  # type: ignore
                time.sleep(1)  # Sleep for a second before sending the next heartbeat
            except Exception as e:
                logger.error(f"Error in send_heartbeat: {e}")
                if not self.shutdown_requested:
                    time.sleep(1)
        logger.info("Connection closed, stopping heartbeat thread.")

    def _handle_sys_status(self, msg):
        # Basically for now, we just check enum values if the pre-arm is ready
        if msg.onboard_control_sensors_health & PREARM_CHECK:
            self.drone_ready = (
                msg.onboard_control_sensors_health & PREARM_CHECK
                and self.gps_ready
                and self.ekf_ready
            )

    def _monitor_gps_lock(self, msg):
        """This is because we need the GPS before we can start auto missions"""
        if msg.fix_type >= 3:
            # Anything greater than 3
            self.gps_ready = True

    def _monitor_ekf_lock(self, msg):
        """Ensure we have EKF getting the position"""
        if (msg.flags & EKF_POS_HORIZ) or (msg.flags & EKF_POS_VERT):
            # If we have horizontal or vertical position lock
            self.ekf_ready = True

    def _monitor_status_text(self, msg):
        # Cause this wouldn't work in case of GPS (fuzzing)
        # if re.search(r"EKF\d IMU\d is using GPS", msg.text, re.IGNORECASE):
        #     # Need this cause we need to wait till EKF is ready with GPS info
        #     logger.info("Vehicle is ready with gps_lock")
        #     self.drone_ready = True
        if re.search(r"disarm\w*", msg.text, re.IGNORECASE):
            logger.info("Mission ended, vehicle is disarmed.")
            self.drone_in_air = False
        if re.search(r"takeoff\w*", msg.text, re.IGNORECASE):
            self.drone_in_air = True
            logger.info("AUTO Mission started, takeoff.")
        if re.search(r"PreArm.*",msg.text,re.IGNORECASE):
            logger.error("PreArm check failed, vehicle is not ready for flight.")
            error_queue.put(
                {
                    "type": "fuzzer_error",
                    "error": "PreArm check failed, vehicle is not ready for flight.",
                    "component": "monitor_comms",
                    "timestamp": time.time(),
                }
            )
            self.drone_in_air = False
        if re.search(r"Mission: 1 WP", msg.text, re.IGNORECASE):
            self.drone_in_air = True
        if re.search(r".*Mission Complete.*", msg.text, re.IGNORECASE):
            logger.info("Mission ended, vehicle is disarmed.")
            self.drone_in_air = False
        elif re.search(r".*Reached destination.*", msg.text, re.IGNORECASE):
            logger.info("Reached destination , vehicle is disarmed.")
            self.drone_in_air = False
        # Handling scenario when we are in air
        if re.search(r"Mission: 2 WP", msg.text, re.IGNORECASE):
            logger.debug("Now monitoring RC channels")
            self.rc_monitor = True
            self.st_msg_send("LOG RC")
        elif re.search(r"Mission: \d+ RTL", msg.text, re.IGNORECASE):
            logger.debug("Not monitoring RC channels")
            self.rc_monitor = False
            self.st_msg_send("STOP RC")

        # NOTE: 2025-05-28T11:38:06-0400: silipwn: This causes is to stop things too early, so we disable for now
        # elif re.search(r"Mission: \d+ Land", msg.text, re.IGNORECASE):
        #     logger.debug("Not monitoring RC channels")
        #     self.rc_monitor = False
        #     self.st_msg_send("STOP RC")

        # If the drone crashed or something
        if re.search(r"hit ground\w*", msg.text, re.IGNORECASE):
            logger.info("Drone hit the ground, shutting down.")
            logger.debug("Disabling all flags")
            # self.internal_error = True
            # self.shutdown_requested = True
            self.st_msg_send("STOP RC")
            self.drone_in_air = False
            self.rc_monitor = False
            self.drone_ready = False
            self.gps_ready = False

    def monitor_comms(self):
        while self.connected.is_set() and not self.shutdown_requested:
            try:
                msg = self.conn.recv_match(blocking=True)  # type: ignore
                if msg:
                    self.msg_queue.put(msg)
                    if msg.get_type() == "STATUSTEXT":
                        logger.debug(msg.text)
                        # Crazy check because pymavlink lock doesn't work
                        self._monitor_status_text(msg)
                    if msg.get_type() == "COMMAND_ACK":
                        if msg.result is not mavutil.mavlink.MAV_RESULT_ACCEPTED:  # type: ignore
                            # Only create an error if the command was a arming/land/takeoff/auto
                            # Upload mission for rest of the commands just warn
                            if msg.command in [
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                mavutil.mavlink.MAV_CMD_NAV_LAND,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                mavutil.mavlink.MAV_CMD_MISSION_START,
                                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            ]:
                                e = f"Command failed: {msg.command} with result: {msg.result}"
                                error_queue.put(
                                    {
                                        "type": "fuzzer_error",
                                        "error": e,
                                        "component": "monitor_comms",
                                        "timestamp": time.time(),
                                    }
                                )
                                # self.internal_error = True
                                self.shutdown_requested = True
                            logger.debug(
                                f"Command failed: {msg.command} with result: {msg.result}"
                            )
                    if msg.get_type() == "GLOBAL_POSITION_INT":
                        # Update the drone's GPS location state
                        drone_loc_state = {}
                        drone_loc_state["lat"] = (
                            msg.lat / 1e7
                        )  # Convert to degrees  # type: ignore
                        drone_loc_state["lon"] = (
                            msg.lon / 1e7
                        )  # Convert to degrees  # type: ignore
                        drone_loc_state["alt"] = (
                            msg.alt / 1e3
                        )  # Convert to meters  # type: ignore
                        drone_loc_state["rel_alt"] = (
                            msg.relative_alt / 1e3  # Convert to meters  # type: ignore
                        )
                        self.loc_queue.put(drone_loc_state)
                    if (
                        msg.get_type() == "SERVO_OUTPUT_RAW"  # type: ignore
                    ):  # Only when drone is in air
                        if self.rc_monitor:
                            self.rcou_queue.put(msg.to_dict())
                    if msg.get_type() == "MISSION_REQUEST":  # type: ignore
                        self.mission_msg_queue.put(msg)
                    if msg.get_type() == "HEARTBEAT":  # type: ignore
                        self.drone_state = msg.system_status  # type: ignore
                    if msg.get_type() == "SYS_STATUS":
                        self._handle_sys_status(msg)
                    if msg.get_type() == "GPS_RAW_INT":
                        self._monitor_gps_lock(msg)
                    if msg.get_type() == "EKF_STATUS_REPORT":  # AP_Specific
                        self._monitor_ekf_lock(msg)
            except Exception as e:
                logger.error(f"Error in monitor_comms: {e}")
                error_queue.put(
                    {
                        "type": "tcp_connection_error",
                        "error": str(e),
                        "component": "monitor_comms",
                        "timestamp": time.time(),
                    }
                )
                logger.info("Connection closed, stopping monitor thread.")
                exit(0)

    def msg_recv(self, msg_type, timeout=mavlink_timeout):
        return self.conn.recv_match(type=msg_type, timeout=timeout, blocking=True)  # type: ignore

    def msg_send(self):
        msg = mavutil.mavlink.MAVLink_statustext_message()  # type: ignore
        return self.conn.mav.send(msg)  # type: ignore

    def st_msg_send(self, text):
        msg = self.conn.mav.statustext_encode(
            mavutil.mavlink.MAV_SEVERITY_INFO, text.encode()  # type: ignore
        )  # type: ignore
        self.conn.mav.send(msg)  # type: ignore

    def set_mode(self, mode):
        # Check if the mode exists in the vehicle mapping
        mode_mapping = self.conn.mode_mapping()  # type: ignore
        set_mode = mode_mapping.get(mode, None)
        if not set_mode:
            # TODO Figure out how to properly tear down everything
            logger.error("Error: Invalid mode specified")
        self.conn.mav.command_long_send(
            self.conn.target_system,  # type: ignore
            self.conn.target_component,  # type: ignore
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,  # Base mode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            set_mode,
            0,
            0,
            0,
            0,
            0,
        )  # type: ignore
        # self.conn.set_mode(set_mode)
        logger.info("Setting mode to: " + mode)

    def set_param(self, param_id, param_value, param_type="uint8"):
        """
        Set a parameter on the MAVLink-connected system.

        Args:
            param_id (str): The name/ID of the parameter to set.
            param_value (float or int): The value to set for the parameter.
            param_type (str, optional): The MAVLink parameter type (default: "uint8").

        Sends a PARAM_SET message to the target system/component with the specified parameter.
        """
        enum_types = {
            "uint8": mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
            "int8": mavutil.mavlink.MAV_PARAM_TYPE_INT8,
            "uint16": mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
            "int16": mavutil.mavlink.MAV_PARAM_TYPE_INT16,
            "uint32": mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
            "int32": mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            "float": mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            "double": mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
        }
        # TODO Might have to handle the case where have an extended parameter type
        self.conn.mav.param_set_send(
            self.conn.target_system,  # type: ignore
            self.conn.target_component,  # type: ignore
            bytes(param_id, "utf-8"),
            float(param_value),
            enum_types.get(param_type),
        )  # type: ignore

    def show_param(self, param_name, timeout=mavlink_timeout):
        """
        Get a parameter on the MAVLink-connected system.

        Args:
            param_id (str): The name/ID of the parameter to set.
            param_value (float or int): The value to set for the parameter.

        Sends a PARAM_SHOW message to the target system/component with the specified parameter.
        """
        self.conn.mav.param_request_read_send(
            self.conn.target_system,  # type: ignore
            self.conn.target_component,  # type: ignore
            bytes(param_name, "ascii"),
            -1,
        )  # type: ignore
        while True:
            msg = self.conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)  # type: ignore
            return msg

    def land(self):
        """Land the vehicle."""
        self.conn.mav.command_long_send(
            self.conn.target_system,  # type: ignore
            self.conn.target_component,  # type: ignore
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )  # type: ignore
        while True:
            loc = self.loc_queue.get(timeout=mavlink_timeout)
            if loc["rel_alt"] <= altitude_threshold:
                logger.info(
                    f"Drone reached a relative altitude: {loc['rel_alt']} meters"
                )
                self.drone_in_air = False
                break
            time.sleep(0.1)

    def rtl(self):
        """Return to Launch (RTL) the vehicle."""
        self.conn.mav.command_long_send(
            self.conn.target_system,  # type: ignore
            self.conn.target_component,  # type: ignore
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )  # type: ignore

    def arm(self):
        self.conn.mav.command_long_send(
            self.conn.target_system,  # type: ignore
            self.conn.target_component,  # type: ignore
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1 to arm
            0,
            0,
            0,
            0,
            0,
            0,
        )  # type: ignore

    def takeoff(self, altitude):
        self.conn.mav.command_long_send(
            self.conn.target_system,  # type: ignore
            self.conn.target_component,  # type: ignore
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude,
        )  # type: ignore
        # Check if the drone state is within the altitude range
        logger.debug("Waiting for location to be within the altitude range")
        while (
            True
        ):  # TODO: Change this to a case where we can timeout, in worst case scenario
            loc = self.loc_queue.get(timeout=mavlink_timeout)
            if (
                altitude - altitude_threshold
                <= loc["rel_alt"]
                <= altitude + altitude_threshold
            ):
                logger.info(f"Drone has taken off to altitude: {loc['rel_alt']} meters")
                self.drone_in_air = True
                break
            time.sleep(0.1)

    def go_to_waypoint(self, lat, lon, alt):
        """Navigate to a specified waypoint."""
        logger.info(f"Navigating to waypoint: lat={lat}, lon={lon}, alt={alt}")
        self.conn.mav.set_position_target_global_int_send(
            0,
            self.conn.target_system,  # type: ignore
            self.conn.target_component,  # type: ignore
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,  # Bitmask: enable x, y, z
            int(lat * 1e7),  # Latitude in 1e7 degrees
            int(lon * 1e7),  # Longitude in 1e7 degrees
            alt,  # Altitude in meters
            0,
            0,
            0,  # x, y, z velocity
            0,
            0,
            0,  # x, y, z acceleration
            0,
            0,  # yaw, yaw rate
        )  # type: ignore
        while True:
            loc = self.loc_queue.get(timeout=mavlink_timeout)
            if (lat - approx_threshold <= loc["lat"] <= lat + approx_threshold) and (
                lon - approx_threshold <= loc["lon"] <= lon + approx_threshold
            ):
                logger.info(f"Reached waypoint: lat={loc['lat']}, lon={loc['lon']}")
                break

    def cleanup(self, shutdown=True):
        if shutdown:
            self.shutdown_requested = shutdown
            rcou_list = []
            logger.info(f"{self.msg_queue.qsize()} messages in the queue")
            while not self.rcou_queue.empty():
                rcou_list.append(self.rcou_queue.get())
            time.sleep(0.5)  # Give some time for the threads to finish
            # Clear all the queues
            while not self.msg_queue.empty():
                self.msg_queue.get()
            while not self.loc_queue.empty():
                self.loc_queue.get()
            if rcou_list:
                logger.info(f"Received {len(rcou_list)} RC channel updates.")
            return rcou_list
        if self.conn:
            self.conn.close()  # type: ignore
            logger.info("TCP connection closed.")
        self.connected.clear()


def get_enum(root, enum_name):
    """
    Find and return a list of enum values for the given enum name.
    Returns a dictionary with entry names as keys and their values and descriptions.
    """
    enum_elements = root.xpath(f"//enum[@name='{enum_name}']")
    if not enum_elements:
        print(f"Enum '{enum_name}' not found")
        return None

    enum_values = []
    for enum in enum_elements:
        for entry in enum.xpath(".//entry"):
            # name = entry.get("name")
            value = entry.get("value")
            # description = entry.xpath("./description")
            # desc_text = description[0].text if description else "No description"
            enum_values.append(value)

    return enum_values


def include_xml(elem, base_path, processed_files=None):
    if processed_files is None:
        processed_files = set()

    for include in elem.xpath(".//include"):
        filename = include.text
        filepath = os.path.join(base_path, filename)
        # Check if the file has already been processed
        if filepath in processed_files:
            # print(f"Skipping already processed file: {filepath}")
            continue

        if os.path.exists(filepath):
            parser = etree.XMLParser(remove_blank_text=True)
            include_tree = etree.parse(filepath, parser)
            include_root = include_tree.getroot()

            # Mark the file as processed
            processed_files.add(filepath)

            # Recursively process includes in the included file
            include_xml(include_root, os.path.dirname(filepath), processed_files)

            # Replace the include element with the contents of the inzcluded file
            parent = include.getparent()
            index = parent.index(include)
            parent.remove(include)
            for child in reversed(list(include_root)):
                parent.insert(index, child)


def load_xml_messages(file_path: str, filter_list: list) -> list:
    """
    Parse the XML at file_path, include any referenced XML via include_xml(),
    and return a list of messages (matching filter_list) as dicts:
      {
        "msg_id": ...,
        "msg_name": ...,
        "fields": [
          { "name": ..., "type": ..., "desc": ..., "enum_vals": [...]? },
          ...
        ]
      }
    """
    # 1) Parse & include
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(file_path, parser)
    root = tree.getroot()
    include_xml(root, os.path.dirname(os.path.abspath(file_path)))

    # 2) Prep filter set & enum cache
    filter_set = set(filter_list)
    enum_cache = {}
    messages = []

    # 3) Grab all <message> and <entry> elements
    msg_elements = root.findall(".//messages/message") + root.findall(".//entry")

    for msg in msg_elements:
        msg_name = msg.get("name")
        if msg_name not in filter_set:
            continue

        msg_id = msg.get("id") or msg.get("value")
        logging.debug(f"Loading message {msg_name} (id={msg_id})")

        # 4) Collect <field> children up to <extensions/>
        fields = []
        saw_field = False
        for child in msg:
            if child.tag == "extensions":
                break
            if child.tag == "field":
                saw_field = True
                name = child.get("name")
                typ = child.get("type")
                desc = (child.text or "").strip()
                units = child.get("units", "")
                enum_name = child.get("enum")
                entry = {"name": name, "type": typ, "desc": desc, "units": units}

                # cache & attach enum values if present
                if enum_name:
                    if enum_name not in enum_cache:
                        enum_cache[enum_name] = get_enum(root, enum_name) or []
                    if enum_cache[enum_name]:
                        entry["enum_vals"] = enum_cache[enum_name]

                fields.append(entry)

        # 5) If no pre-extension fields, fall back to <param>
        if not saw_field:
            for param in msg.findall(".//param"):
                label = param.get("label")
                if not label:
                    continue

                entry = {"name": label}
                desc_text = (param.text or "").strip().strip(".")

                if param.get("enum"):
                    enum_name = param.get("enum")
                    # Ensure get_enum returns list of values. These are typically numeric strings.
                    enum_values_str = get_enum(root, enum_name)

                    entry["desc"] = f"{desc_text} (Enum: {enum_name})"
                    if enum_values_str:
                        try:
                            # Convert enum values to int, as they are typically numeric in MAVLink
                            entry["enum_vals"] = [int(ev) for ev in enum_values_str]
                            entry["type"] = "enum"  # Unified type for enums
                        except ValueError:
                            logger.warning(
                                f"Enum '{enum_name}' for param '{label}' contains non-integer values. Storing as strings."
                            )
                            entry["enum_vals"] = (
                                enum_values_str  # Store as strings if not all int
                            )
                            entry["type"] = (
                                "enum_str"  # Indicate string enum if necessary
                            )
                    else:
                        entry["desc"] += " - Enum not found or empty"
                        entry["type"] = "unknown_enum_param"  # Or handle as error
                else:
                    # Handle range-based params (typically float for MAVLink params)
                    mn_str = param.get("minValue")
                    mx_str = param.get("maxValue")
                    # inc_str = param.get("increment") # Increment not directly used by random.uniform
                    units = param.get("units", "")

                    entry["desc"] = desc_text
                    if units:
                        entry["desc"] += f" (Units: {units})"

                    try:
                        # Params are often floats. Use defaults if min/max are not specified.
                        entry["range_min"] = (
                            float(mn_str) if mn_str is not None else 0.0
                        )  # Wider default range
                        entry["range_max"] = (
                            float(mx_str) if mx_str is not None else 360.0
                        )
                        entry["type"] = "param_range_float"
                    except (ValueError, TypeError):
                        logger.warning(
                            f"Could not parse minValue/maxValue for param '{label}' as float. Using default range."
                        )
                        entry["range_min"] = 0.0
                        entry["range_max"] = 360.0
                        entry["type"] = "param_range_float"  # Default to float range

                fields.append(entry)

        messages.append({"msg_id": msg_id, "msg_name": msg_name, "fields": fields})

    return messages


def generate_field_value(
    field_type, field_desc=None, field_units=None, field_range=[None, None, None]
):
    """
    Generate a random value for a field based on its type.
    The idea is to be able to actually parse the field_description
    and generate more relevant inputs

    Returns:
        Values to use inside the message
    """
    # First recurse if need be
    if "[" in field_type:
        generated_value = []
        # Extract the base type and array size
        match = re.search(r"\[(\d+)\]", field_type)
        if match:
            field_size = int(match.group(1))
        else:
            raise ValueError(
                f"field_type '{field_type}' does not contain a size in brackets"
            )
        field_type = field_type.split("[")[0]
        for _ in range(field_size):
            generated_value.append(
                generate_field_value(field_type, field_desc, field_units, field_range)
            )
        if field_type == "char":
            # Ensure we convert the list of integer into bytes
            generated_value = bytes(generated_value)
        return generated_value
    # Let's now check the description and units
    # unit_lookup = {
    #     "A": [],
    #     "bytes": [],
    #     "cA": [],
    #     "cdeg": [],
    #     "cdegC": [],
    #     "cV": [],
    #     "d%": [],
    #     "deg": [],
    #     "degC": [],
    #     "degE7": [],
    #     "deg/s": [],
    #     "ds": [],
    #     "m": [],
    #     "mAh": [],
    #     "mgauss": [],
    #     "mm": [],
    #     "m/s": [],
    #     "ms": [],
    #     "m/s/s": [],
    #     "mV": [],
    #     "Pa": [],
    #     "rad": [],
    #     "rad/s": [],
    #     "rpm": [],
    #     "s": [],
    #     "us": [],
    #     "V": [],
    #     "%": [],
    #     "Ah": [],
    #     "bits/s": [],
    #     "bytes/s": [],
    #     "c%": [],
    #     "cdeg/s": [],
    #     "cm": [],
    #     "cm^2": [],
    #     "cm^3": [],
    #     "cm^3/min": [],
    #     "cm/s": [],
    #     "cs": [],
    #     "dam": [],
    #     "dB": [],
    #     "deg/2": [],
    #     "degE5": [],
    #     "dm": [],
    #     "dm/s": [],
    #     "dpix": [],
    #     "g": [],
    #     "gauss": [],
    #     "hJ": [],
    #     "hPa": [],
    #     "Hz": [],
    #     "kg": [],
    #     "KiB/s": [],
    #     "kPa": [],
    #     "mG": [],
    #     "MiB": [],
    #     "MiB/s": [],
    #     "mrad/s": [],
    #     "m/s*5": [],
    #     "ns": [],
    #     "pix": [],
    #     "W": [],
    #     "mbar": [],
    #     "mm/s": [],
    # }
    # Check if we have a valid range for the values
    if field_range[0] is not None and field_range[1] is not None:
        if field_type in ["float", "double"]:
            return random.uniform(field_range[0], field_range[1])
        elif field_range[2] is not None:
            # NOTE: Always only generates integer
            return random.randrange(field_range[0], field_range[1], field_range[2])
        else:
            return random.randint(field_range[0], field_range[1])
    else:
        # Now check if we have min/max values
        if field_type.startswith("uint8"):
            return random.randint(0, 255)
        elif field_type.startswith("uint16"):
            return random.randint(0, 65535)
        elif field_type.startswith("uint32"):
            return random.randint(0, 4294967295)
        elif field_type.startswith("int8"):
            return random.randint(-128, 127)
        elif field_type.startswith("int16"):
            return random.randint(-32768, 32767)
        elif field_type.startswith("int32"):
            return random.randint(-2147483648, 2147483647)
        elif field_type.startswith("float"):
            return random.uniform(-10, 10)
        elif field_type.startswith("char"):
            return random.randint(0, 255)
        else:
            return 0


class FuzzState(Enum):
    Init = 0
    Bitflip = 1
    Arithmetic = 2
    Interest = 3
    # TODO Havoc


@dataclass(order=True)
class PriorityQueueEntry:
    """An entry in the priority queue, sorted by priority."""

    priority: int
    data: Any = field(compare=False)
    metadata: dict = field(default_factory=dict, compare=False)

    def __repr__(self):
        # A more compact representation for logging
        return f"Entry(priority={self.priority}, data={self.data!r})"


class FuzzConfig:
    def __init__(self, args):
        """Initialize the FuzzConfig object with the provided arguments.

        Args:
            args: Command-line arguments or configuration settings.
        """
        # Register signal handlers
        # signal.signal(signal.SIGINT, self.signal_handler)
        # signal.signal(signal.SIGTERM, self.signal_handler)
        self.fuzzer_shutdown_requested = False  # Handles the entire fuzzer shutdown

        # Load configuration from config YAML file first
        self.config_file = args.config if args.config else None
        self.config = {}
        if self.config_file and os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                self.config = yaml.safe_load(f)
                logger.info(f"Loaded configuration from {self.config_file}")
        self.fuzzer_state = FuzzState.Init
        self.fuzzer_queue = []
        # Or queue.Queue (if we have multiple producers)
        # Just check if the file contains at least ap_dir and peripheral_file
        if not self.config.get("ap_dir") or not self.config.get("peripheral_file"):
            raise ValueError(
                "Atleast SITL binary and peripheral_file are required in the config file."
            )

        # Load peripheral mapping from peripheral YAML file
        self.peripheral_file = (
            args.peripheral_file
            if args.peripheral_file
            else self.config.get("peripheral_file")
        )
        self.peripheral_mapping = {}
        if self.peripheral_file and os.path.exists(self.peripheral_file):
            with open(self.peripheral_file, "r") as f:
                self.peripheral_mapping = yaml.safe_load(f)
                logger.info(f"Loaded peripheral mapping from {self.peripheral_file}")

        # Allow configuring the modes via config
        # Current fallbacks are ['GUIDED', 'AUTO'], I think these are the most common
        self.supported_modes = self.config.get("supported_modes") or ["GUIDED", "AUTO"]

        self.vehicle = (
            args.vehicle if args.vehicle else self.config.get("vehicle", "copter")
        )
        # Select the model that the oracle uses
        self.oracle_model = self.config.get("oracle_model", "dtw")
        logger.info(f"Using oracle model: {self.oracle_model}")
        if self.oracle_model not in detection_models:
            logger.error("Unknown oracle model specified, using default: dtw")
            self.oracle_model = "dtw"

        # Setup files - command line args override yaml config
        self.sitl_bin = args.bin if args.bin else self.config.get("sitl_bin", None)

        self.ap_dir = (
            args.ap_dir if args.ap_dir else self.config.get("ap_dir", "/ardupilot")
        )
        # Handle the case where we don't have a SITL binary
        if self.sitl_bin is None:
            # Check if we have the binary at ap_dir + build/sitl/bin/ardu + vehicle
            vehicle_bin = f"ardu{self.vehicle}"
            sitl_bin_path = os.path.join(
                self.ap_dir, "build", "sitl", "bin", vehicle_bin
            )
            if file_exists(sitl_bin_path):
                self.sitl_bin = sitl_bin_path
            else:
                raise FileNotFoundError(
                    "SITL binary not found. Please provide a valid path."
                )

        # Handle the case where we don't have a SITL binary
        if self.sitl_bin is None:
            # Check if we have the binary at ap_dir + build/sitl/bin/ardu + vehicle
            vehicle_bin = f"ardu{self.vehicle}"
            sitl_bin_path = os.path.join(
                self.ap_dir, "build", "sitl", "bin", vehicle_bin
            )
            if file_exists(sitl_bin_path):
                self.sitl_bin = sitl_bin_path
            else:
                raise FileNotFoundError(
                    "SITL binary not found. Please provide a valid path."
                )
        self.xml_file = args.xml if args.xml else self.config.get("xml_file")

        # Auto mission configuration
        self.auto_mission_enabled = False
        self.auto_mission_path = (
            args.auto_mission if args.auto_mission else self.config.get("mission_file")
        )
        if self.auto_mission_path and file_exists(self.auto_mission_path):
            logger.info("Mission file found, AUTO mode testing enabled")
            self.auto_mission_enabled = True

        # Variables - command line args override yaml config
        self.peripheral_under_test = (
            args.peripheral if args.peripheral else self.config.get("peripheral")
        )

        # Mission control
        # We consider 300 seconds to be a reasonable timeout for the mission
        self.timeout = self.config.get("timeout", 600)  # 10 minutes for now
        # We consider 300 seconds to be a reasonable timeout for the mission
        self.timeout = self.config.get("timeout", 600)  # 10 minutes for now
        # MAVLink check for https://mavlink.io/en/guide/routing.html
        self.target_system = self.config.get("target_system", 0)
        self.target_component = self.config.get("target_component", 0)
        logger.debug("Setting the system.target_system to: " + str(self.target_system))
        logger.debug(
            "Setting the system.target_component to: " + str(self.target_component)
        )

        # Validate required files
        if file_exists(self.sitl_bin) and file_exists(self.ap_dir):
            logger.info(f"Using SITL binary: {self.sitl_bin}")
            logger.info(f"Using Ardupilot directory: {self.ap_dir}")

        # Parameter file
        # Now we have a parameter dictionary
        param_mapping = {
            "copter": "copter.parm",
            "plane": "plane-jsbsim.parm",
            "rover": "rover.parm",
        }
        self.param_file = os.path.join(
            self.ap_dir, "Tools/autotest/default_params/", param_mapping.get(self.vehicle, "None")
        )
        if file_exists(self.param_file):
            logger.info("Using parameter file: " + self.param_file)

        # Calibration settings
        self.calibration_active = False
        self.calibration_threshold = None
        self.calibration_vals = None
        calibration_rounds = (
            args.calibration_rounds
            if args.calibration_rounds
            else self.config.get("calibration_rounds", 10)
        )
        self.calibration_rounds = calibration_rounds
        # 2025-06-14T10:09:17-0400: silipwn: Why are we setting this to 25? For now that's enough
        self.fuzzer_queue_len = 25
        # Check if we have calibration values
        # Should be stored as [min,max]
        self.calibration_threshold = self.config.get("calibration_threshold") or None

        # Fuzzing related attributes
        self.xml_messages = []
        self.rcou_vals = []
        self.start_time = time.time()  # Rough estimate only
        self.fuzzing_active = False
        self.fuzzing_thread = None
        if self.calibration_threshold:
            self.min_fuzz_threshold = self.calibration_threshold[0]
            self.max_fuzz_threshold = self.calibration_threshold[1]
            rcou_file_path = os.getcwd() + "/rcou_vals.pkl"
            try:
                with open(rcou_file_path, "rb") as f:
                    self.golden_rc_vals = pickle.load(f)
            except FileNotFoundError:
                raise FileNotFoundError(
                    "RC channel values file not found, using empty list"
                )
            assert (
                self.golden_rc_vals
            ), "Golden RC values must be provided when re-using calibrations"
        else:
            self.min_fuzz_threshold = None
            self.max_fuzz_threshold = None
            self.golden_rc_vals = []
        self.sim_ready = False
        self.fuzz_interval = self.config.get(
            "fuzz_interval", 0.5
        )  # Send a fuzzed message every 0.5 seconds
        logger.debug("The fuzz interval is set to: " + str(self.fuzz_interval))
        self.msg_freq = None
        self.fuzz_msgs = []

        # Initialize fuzzer
        self.fuzzer_param_file = None
        assert args.fuzzer_temp_dir, "Temporary directory must be provided"
        self.fuzzer_temp_dir = args.fuzzer_temp_dir
        self.fuzzer_temp_input_dir = self.fuzzer_temp_dir + "/input"
        # Create the temporary input directory if it doesn't exist
        if not os.path.exists(self.fuzzer_temp_input_dir):
            os.makedirs(self.fuzzer_temp_input_dir)
            logger.info(
                f"Created temporary input directory: {self.fuzzer_temp_input_dir}"
            )
        self.setup()
        if self.msg_freq:
            logger.info("Setting the fuzzing interval to match message frequency")
            self.fuzz_interval = self.msg_freq

    def periodic_send(self, frequency, xml_msg, default_values):
        """Send periodic messages based on the specified frequency.

        Args:
            frequency: Frequency at which messages should be sent.
            xml_msg: XML message definition.
            default_values: Default values for the message fields.
        """
        logger.info(f"Starting periodic send for {xml_msg} every {frequency} Hz")
        while True:
            if not self.fuzzing_active and self.sim_ready:
                try:
                    # Convert default values to a dict
                    default_values_dict = {
                        field["name"]: default_values[i]
                        for i, field in enumerate(xml_msg["fields"])
                    }
                    self.send_fuzzed_message(
                        xml_msg["msg_name"], xml_msg["msg_id"], default_values_dict
                    )
                except Exception as e:
                    error_queue.put(
                        {
                            "type": "fuzzer_error",
                            "error": e,
                            "component": "periodic_send",
                            "timestamp": time.time(),
                        }
                    )
                    self.fuzzer_shutdown_requested = True

                time.sleep(1 / frequency)

    def setup(self):
        """Setup the fuzzing configuration and initialize parameters."""
        # Setup the random seed for reproducibility
        random.seed(RANDOM_SEED)

        self.fuzzer_stats = {
            "simulations_completed": 0,
            "messages_sent": 0,
            "last_mission_time": 0.0,
            "current_mission_time": 0.0,
            "dtw_threshold": [],
            "potential_crashes": 0.0,
        }

        # Get the peripheral mapping for the selected peripheral
        if self.peripheral_mapping and "sensors" in self.peripheral_mapping:
            self.peripheral_config = self.peripheral_mapping["sensors"].get(
                self.peripheral_under_test, {}
            )
            logger.info(
                f"Using peripheral mapping for {self.peripheral_under_test}: {self.peripheral_config}"
            )
        else:
            raise Exception(
                f"No peripheral mapping found for {self.peripheral_under_test}"
            )

        # Get the directory for the script and check the git log for the version
        src_commit_hash = (
            subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=self.ap_dir)
            .strip()
            .decode("utf-8")
        )
        logger.debug("Source Under Testing commit hash %s", src_commit_hash)

        # Check if we have additional parameters in the peripheral mapping
        self.default_parameter_set = self.peripheral_mapping.get("generic_params", {})
        if not self.default_parameter_set:
            logger.debug("Don't have any generic parameters to set")

        # Find the filter from the peripheral mapping
        msg_filter = self.peripheral_config.get("msg_type", [])
        logger.info(f"Using message filter: {msg_filter}")

        # Check if we have CMD messages enabled
        if self.peripheral_config.get("cmd_msgs"):
            # Add the command messages to the filter
            msg_filter += self.cmd_params()

        # Check if we have some default calibration messages
        self.calibration_msg = self.peripheral_config.get("calibration_msg", [])
        if self.peripheral_config.get("calibration_msgs"):
            logger.debug("Using default calibration messages")

        # Load XML message definitions if provided
        if self.xml_file and os.path.exists(self.xml_file) and msg_filter:
            logger.info(f"Loading MAVLink message definitions from {self.xml_file}")
            self.xml_messages = load_xml_messages(self.xml_file, filter_list=msg_filter)
            logger.info(f"Loaded {len(self.xml_messages)} message definitions")
        # Check if the supplied length of msg_type and self.xml_messages match
        if len(msg_filter) != len(self.xml_messages):
            logger.error("Couldn't find all the XML messages exiting")
            exit(-1)
        # Check if the peripheral mapping has a frequency associated with it
        # If each peripheral has a frequency, spawn a new thread for each peripheral
        self.periodic_thread = {}
        self.default_msg = {}

        # Iterate through each msg_type and check the associated frequency
        selected_msgs = self.peripheral_config.get("msg_type", [])
        selected_freq = self.peripheral_config.get("frequency", [])
        logger.info(f"Selected messages for fuzzing: {selected_msgs}")
        logger.info(f"Selected frequencies for fuzzing: {selected_freq}")
        if not selected_msgs:
            logger.warning(
                "No messages selected for fuzzing, please check the peripheral mapping"
            )
            return
        # For now: Ideally only select a single message for fuzzing
        final_msg = random.choice(selected_msgs)
        extracted_msg = []
        # Only have the final_msg inside the self.xml_messages
        for msg in self.xml_messages:
            if msg["msg_name"] == final_msg:
                extracted_msg = [msg]
        self.xml_messages = extracted_msg
        logger.info("Selecting fuzzing message: " + str(final_msg))
        # Ensure we use this everywhere else
        for msg in selected_msgs:
            # Get the frequency from the same index
            msg_idx = selected_msgs.index(msg)
            try:
                msg_freq = selected_freq[msg_idx] if msg else -1
            except IndexError:
                logger.warning(
                    "No corresponding frequency found for the message, assuming None (aka -1)"
                )
                msg_freq = -1
            logger.info(f"Message: {msg}, Frequency: {msg_freq}")
            xml_msg = load_xml_messages(self.xml_file, filter_list=[msg])
            # We will get a list of frequencies and then spawn a thread for each peripheral
            if msg_freq != -1 and xml_msg:
                try:
                    self.default_msg[msg_idx] = self.peripheral_config.get(
                        "default_msg", {}
                    )
                    logger.info(
                        f"Default message sent at {msg_freq}: {self.default_msg[msg_idx]}"
                    )
                    self.msg_freq = msg_freq
                except KeyError:
                    logger.error(
                        "Can't find default msg for frequency, Not spawning threads"
                    )
                    return

                self.periodic_thread[msg_freq] = threading.Thread(
                    target=self.periodic_send,
                    args=(msg_freq, xml_msg[0], self.default_msg[msg_idx]),
                    daemon=True,
                )
                self.periodic_thread[msg_freq].start()
                logger.info("Periodic thread started")

        # Also if we have a parameter file, create a temporary one and send it to the simulator
        if self.peripheral_config.get("parameters"):
            self.fuzzer_param_file = tempfile.mkstemp(".parm", "dronefuzz", "/tmp")[1]
            with open(self.fuzzer_param_file, "w") as f:
                for parameter, values in self.peripheral_config["parameters"].items():
                    f.write(f"{parameter} {values}\n")

    # Fuzzer Queue management functions
    def add_to_fuzz_queue(self, fuzz_msg, score, metadata=None):
        """
        Adds a new input to the queue with an associated score.
        Higher scores are prioritized.
        """
        # We use -score because heapq is a min-heap, so a smaller
        # number has a higher priority. This makes higher scores pop first.
        priority = -score
        entry = PriorityQueueEntry(priority=priority, data=fuzz_msg, metadata=metadata)
        heapq.heappush(self.fuzzer_queue, entry)

    def get_next_in_fuzz_queue(self) -> PriorityQueueEntry:
        """
        Gets a copy of highest priority item from the queue.
        Peeks the item
        """
        # heappop returns the item with the smallest priority (-score)
        if len(self.fuzzer_queue) > 0:
            return self.fuzzer_queue[0]  # Peek at the highest priority item
        else:
            # Create the required error message for the queue
            error = "Fuzz queue is empty, cannot get next item"
            error_queue.put(
                {
                    "type": "fuzzer_error",
                    "error": error,
                    "component": "mutator",
                    "timestamp": time.time(),
                }
            )
            return PriorityQueueEntry(priority=0, data=None)

    def sort_fuzz_queue(self):
        """
        Returns a fully sorted list of all items in the queue
        by their original score (highest first).
        """
        # The heap itself is not fully sorted, only guaranteed that the first
        # element is the smallest. sorted() will create a new sorted list.
        return sorted(self.fuzzer_queue, reverse=True)

    def sim_params(self):
        """Add the SIM parameters from the PGFUZZ database."""
        # TODO
        pass

    def cmd_params(self):
        """Add the mission parameters from the PGFUZZ database.

        Returns:
            List of command parameters.
        """
        cmds = ["MAV_CMD_DO_SET_MODE"]
        return cmds

    def random_param_set(self):
        """Randomly set a parameter set for fuzzing."""
        if not self.default_parameter_set:
            logger.warning(
                "No default configuration parameters set for fuzzing, returning"
            )
            return
        selected_param = random.choice(self.default_parameter_set)
        # Check if the value ends in DISABLE or ENABLE, then we set it 0 or 1
        if re.match(r".*ABLE$", selected_param):
            random_val = random.randint(0, 1)
        else:
            random_val = generate_field_value(
                field_type="uint8"
            )  # Default to int8 for now
        self.tcp_conn.set_param(
            param_id=selected_param,
            param_value=random_val,
        )
        self.fuzz_msgs.append(
            [
                time.time() - self.fuzzer_stats["current_mission_time"],
                "PARAM_SET",
                {
                    "param_id": selected_param,
                    "param_value": random_val,
                },
            ]
        )
        self.fuzzer_stats["messages_sent"] += 1

    def _monitor_sim(self):
        """Monitor the SITL simulation for messages and events."""
        # If sim_handle errors out, inform main thread
        # Set the start_time once sim is ready
        assert self.sim_ready is True, "Simulation must be ready before monitoring"
        start_time = time.time()
        # Monitor the sim_handle and check if we have exited
        # TODO: Eventually also check if went beyond average time
        while time.time() - start_time < self.timeout and self.sim_ready:
            ret_val = self.sim_handle.poll()
            # Get the signal number
            if ret_val is not None:
                if ret_val != 0:
                    signal_num = -ret_val  # Because this is a negative value
                    signal_name = "UNKNOWN"
                    try:
                        import signal

                        signal_name = signal.Signals(signal_num).name
                    except ValueError:
                        # Nothing doing :)
                        pass
                    error = f"SITL simulation exited with signal: {signal_name} ({signal_num})"
                    error_queue.put(
                        {
                            "type": "sitl_terminated_error",
                            "error": error,
                            "component": "monitor_sim",
                            "timestamp": time.time(),
                        }
                    )
                else:
                    # If the return value is 0, it means the process exited cleanly, let's exit the loop
                    logger.debug("SITL simulation exited cleanly")
                    exit(0)
            time.sleep(1)  # Check every second
        if self.sim_ready is False:
            # That means we have stopped the simulation
            exit(0)
        # Else we have timed out
        logger.info("SITL simulation timed out")
        error = "SITL timeout error"
        error_queue.put(
            {
                "type": "sitl_timeout_error",
                "error": error,
                "component": "monitor_sim",
                "timestamp": time.time(),
            }
        )
        # Update relevant stats
        self.fuzzer_stats["current_mission_time"] = time.time() - start_time
        exit(0)

    def run_sim(self):
        """Run the SITL simulation with the specified vehicle and parameters."""
        sitl_args = ""
        if self.vehicle == "copter":
            sitl_args = " -S --model + -w --speedup 1 -I0"
        elif self.vehicle == "plane":
                # "-w" "-S" "--home" "-35.362938,149.165085,585,354" "--model" "plane-elevrev"  "--defaults" "/Tools/autotest/default_params/plane-jsbsim.parm"
            sitl_args = " -S --model plane-elevrev -w --speedup 1 -I0"
        elif self.vehicle == "rover":
            # "-w" "-S" "--home" "40.071375,-105.229789,1583,246" "--model" "rover" 
            sitl_args = " -S --model rover -w --speedup 1 -I0"
        self.sitl_cmd = self.sitl_bin + sitl_args + " --defaults " + self.param_file
        logger.info(f"Starting SITL with command: {self.sitl_cmd}")
        if self.calibration_active:
            assert (
                self.fuzzing_active is False
            ), "Cannot run calibration while fuzzing is active"
        if self.fuzzer_param_file:
            self.sitl_cmd += "," + self.fuzzer_param_file
        # Handle the case where vehicle is plane and we need to ensure it lands
        try:
            self.sim_handle = subprocess.Popen(
                ["bash", "-c", self.sitl_cmd],
                # Comment out to debug the original binary
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=False,
                preexec_fn=os.setsid,
                cwd=self.fuzzer_temp_dir,
            )
            init_conn = TCPConn()
            # Reboot to ensure we have reloaded the parameters
            init_conn.reboot_and_wait_for_ack()
            time.sleep(2)  # Give some time for the reboot to complete
            init_conn.cleanup(shutdown=False)
            self.fuzzer_stats["current_mission_time"] = time.time()
            self.tcp_conn = TCPConn()
            self.tcp_conn.setup_threads()
            self.sim_ready = True
            # Start the monitoring thread
            self.monitor_thread = threading.Thread(
                target=self._monitor_sim, daemon=True
            ).start()
        except Exception as e:
            # this would just kill the entire script, so need to handle it gracefully
            logger.error(f"Error starting simulation: {e}")

    def monitor_auto_mission(self):
        """Monitor the drone during an automatic mission."""
        # Wait till the drone is in air
        logger.info("Waiting till drone is in air")
        # self.wait_for_condition(lambda: self.tcp_conn.rc_monitor, timeout=60)
        try:
            while not self.tcp_conn.rc_monitor:
                if self.error_sleep(1):
                    raise InternalError
            self.start_fuzzing()
            mode_ctr = 0
            prev_state = None
            MAX_MODE_CHANGES = 1
            # self.wait_for_condition(lambda: self.tcp_conn.drone_in_air and self.tcp_conn.drone_in_air, timeout=60)
            # BREAKPOINT: 2025-05-26T12:41:06-0400: silipwn: This is where we stop for now, need to figure out how to actually handle this
            # while self.wait_for_condition(
            #     lambda: self.tcp_conn.drone_in_air and self.tcp_conn.drone_in_air,
            #     timeout=60,
            # ):
            while self.tcp_conn.rc_monitor and self.tcp_conn.drone_in_air:
                mode = random.choice(self.supported_modes)
                if (
                    mode_ctr < MAX_MODE_CHANGES  # Randomly set a mode
                ):  # 2025-05-26T15:41:06-0400: silipwn: To ensure we only change modes couple of times
                    self.tcp_conn.set_mode(mode)
                    logger.debug(f"Changing mode to: {mode}")
                    mode_ctr += 1
                    prev_state = mode
                elif mode_ctr >= MAX_MODE_CHANGES and prev_state != "AUTO":
                    logger.debug("Reached mode change limit, not changing mode anymore")
                    self.tcp_conn.set_mode("AUTO")
                    prev_state = "AUTO"
                    logger.debug("Resetting Setting mode to AUTO")
                if random.random() < 0.1:  # Randomly set a parameter
                    self.random_param_set()
                if self.error_sleep(3):
                    raise InternalError
            self.wait_for_condition(lambda: not self.tcp_conn.rc_monitor, timeout=60)
            while self.tcp_conn.drone_in_air:
                if self.error_sleep(1):
                    raise InternalError
            self.stop_fuzzing()
            return
        except InternalError:
            logger.error("Warning detected, stopping fuzzing")
            if self.fuzzing_active:
                self.stop_fuzzing()
            return

    def upload_auto_mission(self, mission_file):
        """Upload a mission from a waypoint file using MAVProxy's waypoint module.

        Args:
            mission_file: Path to the mission file (.waypoints format).
        """
        waypoints = mavwp.MAVWPLoader()
        _ = waypoints.load(mission_file.strip('"'))

        # Clear any existing mission
        self.tcp_conn.conn.waypoint_clear_all_send()  # type: ignore

        # Send waypoint count
        self.tcp_conn.conn.waypoint_count_send(waypoints.count())  # type: ignore

        # Respond to mission requests
        for _ in range(waypoints.count()):
            try:
                # Wait for mission request message
                msg = self.tcp_conn.mission_msg_queue.get(timeout=4)

                logger.info(f"Received MISSION_REQUEST for sequence {msg.seq}")

                # Send the requested waypoint
                self.tcp_conn.conn.mav.send(waypoints.wp(msg.seq))  # type: ignore
                logger.info(f"Sending waypoint {msg.seq}")

            except Exception as e:
                logger.error(f"Error in mission upload: {e}")

    def standard_guided(self, fuzzing=True):
        """Perform a standard guided mission.

        Args:
            fuzzing: Whether to enable fuzzing during the mission.
        """
        self.tcp_conn.set_mode("GUIDED")
        self.tcp_conn.arm()
        # Monitor
        if self.vehicle != "rover":
            self.tcp_conn.takeoff(50)

        # Start fuzzing after takeoff
        if fuzzing:
            self.start_fuzzing()

        self.tcp_conn.rc_monitor = True
        self.tcp_conn.st_msg_send("LOG RC")
        # self.tcp_conn.
        self.tcp_conn.go_to_waypoint(-35.3632621, 149.1652374, 50)
        # Go to Point B -35.3626941, 149.166221
        self.tcp_conn.go_to_waypoint(-35.3626941, 149.166221, 50)
        # Loiter for a while
        mode = random.choice(self.supported_modes)
        self.tcp_conn.set_mode(mode)
        # Run a while loop for 10 seconds
        for _ in range(10):
            if random.uniform(0, 1) > 0.5:  # Randomly set a parameter
                self.random_param_set()
            self.error_sleep(1)
        self.tcp_conn.set_mode("GUIDED")
        # -35.362839699999995, 149.1646279,
        self.tcp_conn.go_to_waypoint(-35.362839699999995, 149.1646279, 50)
        # Go to point X -35.3632621, 149.1652374,
        self.tcp_conn.go_to_waypoint(-35.3632621, 149.1652374, 50)

        # Stop fuzzing before landing
        if fuzzing:
            self.stop_fuzzing()

        # Land
        self.tcp_conn.rc_monitor = False
        self.tcp_conn.st_msg_send("STOP RC")
        self.tcp_conn.rtl()

        while self.tcp_conn.drone_in_air:
            self.error_sleep(1)  # Not raising an error as the main thread
        return

    def send_mission(self, fuzzing=True):
        """Send a mission to the drone.

        Args:
            fuzzing: Whether to enable fuzzing during the mission.
        """
        if self.auto_mission_enabled:
            self.upload_auto_mission(mission_file=self.auto_mission_path)
            # ARM to AUTO and then
            self.tcp_conn.arm()
            self.tcp_conn.set_mode("AUTO")
            self.tcp_conn.apply_throttle()
            self.monitor_auto_mission()
        else:
            logger.info("Using standard triangle mission")
            self.standard_guided(fuzzing=fuzzing)
        logger.info("Finished mission")
        logger.debug(f"Sent {self.fuzzer_stats['messages_sent']} messages")
        self.fuzzer_stats["simulations_completed"] += 1
        self.fuzzer_stats["last_mission_time"] = (
            time.time() - self.fuzzer_stats["current_mission_time"]
        )

    def signal_handler(self, _signum, _frame):
        """Handle shutdown signals gracefully.

        Args:
            _signum: Signal number.
            _frame: Current stack frame.
        """
        logger.error("\nShutdown requested...")
        logger.error("Will terminate after current execution\n")
        self.fuzzer_shutdown_requested = True
        # Figure out a better way to terminate things and exit quickly

    def cleanup_and_exit(self):
        """Perform cleanup and print summary before exit."""
        logger.info("Performing cleanup...")

        # Stop fuzzing first
        self.stop_fuzzing()

        # Cleanup TCP connection
        if hasattr(self, "tcp_conn") and self.tcp_conn:
            self.tcp_conn.cleanup()

        # Terminate simulation
        if hasattr(self, "sim_handle") and self.sim_handle:
            try:
                self.sim_handle.terminate()
                self.sim_handle.wait(timeout=5)  # Wait for process to terminate
            except TimeoutError:
                self.sim_handle.kill()  # Force kill if terminate doesn't work
            logger.info("Simulation terminated.")

        # Print summary
        logger.info("\nFuzzing Session Summary:")
        logger.info("-" * 30)
        logger.info(
            f"Simulations completed: {self.fuzzer_stats['simulations_completed']}"
        )
        logger.info(f"Messages sent: {self.fuzzer_stats['messages_sent']}")
        logger.info(
            f"Last mission time: {self.fuzzer_stats['last_mission_time']:.2f} seconds"
        )
        # logger.info(f"DTW threshold: {self.fuzzer_stats['dtw_threshold']:.2f}")
        logger.info(
            "Total time taken: {:.2f} seconds".format(time.time() - self.start_time)
        )
        logger.info(
            f"Potential crashes detected: {self.fuzzer_stats['potential_crashes']}"
        )
        logger.info("-" * 30)

    def _sigma_calc_lstm(self):
        """Calculate the LSTM thresholds based on the golden RC values."""
        # Setup the PyTorch LSTM
        device = torch.device("cpu")
        # Setup model parameters
        window_size = self.config.get("lstm_window_size", 50)  # Based on experiments
        num_features = self.config.get("lstm_num_features", 4)  # RC motor counts
        batch_size = self.config.get("lstm_batch_size", 32)  # Default batch size
        epochs = self.config.get("lstm_epochs", 25)  # Default epochs
        # Convert the golden RC values list into a numpy array
        # First need to extract and clean up things
        fields = ["servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw"]
        combined_data = []
        for iter in self.golden_rc_vals:
            combined_data.append(np.array([[pkt[f] for f in fields] for pkt in iter]))
        calib_data = np.concatenate(combined_data, axis=0)
        # Create sequences for the LSTM
        calib_data_seq = []
        for i in range(len(calib_data) - window_size):
            calib_data_seq.append(calib_data[i : i + window_size])
        calib_data_seq = np.array(calib_data_seq)
        # Scale the training data
        scaler = MinMaxScaler()
        calib_data_seq_reshaped = calib_data_seq.reshape(-1, num_features)
        calib_data_seq_scaled = scaler.fit_transform(calib_data_seq_reshaped)
        calib_data_seq_scaled = calib_data_seq_scaled.reshape(calib_data_seq.shape)
        # Train the model
        # Convert the array into PyTorch tensors
        calib_data_seq_tensor = (
            torch.from_numpy(calib_data_seq_scaled).float().to(device)
        )
        # Split the data into training and validation sets
        train_tensor, test_tensor = train_test_split(
            calib_data_seq_tensor, test_size=0.2, random_state=RANDOM_SEED
        )
        # Create DataLoader
        train_dataset = torch.utils.data.TensorDataset(train_tensor, train_tensor)
        train_loader = torch.utils.data.DataLoader(
            train_dataset, batch_size=batch_size, shuffle=True
        )

        validation_dataset = torch.utils.data.TensorDataset(test_tensor, test_tensor)
        validation_loader = torch.utils.data.DataLoader(
            validation_dataset, batch_size=batch_size, shuffle=False
        )

        # Model definition
        model = LSTMAE(seq_len=window_size, n_features=num_features).to(device)
        optimizer = optim.Adam(model.parameters())
        criterion = nn.MSELoss()  # Mean Squared Error Loss

        # Don't care about this for now
        # try:
        #     from torchinfo import summary
        #
        #     summary(
        #         model,
        #         input_size=(batch_size, window_size, num_features),
        #         device=device,
        #     )
        # except ImportError:
        #     logger.info(
        #         "torchinfo not installed. Install with 'pip install torchinfo' for detailed model summary."
        #     )
        #     logger.info(model)  # Fallback to basic model print
        logger.info("\nTraining PyTorch model...")
        history = {"train_loss": [], "val_loss": []}

        for epoch in range(epochs):
            model.train()  # Set model to training mode
            running_loss = 0.0
            for batch_X, batch_y in train_loader:
                optimizer.zero_grad()  # Zero the gradients
                outputs = model(batch_X)
                loss = criterion(outputs, batch_y)
                loss.backward()  # Backpropagation
                optimizer.step()  # Update weights
                running_loss += loss.item() * batch_X.size(0)  # Accumulate batch loss

            epoch_train_loss = running_loss / len(train_loader.dataset)
            history["train_loss"].append(epoch_train_loss)

            # Validation phase
            model.eval()  # Set model to evaluation mode
            val_loss = 0.0
            with torch.no_grad():  # Disable gradient calculations
                for batch_X_val, batch_y_val in validation_loader:
                    outputs_val = model(batch_X_val)
                    loss_val = criterion(outputs_val, batch_y_val)
                    val_loss += loss_val.item() * batch_X_val.size(0)

            epoch_val_loss = val_loss / len(validation_loader.dataset)
            history["val_loss"].append(epoch_val_loss)

            logger.info(
                f"Epoch {epoch+1}/{epochs}, Train Loss: {epoch_train_loss:.6f}, Val Loss: {epoch_val_loss:.6f}"
            )

        # Calculate reconstruction errors on the validation set
        model.eval()
        with torch.no_grad():
            test_val_pred = model(test_tensor)

        # Calculate reconstruction errors as Mean Absolute Error (MAE)
        reconstruction_errors = torch.mean(
            torch.abs(test_tensor - test_val_pred), dim=(1, 2)
        )

        # Calculate threshold (3-sigma rule)
        mean_error = torch.mean(reconstruction_errors).item()
        std_error = torch.std(reconstruction_errors).item()
        logger.debug(f"The mean is: {mean_error} and the std_dev is: {std_error}")
        self.min_fuzz_threshold = mean_error - (2 * std_error)
        self.max_fuzz_threshold = mean_error + (2 * std_error)
        logger.debug(
            "LSTM based thresholds calculated: min:{} max:{}".format(
                self.min_fuzz_threshold, self.max_fuzz_threshold
            )
        )

    def _sigma_calc_dtw(self):
        """Calculate the DTW thresholds based on the golden RC values."""
        # Calculate the DTW thresholds based on the golden RC values
        # Compare each value with the other values
        cmp_idx = 0
        for main_idx, golden_rc_vals in enumerate(self.golden_rc_vals):
            for idx, rcou_vals in enumerate(self.golden_rc_vals):
                if main_idx == idx:
                    continue
                logger.debug(
                    "Comparing golden RC values: {} with {}".format(main_idx, idx)
                )
                cmp_idx += 1
                _, distance = self.calculate_dtw(golden_rc_vals, rcou_vals)
                self.fuzzer_stats["dtw_threshold"].append(distance)
        logger.debug("Did {} comparisons".format(cmp_idx))
        # Get the threshold values for 2 sigma
        mean = np.mean(self.fuzzer_stats["dtw_threshold"])
        std_dev = np.std(self.fuzzer_stats["dtw_threshold"])
        logger.debug(f"The mean is: {mean} and the std_dev is: {std_dev}")
        self.min_fuzz_threshold = mean - (2 * std_dev)
        self.max_fuzz_threshold = mean + (2 * std_dev)
        logger.debug(
            "Final DTW thresholds calculated: min:{} max:{}".format(
                self.min_fuzz_threshold, self.max_fuzz_threshold
            )
        )

    def sigma_calc(self):
        if self.oracle_model == "dtw":
            self._sigma_calc_dtw()
        if self.oracle_model == "lstm":
            self._sigma_calc_lstm()
        # Save the RC values as pickle file to later use in the current directory
        pickle_file = os.path.join(os.getcwd(), "rcou_vals.pkl")
        with open(pickle_file, "wb") as f:
            f.write(pickle.dumps(self.golden_rc_vals))
        # Save the calibration values for faster reload next time
        mod_config_file = os.path.join(os.getcwd(), "cal_config.yaml")
        try:
            with open(mod_config_file, "w+") as f:
                config = self.config.copy()
                f.seek(0)
                # This precision is enough for now
                config["calibration_threshold"] = [
                    float(self.min_fuzz_threshold),
                    float(self.max_fuzz_threshold),
                ]
                yaml.dump(config, f)
                f.truncate()
        except Exception as e:
            logger.error(f"Error saving calibration values: {e}")

    def cleanup_sim(self):
        """Cleanup the simulation and reset states."""
        # Stop fuzzing first
        if self.fuzzing_active:
            self.stop_fuzzing()

        # Actually we start one and keep it running ideally
        # Stop the periodic threads
        # if self.periodic_thread:
        #     for thread in self.periodic_thread.values():
        #         thread.join()
        #     logger.info("Periodic threads stopped")

        # Reset the time
        self.fuzzer_stats["current_mission_time"] = 0.0
        # Reset state
        self.sim_ready = False

        # Then cleanup TCP connection
        if self.tcp_conn:
            if self.fuzzer_shutdown_requested:
                self.tcp_conn.cleanup()
            elif self.rcou_vals:
                self.rcou_vals = self.tcp_conn.cleanup()
                if self.calibration_active:
                    self.golden_rc_vals.append(self.rcou_vals)
                else:
                    if not self.min_fuzz_threshold:
                        self.sigma_calc()
                    self.oracle()
            else:
                self.rcou_vals = self.tcp_conn.cleanup()

        # Finally terminate the simulation
        if hasattr(self, "sim_handle") and self.sim_handle:
            self.sim_handle.terminate()
            logger.info("Simulation terminated.")
        time.sleep(1)  # Give some time for the threads to finish

        # TODO Check if we actually have a SITL binary running

    def get_stats_summary(self):
        """Return a formatted string with current fuzzing stats.

        Returns:
            Formatted string with fuzzing statistics.
        """
        return (
            f"Sims: {self.fuzzer_stats['simulations_completed']} | "
            f"Msgs: {self.fuzzer_stats['messages_sent']} | "
            f"Last time: {self.fuzzer_stats['last_mission_time']:.2f}s | "
            # f"DTW threshold: {self.fuzzer_stats['dtw_threshold']:.2f}"
        )

    def oracle_dtw(self):
        combined_distance = 0.0
        for golden_rc_vals in self.golden_rc_vals:
            _, distance = self.calculate_dtw(golden_rc_vals, self.rcou_vals)
            logger.info(
                f"DTW distance calculated: {distance} len: {len(self.golden_rc_vals)}"
            )
            combined_distance += distance
        distance = combined_distance / len(self.golden_rc_vals)
        logger.debug(
            "The combined distance is: {} for simulation {}".format(
                distance, self.fuzzer_stats["simulations_completed"]
            )
        )
        assert (
            self.min_fuzz_threshold is not None or self.max_fuzz_threshold is not None
        )
        if (distance < self.min_fuzz_threshold) or (distance > self.max_fuzz_threshold):
            logger.info(
                f"DTW distance {distance} exceeds {self.min_fuzz_threshold} or is way below threshold {self.max_fuzz_threshold}, potential anomaly detected! at simulation {self.fuzzer_stats['simulations_completed']}"
            )
            # Try to open the LASTLOG.TXT
            log_file_path = os.path.join(self.fuzzer_temp_dir, "logs/LASTLOG.TXT")
            log_content = "N/A"
            try:
                with open(log_file_path, "r") as log_file:
                    log_content = int(log_file.read().strip())
            except FileNotFoundError:
                logger.warning("Can't find LASTLOG.TXT file")
            logger.debug(f"Please refer to the {log_content:08d}.BIN for more details")
            self.fuzzer_stats["potential_crashes"] += 1
            # Save inputs for later analysis
            fd, input_file = tempfile.mkstemp(
                suffix=".txt",
                prefix="inputs-anomalous-",
                dir=self.fuzzer_temp_input_dir,
            )
        else:
            fd, input_file = tempfile.mkstemp(
                suffix=".txt", prefix="inputs", dir=self.fuzzer_temp_input_dir
            )
        logger.info("Saving inputs to %s", input_file)
        if self.fuzzer_state == FuzzState.Init:
            self.add_to_fuzz_queue(self.fuzz_msgs, score=distance)
        else:
            # Check if the distance is less than values in queue
            if self.fuzzer_queue:
                if distance >= self.fuzzer_queue[0].priority:
                    logger.info(
                        "Distance is greater than the first item in the queue, adding to queue"
                    )
                    self.add_to_fuzz_queue(self.fuzz_msgs, score=distance)

        try:
            with os.fdopen(fd, "w") as f:
                # Dump all the values inside the fuzz_msgs
                for msg in self.fuzz_msgs:
                    f.write(f"{msg}\n")
        except Exception as e:
            logger.error(f"Error writing to input file {input_file}: {e}")
            # Close fd if it is opened
            if fd:
                os.close(fd)

    def oracle(self):
        """Perform anomaly detection using DTW distance calculations."""
        if not self.golden_rc_vals:
            logger.error(
                "Don't have any golden RC values to compare, potentially calibration is broken?"
            )
            return
        if self.oracle_model == "dtw":
            self.oracle_dtw()
        elif self.oracle_model == "lstm":
            pass
        # Clean up the fuzz_msgs
        self.fuzz_msgs = []

    def calculate_dtw(self, series1, series2):
        """Calculate the DTW distance between two time series.

        Args:
            series1: First time series.
            series2: Second time series.

        Returns:
            Tuple containing the DTW distance and normalized distance.
        """
        # We get a list of dictionaries, so we need to convert them to numpy arrays
        # Assuming series1 and series2 are lists of dictionaries with keys "chan1_raw","chan2_raw", etc.
        fields = ["servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw"]
        s1 = np.array([[pkt[f] for f in fields] for pkt in series1])
        s2 = np.array([[pkt[f] for f in fields] for pkt in series2])
        num_channels = s1.shape[1]
        series1_normalized = np.zeros_like(s1)
        series2_normalized = np.zeros_like(s2)

        for i in range(num_channels):
            # Normalize channel i of Series 1
            chan1_mean = np.mean(s1[:, i])
            chan1_std = np.std(s1[:, i])
            series1_normalized[:, i] = (s1[:, i] - chan1_mean) / (chan1_std + 1e-8)

            # Normalize channel i of Series 2
            chan2_mean = np.mean(s2[:, i])
            chan2_std = np.std(s2[:, i])
            series2_normalized[:, i] = (s2[:, i] - chan2_mean) / (chan2_std + 1e-8)
        # Compute DTW with Euclidean distance
        alignments = dtw(
            series1_normalized,
            series2_normalized,
            dist_method="euclidean",
            distance_only=True,
        )
        return alignments.distance, alignments.normalizedDistance

    def manage_fuzzer_state(self):
        """Helper function to manage fuzzer state transitions.
        Current state transition are
            Init -> Bitflip
            Bitflip -> Arithmetic
            Arithmetic -> Interest
        """
        if (
            len(self.fuzzer_queue) >= self.fuzzer_queue_len
            and self.fuzzer_state == FuzzState.Init
        ):
            logger.debug("Reached fuzzer queue length, switching to Mutations state")
            self.fuzzer_state = FuzzState.Bitflip
        elif (len(self.fuzzer_queue) >= self.fuzzer_queue_len) and (
            self.fuzzer_state == FuzzState.Bitflip
        ):
            logger.debug("Reached fuzzer queue length, switching to Arithmetic state")
            self.fuzzer_state = FuzzState.Arithmetic
        elif self.fuzzer_state != FuzzState.Init and len(self.fuzzer_queue) == 0:
            logger.debug("Nothing in the queue, switching to Init state")
            self.fuzzer_state = FuzzState.Init
        # Check if we reach the queue length, then we should actually remove lowest priority items
        if len(self.fuzzer_queue) > self.fuzzer_queue_len:
            logger.debug(
                f"Fuzzer queue length exceeded {self.fuzzer_queue_len}, removing lowest priority items"
            )
            # Remove the lowest priority items from the queue
            while len(self.fuzzer_queue) > self.fuzzer_queue_len:
                heapq.heappop(self.fuzzer_queue)
            logger.debug(
                f"Fuzzer queue length is now {len(self.fuzzer_queue)} after cleanup"
            )

    def start_fuzzing(self):
        """Start the fuzzing thread."""
        if not self.xml_messages:
            logger.warning(
                "No XML message definitions loaded. Cannot start peripheral fuzzing."
            )
            return

        self.manage_fuzzer_state()
        self.fuzzing_active = True
        self.fuzzing_thread = threading.Thread(target=self.fuzz_loop, daemon=True)
        self.fuzzing_thread.start()
        logger.info("Fuzzing thread started")

    def stop_fuzzing(self):
        """Stop the fuzzing thread."""
        self.fuzzing_active = False
        if self.fuzzing_thread:
            self.fuzzing_thread.join(timeout=2)
            logger.info("Fuzzing thread stopped")

    def init_generate_message(self):
        """
        Generate an initial set of messages for fuzzing.
        """
        # Generate random values for each field
        msg_def = random.choice(self.xml_messages)
        field_values = {}
        for field in msg_def["fields"]:
            field_name = field["name"]
            field_type = field.get("type")  # Get type safely
            field_desc = field.get("desc", None)
            field_units = field.get("units", None)
            field_max = field.get("maxValue", None)
            field_min = field.get("minValue", None)
            field_itr = field.get("increment", None)
            # Check if the field is enum
            if "enum_vals" in field:
                chosen_enum_value = random.choice(field["enum_vals"])
                if isinstance(chosen_enum_value, str):
                    # Attempt conversion if MAVLink type is numeric
                    if field_type and (
                        field_type.startswith(("uint", "int", "float", "double"))
                        or field_type == "char"
                    ):
                        try:
                            if "float" in field_type or "double" in field_type:
                                field_values[field_name] = float(chosen_enum_value)
                            else:
                                field_values[field_name] = int(chosen_enum_value)
                        except ValueError:
                            logger.warning(
                                f"Could not convert enum string '{chosen_enum_value}' to numeric for field '{field_name}' (type: {field_type}). Using 0 as fallback."
                            )
                            field_values[field_name] = 0
                    else:  # Type is likely string based (e.g. char[], string, enum_str) or unknown
                        field_values[field_name] = chosen_enum_value
                else:  # Value from enum_vals is already a number (e.g. int from type="enum")
                    field_values[field_name] = chosen_enum_value

            elif field_type == "param_range_float":
                min_val = field.get("range_min", -10.0)
                max_val = field.get("range_max", 10.0)
                field_values[field_name] = random.uniform(min_val, max_val)

            elif field_type:  # Fallback for other standard MAVLink types
                field_values[field_name] = generate_field_value(
                    field_type,
                    field_desc=field_desc,
                    field_units=field_units,
                    field_range=[field_min, field_max, field_itr],
                )

            else:
                logger.warning(
                    f"Field '{field_name}' in message '{msg_def['msg_name']}' has no discernible type or unhandled structure. Assigning default value 0."
                )
                field_values[field_name] = 0

        return msg_def["msg_name"], msg_def["msg_id"], field_values

    def _mutate_bitflip(self, field_values):
        """
        Internal helper to perform bitflip mutation on field values.
        The idea in this mode is actually flip bits based on the Length + Step-over depending upon the field type
        """
        # Ideally we'll have a set of 7-8 values depending upon the field type
        # bits: stepover
        # Create a new set of field values
        size = 8  # TODO: Figure out the actual size of the field
        mod_values = field_values.copy()
        # Handle different field types
        for field_name, value in field_values.items():
            index = random.randint(0, size)  # Flip a random bit in the byte
            if type(value) is list:
                value = random.choice(value)  # Select a random value from the list
                if type(value) is float:
                    mod_values[field_name] = flip_float_bit(value, index)
                elif type(value) is int:
                    mod_values[field_name] = value ^ (1 << index)
        return mod_values

    def _mutate_arithmetic(self, field_values):
        """
        Ideally perform arithmetic mutation on field values
        """
        # TODO
        return field_values  # No mutation for now

    def mutate_msg(self):
        """
        Basically mutate a message from the fuzzer queue
        Depending upon the fuzzer state, we will do the relevant mutation
        """
        # Get a message from the fuzzer queue
        entry = self.get_next_in_fuzz_queue()
        msg_list = entry.data
        if not msg_list:
            logger.error("No messages in the fuzzer queue to mutate")
            return None, None, None
        # It should ideally be a list of msgs, that contains the timestamp, msg_name, msg_id and field_values
        msg_entry = random.choice(msg_list)  # Select a random message from the list
        msg_name = msg_entry[1]
        msg_id = msg_entry[2]
        field_values = msg_entry[3]
        if self.fuzzer_state == FuzzState.Bitflip:
            field_values = self._mutate_bitflip(field_values)
        if self.fuzzer_state == FuzzState.Arithmetic:
            field_values = self._mutate_arithmetic(field_values)
        return msg_name, msg_id, field_values

    def fuzz_loop(self):
        """Main fuzzing loop that runs in a separate thread."""
        while self.fuzzing_active and self.tcp_conn.drone_in_air:
            if self.fuzzer_state == FuzzState.Init:
                msg_name, msg_id, field_values = self.init_generate_message()
            elif self.fuzzer_state in [FuzzState.Bitflip, FuzzState.Arithmetic]:
                msg_name, msg_id, field_values = self.mutate_msg()
                if field_values is None:
                    # If we don't have a message to mutate, go back to init state
                    logger.debug("No message to mutate, going back to init state")
                    msg_name, msg_id, field_values = self.init_generate_message()
            # If we are in calibration mode, just send the same values over for the fields
            # TODO Eventually move towards a common state in Fuzzer_State for calibration
            if self.calibration_active:
                if self.calibration_vals is None:
                    # Actually check if we have calibration values from the file
                    if self.calibration_msg:
                        # Use the first message in the calibration_msg
                        self.calibration_vals = self.calibration_msg
                        # Needs to be a dict for sending
                        # NOTE: Just select a message for now
                        msg_def = random.choice(self.xml_messages)
                        if isinstance(self.calibration_vals, list):
                            # Convert list to dict with keys from msg_def fields
                            field_names = [f["name"] for f in msg_def["fields"]]
                            self.calibration_vals = dict(
                                zip(field_names, self.calibration_vals)
                            )
                        field_values = self.calibration_vals
                    else:
                        self.calibration_vals = field_values
                else:
                    field_values = self.calibration_vals
            # Send the fuzzed message
            msg_dict = self.send_fuzzed_message(msg_name, msg_id, field_values)
            # To ensure we only save fuzzed message
            if not self.calibration_active:
                self.fuzz_msgs.append(msg_dict)
                # If we are in the Init mode, we save all the messages
                # 2025-06-14T10:15:35-0400: silipwn: Maybe better to save after oracle
                # if self.fuzzer_state == FuzzState.Init:
                #     self.fuzzer_queue.append(msg_dict)
            self.fuzzer_stats["messages_sent"] += 1

            time.sleep(1 / self.fuzz_interval)

    def hueristics(self, field_values):
        """Apply heuristic replacements for specific fields.

        Args:
            field_values: Dictionary of field values to modify.
        """
        # Heuristic replacer for some common fields
        for field_name in field_values.keys():
            # Check if we have a field name that contains "time"
            if "time_boot_ms" in field_name:
                current_time = round(
                    (time.time() - self.fuzzer_stats["current_mission_time"]) * 1000
                )
                field_values[field_name] = current_time
            if "time_usec" in field_name:
                field_values[field_name] = int(time.time())
            if "target_system" in field_name:
                field_values[field_name] = self.target_system
            if "target_component" in field_name:
                field_values[field_name] = self.target_component
            # Hardcoded stuff, please remove when doing final eval
            # CS1
            if "sensor_type" in field_name:
                field_values[field_name] = 0
            if "frame" in field_name:
                field_values[field_name] = 12
            if "min_distance" in field_name:
                field_values[field_name] = 0.0
            if "max_distance" in field_name:
                field_values[field_name] = 12.0
            if field_name == "q":  # Exact match only
                # Replace quaternion with a random value
                # TODO: Please verify if this assumption is correct
                field_values[field_name] = [random.uniform(-1, 1) for _ in range(4)]
            # GPS Values
            if "gps_id" in field_name:
                field_values[field_name] = 0
            if "satellites_visible" in field_name:
                field_values[field_name] = random.randint(0, 30)
            if "fix_type" in field_name:
                field_values[field_name] = random.randint(3, 6)

    # Smart sleep
    def error_sleep(self, seconds, period=1):
        """
        Basically sleep for seconds while checking with a period for an error
        Also return True or False, if an error was encountered.
        """
        while seconds > 0:
            time.sleep(period)
            if not error_queue.empty():
                self.handle_errors()
                error_queue.queue.clear()  # Clear the queue after handling
                return True
            seconds -= period
        return False

    # Smart wait
    def wait_for_condition(self, predicate, timeout=30, poll_interval=0.2):
        """
        Wait for a condition to be met with a timeout.

        This function repeatedly checks a condition until it becomes true or until a timeout
        is reached.

        Args:
            predicate (callable): A function that returns a boolean. The function will return
                when this predicate returns True.
            timeout (float, optional): Maximum time in seconds to wait for the condition.
                Defaults to 30 seconds.
            poll_interval (float, optional): Time in seconds between checks of the predicate.
                Defaults to 0.2 seconds.

        Returns:
            bool: True if the condition was met within the timeout, False otherwise or if
                an error was encountered.

        Note:
            This function will return immediately if an error is detected in the error_queue.
        """
        waited = 0
        while not predicate():
            if not error_queue.empty():
                self.handle_errors()
                return False
            time.sleep(poll_interval)
            waited += poll_interval
            if waited >= timeout:
                logger.error("Timeout waiting for condition.")
                return False
        return True

    def handle_errors(self):
        """
        Handles errors that occur during fuzzing operations.

        This method processes errors from the error queue and takes appropriate actions:
        - For TCP connection errors, SITL termination errors, or SITL timeout errors:
            1. Logs the crash
            2. Saves the current message sequence to a temporary file
            3. Increments the potential crash counter
            4. Cleans up the simulation
            5. If in calibration mode, exits with error code 1
        - For internal fuzzer errors:
            1. Logs the error with component information
            2. Exits the program with an error code (1) as these are considered unrecoverable

        The method continuously processes all errors in the queue until it's empty.
        """
        # Check if we don't have any errors
        while not error_queue.empty():
            error = error_queue.get()

            # Handle TCP connection errors
            if (
                error.get("type") == "tcp_connection_error"
                or error.get("type") == "sitl_terminated_error"
                or error.get("type") == "sitl_timeout_error"
            ):
                tqdm.write(
                    f"SITL has issues | Can't establish connection: {error['error']}, saving inputs ..."
                )
                # Save the current message sequence as an anomaly
                fd, input_file = tempfile.mkstemp(
                    suffix=".txt",
                    prefix="inputs-crash-anomaly-",
                    dir=self.fuzzer_temp_input_dir,
                )
                logger.info("Saving inputs to %s", input_file)
                with os.fdopen(fd, "w") as f:
                    # Save current messages that may have triggered the connection issue
                    for msg in self.fuzz_msgs:
                        f.write(f"{msg}\n")
                # Count it as a potential crash
                self.fuzzer_stats["potential_crashes"] += 1
                
                # If we're in calibration mode, this is a fatal error
                if self.calibration_active:
                    logger.error("SITL error during calibration phase - this is fatal")
                    self.cleanup_and_exit()
                    exit(1)
                else:
                    self.cleanup_sim()
                    
            if error.get("type") == "fuzzer_error":
                logger.error(error["error"])
                logger.error("Not recoverable state, exiting...")
                exit(1)

    def send_fuzzed_message(self, msg_name, msg_id, field_values: Dict):
        """Send a mutated message using the MAVLink connection.

        Args:
            msg_name: Name of the message.
            msg_id: ID of the message.
            field_values: Dictionary of field values for the message.

        Returns:
            List containing the message time, name, id, and field values.
        """
        self.hueristics(field_values)
        # ^ TODO: Do we apply this only if we are in fuzzing mode?
        try:
            # Get the message class from mavutil
            msg_class = getattr(self.tcp_conn.conn.mav, f"{msg_name.lower()}_send")  # type: ignore

            # Create the message send with the fuzzed values
            # Send the message
            msg_time = time.time() - self.fuzzer_stats["current_mission_time"]
            msg_class(**field_values)  # type: ignore
            return [msg_time, msg_name, msg_id, field_values]

        except AttributeError:
            # If the field_values are not correct in length (7), we add the message with 0s
            while len(field_values) < 7:
                field_values[f"param{len(field_values) + 1}"] = 0
            # Structure the message to be similar to the LONG_COMMAND message
            # Which means we rename all the fields to match the 7 params
            # Should be a dict with keys like param1, param2, etc.
            val_idx = 0
            modified_field_values = {}
            for _, val in enumerate(field_values.values()):
                val_key = f"param{val_idx + 1}"
                modified_field_values[val_key] = float(
                    val
                )  # To ensure that we always send float values
                val_idx += 1
            packed_msg = mavutil.mavlink.MAVLink_command_long_message(
                self.target_system,  # target_system  # type: ignore
                self.target_component,  # target_component  # type: ignore
                int(msg_id),  # command
                0,  # confirmation
                **modified_field_values,  # parameters
            )
            msg_time = time.time() - self.fuzzer_stats["current_mission_time"]
            self.tcp_conn.conn.mav.send(packed_msg)  # type: ignore
            return [msg_time, msg_name, msg_id, modified_field_values]
        except Exception as e:
            logger.error(
                f"Error sending message {msg_name} with ID {msg_id} and values {field_values}: {e}"
            )
            return []


# Misc utilities and sanity checks
def file_exists(file_o_dir):
    if os.path.exists(file_o_dir):
        return True
    else:
        logger.error(f"File or directory {file_o_dir} does not exist.")  # HACK
        raise FileNotFoundError(f"File or directory {file_o_dir} does not exist.")


if __name__ == "__main__":
    cfg = None
    try:
        argument_parser = argparse.ArgumentParser(
            description="DroneFuzz++ - A fuzzer for ArduPilot"
        )
        argument_parser.add_argument(
            "--bin", type=str, help="Path to the SITL binary", required=False
        )
        argument_parser.add_argument(
            "--peripheral", type=str, help="Peripheral to fuzz", required=False
        )
        argument_parser.add_argument(
            "--ap_dir", type=str, help="Ardupilot directory", required=False
        )
        argument_parser.add_argument(
            "--auto_mission", type=str, help="Auto mission file", required=False
        )
        argument_parser.add_argument(
            "--xml",
            type=str,
            help="Path to MAVLink XML definition file",
            required=False,
        )
        argument_parser.add_argument(
            "--config",
            type=str,
            help="YAML file for general configuration",
        )
        argument_parser.add_argument(
            "--peripheral_file",
            type=str,
            help="YAML file for peripheral mapping",
        )
        argument_parser.add_argument(
            "--vehicle",
            type=str,
            help="Vehicle type (copter, plane, rover, etc.)",
            required=False,
        )
        argument_parser.add_argument(
            "--calibration_rounds",
            type=int,
            help="Number of calibration rounds to run",
            required=False,
        )
        argument_parser.add_argument(
            "--dtw_threshold",
            type=float,
            help="DTW threshold for anomaly detection",
            required=False,
        )
        args = argument_parser.parse_args()

        # Check for required arguments if --config is not provided
        if not args.config:
            missing_args = []
            if not args.bin:
                missing_args.append("--bin")
            if not args.peripheral:
                missing_args.append("--peripheral")
            if not args.xml:
                missing_args.append("--xml")
            if not args.peripheral_file:
                missing_args.append("--peripheral_file")
            if not args.ap_dir:
                missing_args.append("--ap_dir")

            if missing_args:
                argument_parser.error(
                    "The following arguments are required when --config is not provided: {}".format(
                        ", ".join(missing_args)
                    )
                )
        # Create a temporary folder for the fuzzed messages locally in the same directory that we are running
        fuzzer_temp_dir = tempfile.mkdtemp("dronefuzz", "fuzzing_data", os.getcwd())
        logger = setup_logging(fuzzer_temp_dir)
        # Add the fuzzer temp dir to args for later use
        args.fuzzer_temp_dir = fuzzer_temp_dir
        cfg = FuzzConfig(args)

        if not cfg.calibration_threshold:
            # Establish the threshold for the fuzzing runs
            # Run the mission with simulations and default parameters
            # Check if we already have calibration values
            logger.info("Beginning calibration")
            cfg.calibration_active = True

            # Create tqdm progress bar for calibration
            calib_pbar = tqdm(
                range(cfg.calibration_rounds), desc="Calibration Progress"
            )

            # Function to update calibration tqdm with stats
            def update_calib_tqdm_postfix():
                calib_pbar.set_postfix(
                    {
                        "round": f"{calib_pbar.n + 1}/{cfg.calibration_rounds}",
                        # "dtw_sum": f"{cfg.fuzzer_stats['dtw_threshold']:.1f}",
                        "time": f"{cfg.fuzzer_stats['last_mission_time']:.1f}s",
                    }
                )
                # tqdm.write can be used here for important messages if needed
                # For example: tqdm.write(f"Important calibration update for round {calib_pbar.n + 1}")

            for i in calib_pbar:
                logger.info(
                    f"Starting calibration round {i+1}/{cfg.calibration_rounds}"
                )
                # Use tqdm.write for messages that should not interfere with the bar
                tqdm.write(f"Calibration Round: {i+1}/{cfg.calibration_rounds}")
                cfg.run_sim()

                logger.info("Waiting for drone to be ready with GPS lock...")
                while (
                    cfg.tcp_conn.drone_ready is False
                    and not cfg.fuzzer_shutdown_requested
                ):
                    if cfg.error_sleep(1):
                        logger.error("Error encountered during calibration waiting phase")
                        cfg.cleanup_and_exit()
                        exit(1)
                    update_calib_tqdm_postfix()  # Keep progress bar visible during waiting
                
                # Check for errors after waiting loop
                if not error_queue.empty():
                    logger.error("Error detected during calibration phase")
                    cfg.handle_errors()
                    cfg.cleanup_and_exit()
                    exit(1)
                
                if cfg.tcp_conn.drone_ready and cfg.vehicle == "plane":
                    # Sleep for some more time to ensure the Gyro is consistent
                    time.sleep(8)

                if cfg.fuzzer_shutdown_requested:
                    logger.error("Shutdown requested during calibration")
                    cfg.cleanup_and_exit()
                    exit(1)
                else:
                    try:
                        cfg.send_mission()
                    except Exception as e:
                        logger.error(f"Error during calibration mission: {e}")
                        cfg.cleanup_and_exit()
                        exit(1)

                # Check for errors after mission
                if not error_queue.empty():
                    logger.error("Error detected after calibration mission")
                    cfg.handle_errors()
                    cfg.cleanup_and_exit()
                    exit(1)

                cfg.cleanup_sim()
                update_calib_tqdm_postfix()

            # cfg.fuzzer_stats["dtw_threshold"] = cfg.fuzzer_stats["dtw_threshold"] / (
            #     cfg.calibration_rounds - 1
            # )
            cfg.calibration_active = False
            cfg.fuzzer_stats["current_mission_time"] = 0.0
            cfg.fuzzer_stats["simulations_completed"] = 0
            cfg.fuzzer_stats["messages_sent"] = 0
        else:
            logger.debug("Calibration values already set, skipping calibration")

        # Run the simulation with fuzzing
        # Main loop with progress tracking and stats
        fuzzing_iterations = 0
        pbar = tqdm(desc="Fuzzing Progress")

        # Function to update tqdm with stats
        def update_tqdm_postfix():
            pbar.set_postfix(
                {
                    "time": f"{cfg.fuzzer_stats['last_mission_time']:.1f}s",
                    "state": f"{cfg.fuzzer_state}",
                    "sims": cfg.fuzzer_stats["simulations_completed"],
                    "msgs": cfg.fuzzer_stats["messages_sent"],
                    "dtw_min": f"{cfg.min_fuzz_threshold:.1f}",
                    "dtw_max": f"{cfg.max_fuzz_threshold:.1f}",
                    "bugs": f"{cfg.fuzzer_stats['potential_crashes']}",
                }
            )

        while not cfg.fuzzer_shutdown_requested:
            fuzzing_iterations += 1
            logger.info(f"Starting fuzzing iteration {fuzzing_iterations}")
            tqdm.write(f"Fuzzing Iteration: {fuzzing_iterations}")
            cfg.run_sim()

            tqdm.write("Waiting for drone GPS lock...")
            while (
                not cfg.tcp_conn.drone_ready
                and not cfg.fuzzer_shutdown_requested
                and error_queue.empty()
            ):
                if cfg.error_sleep(1):
                    logger.error("Potential error encountered during waiting")
                pbar.refresh()  # Keep progress bar visible during waiting

            if cfg.tcp_conn.drone_ready and cfg.vehicle == "plane":
                # Sleep for some more time to ensure the Gyro is consistent
                time.sleep(8)

            if not error_queue.empty():
                cfg.handle_errors()

            elif not cfg.fuzzer_shutdown_requested:
                cfg.send_mission()
                cfg.cleanup_sim()

            logger.debug(
                f"Finished fuzzing with {cfg.fuzzer_stats['messages_sent']} messages sent"
            )
            pbar.update(1)
            update_tqdm_postfix()
            # except Exception as e:
            #     tqdm.write(f"Error in main loop: {e}") # Use tqdm.write for errors too
            #     if not cfg.shutdown_requested:
            #         print("Attempting to restart simulation...")
            #         time.sleep(5)  # Wait before retrying
    except Exception as e:
        logger.critical(f"Fatal error: {e}", exc_info=True)
    finally:
        # Ensure cleanup happens even if there's an unhandled exception
        if cfg:
            cfg.cleanup_and_exit()
