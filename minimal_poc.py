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
import itertools
import logging
import datetime
import pickle
import shutil
import shlex
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
from queue import Queue, Empty
import threading


# Exceptions
class InternalError(Exception):
    pass


mavlink_timeout = 5
approx_threshold = 0.00005  # Threshold for approximate location matching
altitude_threshold = 0.1  # Threshold for altitude matching
logger = logging.getLogger("dummy")
PREARM_CHECK = 0x10000000
EKF_POS_HORIZ = 0x8
EKF_POS_VERT = 0x10
MAX_MODE_CHANGES = 3

# Global error queue
error_queue = Queue()

RANDOM_SEED = 42  # For reproducibility

# Supported models
detection_models = ["dtw", "lstm"]


# Setup logging
def setup_logging(name="dronefuzz", file_dir=None):
    """Setup logging with timestamp in filename"""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"dronefuzz_{timestamp}.log"
    if file_dir:
        log_filename = os.path.join(file_dir, log_filename)

    # Create logger
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
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
        "%(asctime)s - %(name)s - %(lineno)d - %(levelname)s - %(message)s"
    )

    # Apply formatters
    file_handler.setFormatter(file_formatter)
    console_handler.setFormatter(console_formatter)

    # Add handlers to logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    print(f"Logging initialized. Log file: {log_filename}")
    return logger


class _LSTMHead(nn.Module):
    def __init__(self, h, hidden=None):
        super().__init__()
        mid = hidden or max(8, h // 2)
        self.net = nn.Sequential(nn.Linear(h, mid), nn.ReLU(), nn.Linear(mid, 1))

    def forward(self, z):  # z: [B,T,H] or [B,H]
        if z.dim() == 3:
            z = z.mean(dim=1)  # mean pool over time
        return self.net(z).squeeze(-1)  # logits [B]


def _ssl_finetune_head(self, model_ae, device):
    import torch, torch.nn as nn, os
    from torch.utils.data import DataLoader, TensorDataset

    if len(self.ssl_buffer) < self.ssl_min_train:
        logger.debug(
            f"SSL: buffer {len(self.ssl_buffer)} < min {self.ssl_min_train}, skip finetune"
        )
        return 0

    head = _LSTMHead(h=self.lstm_hidden).to(device)
    if os.path.exists(self.ssl_head_path):
        head.load_state_dict(torch.load(self.ssl_head_path, map_location=device))
        logger.info("SSL: loaded existing head weights")
    head.train()

    opt = torch.optim.Adam(head.parameters(), lr=self.ssl_head_lr)
    loss_fn = nn.BCEWithLogitsLoss(reduction="none")

    # Build tensors
    X = np.stack([w for (w, _, _) in self.ssl_buffer])  # [N,T,C]
    y = np.array([y for (_, y, _) in self.ssl_buffer], dtype=np.float32)
    wt = np.array([w for (_, _, w) in self.ssl_buffer], dtype=np.float32)

    Xt = torch.from_numpy(X).float().to(device)
    yt = torch.from_numpy(y).float().to(device)
    wt = torch.from_numpy(wt).float().to(device)

    with torch.no_grad():
        z_all, _ = model_ae.enc(Xt)  # [N,T,H]
        z_pool = z_all.mean(dim=1)  # [N,H]

    ds = TensorDataset(z_pool, yt, wt)
    dl = DataLoader(ds, batch_size=self.ssl_head_batch, shuffle=True, drop_last=False)

    total_steps = 0
    for ep in range(self.ssl_head_epochs):
        ep_loss = 0.0
        for z_b, y_b, w_b in dl:
            logits = head(z_b)  # [B]
            loss_v = loss_fn(logits, y_b)  # [B]
            loss = (loss_v * w_b).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            ep_loss += float(loss.item())
            total_steps += 1
        logger.info(
            f"SSL head epoch {ep+1}/{self.ssl_head_epochs}: loss={ep_loss/ max(1,len(dl)):.6f}"
        )

    os.makedirs(os.path.dirname(self.ssl_head_path), exist_ok=True)
    torch.save(head.state_dict(), self.ssl_head_path)
    logger.info(f"SSL head saved to {self.ssl_head_path}")
    return len(self.ssl_buffer)


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


def gen_int_step(min_val, max_val, increment):
    """Generate a random int value within a specified increment"""
    if int(increment) == 0:
        return random.choice(range(int(min_val), int(max_val) + 1, 1))
    return random.choice(range(int(min_val), int(max_val) + 1, int(increment)))


class TCPConn:
    def __init__(self, autopilot_type="ardupilot"):
        # Python inits
        self.autopilot_type = autopilot_type
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
        self.autopilot_type = autopilot_type
        # 2025-06-18T13:35:06-0400: silipwn: Not sure if we actually are using this, so disabling for now
        # self.internal_error = False
        self.drone_state = mavutil.mavlink.MAV_STATE_UNINIT  # Initial state
        if self.autopilot_type == "ardupilot":
            # Connection details
            with open(os.devnull, "w") as fnull:
                with redirect_stdout(fnull):
                    self.conn = mavutil.mavlink_connection(
                        "tcp:localhost:5760", autoreconnect=True, retries=3
                    )  # type: ignore
        elif self.autopilot_type == "px4":
            # PX4 uses UDP connection
            self.conn = mavutil.mavlink_connection(
                "udp:localhost:14550", autoreconnect=True
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
        # print(f"SYS_STATUS: {msg}")
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

    def _monitor_status_text_ap(self, msg):
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
        if re.search(r"PreArm.*", msg.text, re.IGNORECASE):
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

    def _monitor_status_text_px4(self, msg):
        print(f"DRONE_MSG: {msg.text}")
        # TODO: Need to verify all these messages
        # if re.search(r"Ready for takeoff!", msg.text, re.IGNORECASE):
        #     logger.info("Vehicle is ready for takeoff.")
        #     self.drone_ready = True
        # ^^ 2025-11-09T09:09:40-0500: silipwn: Doesn't actually work :|
        if re.search(r"finished\w*", msg.text, re.IGNORECASE):
            logger.info("Mission ended, vehicle is disarmed.")
            self.drone_in_air = False
        if re.search(r"takeoff\w*", msg.text, re.IGNORECASE):
            self.drone_in_air = True
            logger.info("AUTO Mission started, takeoff.")
        if re.search(r"PreArm.*", msg.text, re.IGNORECASE):
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
                        logger.debug(f"DRONE_MSG: {msg.text}")
                        # Crazy check because pymavlink lock doesn't work
                        if self.autopilot_type == "ardupilot":
                            self._monitor_status_text_ap(msg)
                        elif self.autopilot_type == "px4":
                            self._monitor_status_text_px4(msg)
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
                    if msg.get_type() == "MISSION_REQUEST_INT":  # type: ignore
                        self.mission_msg_queue.put(msg)
                    if msg.get_type() == "MISSION_CURRENT":
                        # Check if we have the situation
                        try:
                            if msg.seq == (msg.total - 1):
                                self.rc_monitor = False
                            if (msg.seq == 1) & (msg.total > 1):
                                self.rc_monitor = True  # 2025-11-13T16:44:17-0500: silipwn: This ideally means we are
                                # on the right track
                        except Exception as e:
                            logger.warning("I wrote something stupid {e}")
                    if msg.get_type() == "HEARTBEAT":  # type: ignore
                        if self.autopilot_type == "px4":
                            # Hack to get the drone ready
                            if msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
                                self.drone_ready = True
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

    def custom_msg_send(self, type, field_values):
        # Basically get the type of message and then get the values and send
        msg_class = getattr(self.conn.mav, f"{type.lower()}_send")
        msg_class(**field_values)

    def st_msg_send(self, text):
        msg = self.conn.mav.statustext_encode(
            mavutil.mavlink.MAV_SEVERITY_INFO, text.encode()  # type: ignore
        )  # type: ignore
        self.conn.mav.send(msg)  # type: ignore

    def set_mode(self, mode):
        # Check if the mode exists in the vehicle mapping
        mode_mapping = self.conn.mode_mapping()  # type: ignore
        set_mode = mode_mapping.get(mode, None)
        if self.autopilot_type == "px4":
            set_mode = float(set_mode[-1])
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
        # Get the drone's current location
        loc = self.loc_queue.get(timeout=mavlink_timeout)
        if loc is None:
            logger.error("No location data received, skipping takeoff")
            return

        current_lat = loc["lat"]
        current_lon = loc["lon"]
        current_rel_alt = loc["rel_alt"]

        altitude = current_rel_alt + altitude
        logger.debug(f"Target takeoff altitude: {altitude} meters (relative)")

        self.conn.mav.command_long_send(
            self.conn.target_system,  # type: ignore
            self.conn.target_component,  # type: ignore
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,  # param1: min pitch (0 for rotorcraft)
            0,  # param2: empty
            0,  # param3: empty
            0,
            0,
            0,
            altitude,  # param7: altitude (relative to home)
        )  # type: ignore

        # Wait for the drone to reach the target altitude
        logger.debug(f"Waiting for drone to reach altitude: {altitude} meters")
        timeout_counter = 0
        max_timeout_iterations = 600  # 60 seconds with 0.1s sleep

        while timeout_counter < max_timeout_iterations:
            loc = self.loc_queue.get(timeout=mavlink_timeout)
            if (
                altitude - altitude_threshold
                <= loc["rel_alt"]
                <= altitude + altitude_threshold
            ):
                logger.info(f"Drone has taken off to altitude: {loc['rel_alt']} meters")
                self.drone_in_air = True
                break
            timeout_counter += 1
            if timeout_counter % 50 == 0:  # Log progress every 5 seconds
                logger.debug(
                    f"Current altitude: {loc['rel_alt']} meters, target: {altitude} meters"
                )
            time.sleep(0.1)

        if timeout_counter >= max_timeout_iterations:
            logger.warning(
                f"Takeoff timeout: reached {loc['rel_alt']} meters, target was {altitude} meters"
            )

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
        logger.error(f"Enum '{enum_name}' not found")
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
            # if child.tag == "extensions":
            #     break
            if child.tag == "field" or child.tag == "extensions":
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

                # Only append if we have some name else ignore
                if entry["name"] is not None:
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
                    inc_str = param.get(
                        "increment"
                    )  # Increment not directly used by random.uniform
                    units = param.get("units", "")

                    entry["desc"] = desc_text
                    if units:
                        entry["desc"] += f" (Units: {units})"

                    try:
                        # Params are often floats. Use defaults if min/max are not specified.
                        entry["range_min"] = (
                            mn_str if mn_str is not None else 0.0
                        )  # Wider default range
                        entry["range_max"] = mx_str if mx_str is not None else 360.0
                        entry["increment"] = inc_str if inc_str is not None else 1.0
                        entry["type"] = "param_range_auto"
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


class CoverageData:
    """Class to hold coverage data for the simulator"""

    def __init__(self, src_dir=None, fuzz_dir=None):
        """Setup the coverage file"""
        if src_dir is None or fuzz_dir is None:
            raise Exception(
                "Source directory and fuzzer directory must be provided for coverage data."
            )
        # First zero the counters in the root dir?
        logger.info(f"Initializing coverage data with source directory: {src_dir}")
        self.src_dir = src_dir
        self.fuzz_dir = fuzz_dir
        # mkdir for coverage data
        self.coverage_dir = os.path.join(self.fuzz_dir, "coverage")
        os.makedirs(self.coverage_dir, exist_ok=True)
        self.reset()
        lcov_cmd = f"lcov --no-external --capture --directory {self.src_dir} --output-file {self.coverage_dir}/base_coverage.info"
        try:
            subprocess.run(
                shlex.split(lcov_cmd),
                check=True,
                stdout=subprocess.DEVNULL,  # Don't care about the output for now
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to capture initial coverage data: {e}")

    def reset(self):
        """Command to reset the coverage data"""
        # Zeroes the counters in the source directory
        lcov_reset_cmd = f"lcov --no-external --zerocounters --directory {self.src_dir}"
        try:
            subprocess.run(
                shlex.split(lcov_reset_cmd),
                check=True,
                stdout=subprocess.DEVNULL,  # Don't care about the output for now
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to reset coverage data: {e}")

    def update(self):
        """Just check the and update the relevant coverage data"""
        lcov_capture_cmd = f"lcov --no-external --capture --directory {self.src_dir} --output-file {self.coverage_dir}/current_simulation_coverage.info"
        try:
            subprocess.run(
                shlex.split(lcov_capture_cmd),
                check=True,
                stdout=subprocess.DEVNULL,  # Don't care about the output for now
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to update coverage data: {e}")

    def calibration_save(self):
        """Save the current coverage data and save"""
        lcov_capture_cmd = f"lcov --no-external --capture --directory {self.src_dir} --output-file {self.coverage_dir}/base_cal_coverage.info"
        try:
            subprocess.run(
                shlex.split(lcov_capture_cmd),
                check=True,
                stdout=subprocess.DEVNULL,  # Don't care about the output for now
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to update coverage data: {e}")

    def archive_data(self, filename=None):
        """Archive the coverage data if requested"""
        # Copy the info file into the fuzz_dir with the requested filename
        if isinstance(filename, str):
            filename_extracted = filename
        else:
            filename_extracted = f"{filename:08d}"
        shutil.copy(
            f"{self.coverage_dir}/current_simulation_coverage.info",
            f"{self.coverage_dir}/anomaly-{filename_extracted}.info",
        )


def save_diff_img(filename, fuzz_enum_mode, script_dir, output_dir):
    # Run the script and save the diff image
    if filename == 0 or filename == "N/A":
        raise Exception("Filename is not valid, cannot save diff image.")
    output_filename = os.path.join(output_dir, f"{filename}.png")
    plot_script = os.path.join(script_dir, "plot_servo_values.py")
    cmd = f"python3 {plot_script} {fuzz_enum_mode:08d}.BIN {filename:08d}.BIN --rc-log-filter --output {output_filename}"
    log_dir = os.path.join(output_dir, "logs")
    try:
        subprocess.run(shlex.split(cmd), check=True, cwd=log_dir)
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to save diff image: {e}")


# --- helper: stitch overlapped windows by mean on overlaps ---
def _stitch_overlap_mean(wins: np.ndarray, stride: int) -> np.ndarray:
    """
    wins: [B, T, C] windows (already in same scale).
    Returns a single sequence [L, C] formed by overlap-averaging with given stride.
    """
    B, T, C = wins.shape
    L = (B - 1) * stride + T
    out = np.zeros((L, C), dtype=wins.dtype)
    cnt = np.zeros((L, 1), dtype=np.float32)
    for b in range(B):
        s = b * stride
        out[s : s + T] += wins[b]
        cnt[s : s + T] += 1.0
    cnt[cnt == 0] = 1.0
    return out / cnt


# --- new: 3-panel overlay (BIN original vs LSTM recon vs difference) ---
def save_lstm_bin_triptych(
    filename_idx: int | None,
    recon_norm: np.ndarray,  # [B,T,4] normalized reconstruction
    mean: np.ndarray,  # [4]
    std: np.ndarray,  # [4]
    stride: int,
    output_dir: str,
    title: str = "LSTM Reconstruction vs BIN (raw units)",
    channels=("C1", "C2", "C3", "C4"),
    rc_log_filter: bool = True,
):
    """
    Produces three subplots:
      (top)   BIN raw C1..C4
      (mid)   LSTM reconstruction (denorm, stitched)
      (bottom)Per-channel difference: recon - BIN, aligned on a common time grid
    If filename_idx is None or BIN is missing, it will plot only the reconstruction.
    """
    import matplotlib.pyplot as plt
    from pymavlink import mavutil
    import os, numpy as np

    images_dir = os.path.join(output_dir, "images")
    os.makedirs(images_dir, exist_ok=True)

    # 1) Denormalize and stitch [B,T,4] -> [L,4]
    recon_denorm = recon_norm * (std[None, None, :]) + (mean[None, None, :])
    recon_seq = _stitch_overlap_mean(recon_denorm, stride=stride)  # [L,4]

    # 2) Try to read matching BIN -> ts, bin_seq [N,4]
    ts = None
    bin_seq = None
    if isinstance(filename_idx, int):
        bin_path = os.path.join(output_dir, "logs", f"{filename_idx:08d}.BIN")
        if os.path.exists(bin_path):
            mlog = mavutil.mavlink_connection(bin_path)
            rc_ok = not rc_log_filter
            times = []
            vals = {ch: [] for ch in [f"C{i}" for i in range(1, 17)]}
            while True:
                msg = mlog.recv_match()
                if msg is None:
                    break
                if msg.get_type() == "MSG" and rc_log_filter:
                    if "LOG RC" in msg.Message:
                        rc_ok = True
                    if "STOP RC" in msg.Message:
                        rc_ok = False
                if msg.get_type() == "RCOU" and rc_ok:
                    times.append(msg.TimeUS / 1e6)
                    for i in range(1, 17):
                        ch = f"C{i}"
                        vals[ch].append(getattr(msg, ch) if hasattr(msg, ch) else 0)

            if times:
                ts = np.array(times, dtype=np.float64)
                bin_seq = np.stack(
                    [np.array(vals[ch], dtype=np.float32) for ch in channels], axis=1
                )  # [N,4]

    # 3) Build plots
    if bin_seq is None or ts is None:
        # Only reconstruction available
        fig, ax = plt.subplots(1, 1, figsize=(12, 5))
        t_rec = np.arange(recon_seq.shape[0])
        for c_idx, ch in enumerate(channels):
            ax.plot(t_rec, recon_seq[:, c_idx], "--", label=f"{ch} (recon)")
        ax.set_title(title + " (no BIN available)")
        ax.set_xlabel("sample")
        ax.set_ylabel("servo (raw)")
        ax.grid(True)
        ax.legend(ncol=4, fontsize=9)
        out_path = os.path.join(images_dir, "lstm_triptych_recon_only.png")
        plt.tight_layout()
        plt.savefig(out_path, dpi=150)
        plt.close(fig)
        return

    # 4) Align recon to BIN time with a common grid and compute diff
    L = recon_seq.shape[0]
    ts_recon = np.linspace(ts[0], ts[-1], L)  # synthetic time for recon
    # choose a common time grid in the overlap range
    t0 = max(ts[0], ts_recon[0])
    t1 = min(ts[-1], ts_recon[-1])
    if t1 <= t0:
        # degenerate overlap; fall back to independent axes
        common_t = None
    else:
        common_t = np.linspace(t0, t1, min(len(ts), L))

    # Interpolate to common grid if possible
    if common_t is not None:
        bin_interp = np.column_stack(
            [np.interp(common_t, ts, bin_seq[:, i]) for i in range(bin_seq.shape[1])]
        )
        rec_interp = np.column_stack(
            [
                np.interp(common_t, ts_recon, recon_seq[:, i])
                for i in range(recon_seq.shape[1])
            ]
        )
        diff = rec_interp - bin_interp  # [K,4]
    else:
        bin_interp, rec_interp, diff = None, None, None

    # 5) Triptych
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    # (top) BIN
    for c_idx, ch in enumerate(channels):
        axes[0].plot(ts, bin_seq[:, c_idx], label=f"{ch} (BIN)")
    axes[0].set_title(f"{title} – BIN")
    axes[0].set_ylabel("servo (raw)")
    axes[0].grid(True)
    axes[0].legend(ncol=4, fontsize=9)

    # (mid) Recon
    for c_idx, ch in enumerate(channels):
        axes[1].plot(ts_recon, recon_seq[:, c_idx], "--", label=f"{ch} (recon)")
    axes[1].set_title(f"{title} – Reconstruction")
    axes[1].set_ylabel("servo (raw)")
    axes[1].grid(True)
    axes[1].legend(ncol=4, fontsize=9)

    # (bottom) Difference on common grid (if available)
    if diff is not None:
        for c_idx, ch in enumerate(channels):
            axes[2].plot(common_t, diff[:, c_idx], label=f"{ch} (recon − BIN)")
        axes[2].axhline(0.0, linestyle=":", linewidth=0.8)
        axes[2].set_title("Channel Differences (aligned)")
        axes[2].set_xlabel("time (s)")
        axes[2].set_ylabel("Δ servo")
        axes[2].grid(True)
        axes[2].legend(ncol=4, fontsize=9)
    else:
        axes[2].text(
            0.5,
            0.5,
            "No overlap to compute differences",
            ha="center",
            va="center",
            transform=axes[2].transAxes,
        )
        axes[2].set_axis_off()

    plt.tight_layout()
    out_name = f"{filename_idx:08d}-lstm_triptych.png"
    out_path = os.path.join(images_dir, out_name)
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


class FuzzConfig:
    def __init__(self, args, logger_instance):
        """Initialize the FuzzConfig object with the provided arguments.

        Args:
            args: Command-line arguments or configuration settings.
        """
        # Register signal handlers
        # signal.signal(signal.SIGINT, self.signal_handler)
        # signal.signal(signal.SIGTERM, self.signal_handler)
        self.fuzzer_shutdown_requested = False  # Handles the entire fuzzer shutdown

        global logger
        logger = logger_instance
        # Load configuration from config YAML file first
        self.config_file = getattr(args, "config", None)
        self.config = {}
        if self.config_file and os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                self.config = yaml.safe_load(f)
                logger.info(f"Loaded configuration from {self.config_file}")
        self.fuzzer_state = FuzzState.Init
        self.fuzzer_queue = []
        # Or queue.Queue (if we have multiple producers)
        # Just check if the file contains at least src_dir and peripheral_file
        if not self.config.get("peripheral_file") and not self.config.get(
            "autopilot_type"
        ):
            raise ValueError(
                "Atleast autopilot_type and peripheral_file are required in the config file."
                "Atleast autopilot_type and peripheral_file are required in the config file."
            )
        # Load peripheral mapping from peripheral YAML file
        self.peripheral_file = (
            getattr(args, "peripheral_file", None)
            if getattr(args, "peripheral_file", None)
            else self.config.get("peripheral_file")
        )
        # By default we have Ardupilot
        self.autopilot_type = (
            getattr(args, "autopilot_type", None)
            if getattr(args, "autopilot_type", None)
            else self.config.get("autopilot_type", "ardupilot")
        )
        logger.info(f"Using autopilot type: {self.autopilot_type}")

        self.peripheral_mapping = {}
        if self.peripheral_file and os.path.exists(self.peripheral_file):
            with open(self.peripheral_file, "r") as f:
                self.peripheral_mapping = yaml.safe_load(f)
                logger.info(f"Loaded peripheral mapping from {self.peripheral_file}")

        # Allow configuring the modes via config
        # Current fallbacks are ['GUIDED', 'AUTO'], I think these are the most common
        self.supported_modes = self.config.get("supported_modes") or ["GUIDED", "AUTO"]

        self.vehicle = (
            getattr(args, "vehicle", None)
            if getattr(args, "vehicle", None)
            else self.config.get("vehicle", "copter")
        )
        # Select the model that the oracle uses
        self.oracle_model = self.config.get("oracle_model", "lstm")
        logger.info(f"Using oracle model: {self.oracle_model}")
        if self.oracle_model not in detection_models:
            logger.error("Unknown oracle model specified, using default: lstm")
            self.oracle_model = "lstm"

        # ---- SSL (semi-supervised) settings: part 1 (no paths yet) ----
        self.ssl_buffer = []  # list of (window[T,C], y, weight)
        self.ssl_max_buf = int(self.config.get("ssl_max_buf", 2048))
        self.ssl_min_train = int(self.config.get("ssl_min_train", 256))
        self.ssl_train_every = int(
            self.config.get("ssl_train_every", 3)
        )  # every N missions
        self.ssl_head_lr = float(self.config.get("ssl_head_lr", 1e-4))
        self.ssl_head_batch = int(self.config.get("ssl_head_batch", 128))
        self.ssl_head_epochs = int(self.config.get("ssl_head_epochs", 3))
        self.ssl_normal_z = float(
            self.config.get("ssl_normal_z", 0.2)
        )  # confident normal
        self.ssl_anom_z = float(self.config.get("ssl_anom_z", 2.0))  # confident anomaly
        self.ssl_w_normal = float(
            self.config.get("ssl_w_normal", 0.5)
        )  # down-weight normals
        self.ssl_w_anom = float(
            self.config.get("ssl_w_anom", 1.0)
        )  # anomalies full weight
        self.ssl_last_train_sims = -1

        # LSTM AE dims (mirror sigma_calc_lstm choices)
        self.lstm_window = int(self.config.get("lstm_window", 128))
        self.lstm_stride = int(self.config.get("lstm_stride", 64))

        if self.lstm_window <= 0:
            logger.warning("Invalid lstm_window (%s); forcing to 128", self.lstm_window)
            self.lstm_window = 128
        if self.lstm_stride <= 0:
            logger.warning("Invalid lstm_stride (%s); forcing to 64", self.lstm_stride)
            self.lstm_stride = 64

        self.lstm_hidden = int(self.config.get("lstm_hidden", 32))

        # Setup files - command line args override yaml config
        self.sitl_bin = (
            getattr(args, "bin", None)
            if getattr(args, "bin", None)
            else self.config.get("sitl_bin", None)
        )

        # Setup the new dir
        self.src_dir = None
        if self.autopilot_type == "ardupilot":
            self.src_dir = getattr(args, "ap_dir", None) or self.config.get(
                "ap_dir", "/ardupilot"
            )
        elif self.autopilot_type == "px4":
            self.src_dir = getattr(args, "px4_dir", None) or self.config.get(
                "px4_dir", "/px4"
            )

        if self.src_dir is None:
            raise ValueError("Failed to obtain src_dir")

        # Get script directory
        self.script_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "scripts/"
        )
        # Handle the case where we don't have a SITL binary
        if self.autopilot_type == "ardupilot":
            if self.sitl_bin is None:
                # Check if we have the binary at src_dir + build/sitl/bin/ardu + vehicle
                vehicle_bin = f"ardu{self.vehicle}"
                sitl_bin_path = os.path.join(
                    self.src_dir, "build", "sitl", "bin", vehicle_bin
                )
                if file_exists(sitl_bin_path):
                    self.sitl_bin = sitl_bin_path
                else:
                    raise FileNotFoundError(
                        "SITL binary not found. Please provide a valid path."
                    )
        elif self.autopilot_type == "px4":
            # PX4 uses make command instead of direct binary path
            # Check if PX4 directory exists and has Makefile
            if not file_exists(self.src_dir):
                raise FileNotFoundError(
                    f"PX4 directory not found at {self.src_dir}. Please provide a valid path."
                )
            makefile_path = os.path.join(self.src_dir, "Makefile")
            if not file_exists(makefile_path):
                raise FileNotFoundError(
                    f"PX4 Makefile not found at {makefile_path}. Please ensure PX4 is properly installed."
                )
            # Set a placeholder for sitl_bin to pass validation (will use make command instead)
            self.sitl_bin = makefile_path
            logger.info(f"PX4 directory found at: {self.src_dir}")

        self.xml_file = (
            getattr(args, "xml", None)
            if getattr(args, "xml", None)
            else self.config.get("xml_file")
        )
        # Random param set
        self.random_param_prob = getattr(args, "random_param_probability", 0.1)

        # Auto mission configuration
        self.auto_mission_enabled = False
        self.auto_mission_path = (
            getattr(args, "auto_mission", None)
            if getattr(args, "auto_mission", None)
            else self.config.get("mission_file")
        )
        if self.auto_mission_path and file_exists(self.auto_mission_path):
            logger.info("Mission file found, AUTO mode testing enabled")
            self.auto_mission_enabled = True

        # Variables - command line args override yaml config
        self.peripheral_under_test = (
            getattr(args, "peripheral", None)
            if getattr(args, "peripheral", None)
            else self.config.get("peripheral")
        )

        # Mission control
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
        if self.autopilot_type == "ardupilot":
            if file_exists(self.sitl_bin) and file_exists(self.src_dir):
                logger.info(f"Using SITL binary: {self.sitl_bin}")
                logger.info(f"Using Ardupilot directory: {self.src_dir}")
            # Parameter file for Ardupilot
            # Now we have a parameter dictionary
            param_mapping = {
                "copter": "copter.parm",
                "plane": "plane-jsbsim.parm",
                "rover": "rover.parm",
            }
            self.param_file = os.path.join(
                self.src_dir,
                "Tools/autotest/default_params/",
                param_mapping.get(self.vehicle, "None"),
            )
            if file_exists(self.param_file):
                logger.info("Using parameter file: " + self.param_file)
        elif self.autopilot_type == "px4":
            if file_exists(self.src_dir):
                logger.info(f"Using PX4 directory: {self.src_dir}")

        # Calibration settings
        self.calibration_active = False
        self.calibration_mode_list = list(
            itertools.product(self.supported_modes, repeat=MAX_MODE_CHANGES)
        )
        logger.debug(f"Printing the calibration mode list {self.calibration_mode_list}")
        self.enumerated_modes = {
            t: i for i, t in enumerate(self.calibration_mode_list, start=1)
        }
        self.fuzz_enum_mode = 0
        # Counter to check each calibration mode generated above
        self.calibration_modes_ctr = 0
        self.calibration_threshold = None
        self.calibration_vals = None
        calibration_rounds = (
            getattr(args, "calibration_rounds", None)
            if getattr(args, "calibration_rounds", None)
            else self.config.get("calibration_rounds", 10)
        )
        self.calibration_rounds = calibration_rounds
        if self.calibration_rounds < len(self.calibration_mode_list):
            logger.error("The configured number of calibration rounds is too low.")
            raise ValueError("Calibration rounds are too low.")
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
        assert getattr(
            args, "fuzzer_temp_dir", None
        ), "Temporary directory must be provided"
        self.fuzzer_temp_dir = getattr(args, "fuzzer_temp_dir", None)
        self.fuzzer_temp_input_dir = self.fuzzer_temp_dir + "/input"
        # Create the temporary input directory if it doesn't exist
        if not os.path.exists(self.fuzzer_temp_input_dir):
            os.makedirs(self.fuzzer_temp_input_dir)
            logger.info(
                f"Created temporary input directory: {self.fuzzer_temp_input_dir}"
            )

        self.ssl_head_path = os.path.join(self.fuzzer_temp_dir, "lstm_head.pt")

        # Check if we have additional parameters in the peripheral mapping
        self.generic_params = getattr(args, "generic_params", None)
        if self.generic_params is None:
            # Try to load from config file first
            self.generic_params = self.config.get("generic_params", [])

        # Parse generic parameters and initialize default parameter set
        self.default_parameter_set = self._parse_generic_params()
        if not self.default_parameter_set:
            logger.warning("No generic parameters loaded")

        self.setup()
        if self.msg_freq:
            logger.info("Setting the fuzzing interval to match message frequency")
            self.fuzz_interval = self.msg_freq

        # Setup the coverage metrics
        self.coverage_class = CoverageData(
            src_dir=self.src_dir, fuzz_dir=self.fuzzer_temp_dir
        )

        self.last_scores = {
            "dtw": float("nan"),
            "z_dtw": float("nan"),
            "lstm_err": float("nan"),
            "p_anom": float("nan"),
        }

    def _parse_numeric_value(self, value_str):
        """Parse a numeric value string, preserving int/float type.

        Args:
            value_str: String representation of a numeric value

        Returns:
            Parsed numeric value (int or float) or None if invalid
        """
        try:
            # Check if it's an integer (no decimal point)
            if "." not in value_str and "e" not in value_str.lower():
                return int(value_str)
            else:
                return float(value_str)
        except ValueError:
            return None

    def _parse_generic_params(self):
        """Parse generic_params from config and return a structured parameter set.

        Expected format in config:
        generic_params:
          - "param_name MIN MAX STEP"
          - "param_name MIN MAX"
          - "param_name"

        Returns:
            dict: Dictionary mapping parameter names to their constraints
        """
        if not self.generic_params:
            return {}

        parsed_params = {}

        for param_line in self.generic_params:
            if not param_line or not isinstance(param_line, str):
                continue

            parts = param_line.strip().split()
            if not parts:
                continue

            param_name = parts[0]

            if len(parts) == 1:
                # Just parameter name, no constraints
                parsed_params[param_name] = {}
            elif len(parts) == 3:
                # param_name MIN MAX
                min_val = self._parse_numeric_value(parts[1])
                max_val = self._parse_numeric_value(parts[2])

                if min_val is not None and max_val is not None:
                    parsed_params[param_name] = {"min": min_val, "max": max_val}
                else:
                    logger.warning(
                        f"Invalid numeric values for parameter {param_name}: {parts[1]}, {parts[2]}"
                    )
                    parsed_params[param_name] = {}
            elif len(parts) == 4:
                # param_name MIN MAX STEP
                min_val = self._parse_numeric_value(parts[1])
                max_val = self._parse_numeric_value(parts[2])
                step_val = self._parse_numeric_value(parts[3])

                if min_val is not None and max_val is not None and step_val is not None:
                    parsed_params[param_name] = {
                        "min": min_val,
                        "max": max_val,
                        "step": step_val,
                    }
                else:
                    logger.warning(
                        f"Invalid numeric values for parameter {param_name}: {parts[1]}, {parts[2]}, {parts[3]}"
                    )
                    parsed_params[param_name] = {}
            else:
                logger.warning(f"Invalid parameter format: {param_line}")

        logger.info(f"Parsed {len(parsed_params)} generic parameters for fuzzing")
        return parsed_params

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
                            "error": str(e),
                            "component": "periodic_send",
                            "timestamp": time.time(),
                        }
                    )
                    self.fuzzer_shutdown_requested = True

                time.sleep(1 / frequency)

    def periodic_replayer(self, frequency, xml_msg, default_values):
        """Send periodic messages based on the specified frequency for replayer."""
        pass

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

        self.fuzzer_stats.setdefault("bugs_dtw", 0)
        self.fuzzer_stats.setdefault("bugs_lstm", 0)

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
            subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=self.src_dir)
            .strip()
            .decode("utf-8")
        )
        logger.debug("Source Under Testing commit hash %s", src_commit_hash)

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
            # Need to add additional parameters for plane to land
            if self.vehicle == "plane":
                self.peripheral_config["parameters"].update({"RTL_AUTOLAND": 1})
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
            logger.debug(
                "No default configuration parameters set for fuzzing, returning"
            )
            return

        # Select a random parameter from the parsed parameter set
        selected_param = random.choice(list(self.default_parameter_set.keys()))
        param_constraints = self.default_parameter_set[selected_param]

        # Generate value based on parameter constraints
        if "min" in param_constraints and "max" in param_constraints:
            min_val = param_constraints["min"]
            max_val = param_constraints["max"]

            if "step" in param_constraints:
                # Use step-based generation for discrete values
                step_val = param_constraints["step"]
                if step_val == 0:
                    logger.warning(
                        f"Step value for parameter {selected_param} is 0, using default value"
                    )
                    random_val = min_val
                else:
                    random_val = gen_int_step(min_val, max_val, step_val)
            else:
                # Generate continuous value between min and max
                if isinstance(min_val, float) or isinstance(max_val, float):
                    random_val = random.uniform(min_val, max_val)
                else:
                    random_val = random.randint(int(min_val), int(max_val))
        else:
            # No constraints specified, use heuristics based on parameter name
            if re.match(r".*ABLE$", selected_param):
                # Parameters ending in ABLE (ENABLE/DISABLE) should be 0 or 1
                random_val = random.randint(0, 1)
            else:
                # Default to generating a value using the field generator
                random_val = generate_field_value(field_type="uint8")

        logger.debug(f"Setting parameter {selected_param} to {random_val}")

        self.tcp_conn.set_param(
            param_id=selected_param,
            param_value=random_val,
            param_type="int32",
        )
        # Ensure we properly add the value to queue
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

    def _infer_px4_param_type(self, val):
        """
        Simple heuristic mapping for PX4 PARAM_SET types:
        - floats -> REAL32
        - ints   -> choose a reasonable int bucket
        You can override per-param by passing dicts in mapping: {value: X, type: "int32"}.
        """
        if isinstance(val, dict):
            v = val.get("value")
            t = val.get("type", "").lower()
            if t in (
                "uint8",
                "int8",
                "uint16",
                "int16",
                "uint32",
                "int32",
                "float",
                "double",
            ):
                return v, t
            # fallthrough to infer if "type" missing/wrong
            val = v

        if isinstance(val, float):
            return float(val), "float"
        # ints: pick smallest viable
        i = int(val)
        if 0 <= i <= 255:
            return i, "uint8"
        if -128 <= i <= 127:
            return i, "int8"
        if 0 <= i <= 65535:
            return i, "uint16"
        if -32768 <= i <= 32767:
            return i, "int16"
        # PX4 params are int32 for most integer params
        return i, "int32"

    def _readback_param_px4(self, name, expect_val, tol=1e-6, timeout=3.0):
        """
        Request a PARAM_VALUE and confirm within tolerance for floats (or exact for ints).
        """
        try:
            msg = self.tcp_conn.show_param(name, timeout=max(1.0, timeout))
            if msg is None:
                return False, None
            got = float(msg.param_value)
            # If expected was an int, compare as int
            if isinstance(expect_val, int):
                return int(round(got)) == int(expect_val), got
            # else float compare
            return abs(got - float(expect_val)) <= tol, got
        except Exception:
            return False, None

    def _apply_params_px4(self, params: dict, verify=True, reboot_if_needed=False):
        """
        Apply PX4 parameters via MAVLink PARAM_SET using TCPConn.set_param, verify, and optionally reboot.
        Mapping may contain:
        parameters:
            MNT_MODE_IN: 4
            MAV_1_MODE: {value: 10, type: "int32"}
            ...
        """
        if not params:
            logger.info("No PX4 parameters to apply.")
            return

        logger.info(f"Applying {len(params)} PX4 parameters...")
        failures = []

        for name, raw_val in params.items():
            val, ptype = self._infer_px4_param_type(raw_val)
            try:
                self.tcp_conn.set_param(
                    param_id=name, param_value=val, param_type=ptype
                )
                logger.debug(f"PARAM_SET {name}={val} ({ptype}) sent")
                # PX4 immediately updates and usually broadcasts on change; we force a read for certainty.
                if verify:
                    ok, got = self._readback_param_px4(name, val)
                    if not ok:
                        logger.warning(
                            f"Verify failed: {name} expected {val}, got {got}"
                        )
                        failures.append(name)
                    else:
                        logger.debug(f"Verified {name}={val}")
            except Exception as e:
                logger.error(f"Error setting {name}: {e}")
                failures.append(name)

        if failures:
            logger.warning(f"PX4 param apply had {len(failures)} failures: {failures}")

        # Optional reboot in case a param needs restart to take effect.
        if reboot_if_needed and failures == []:
            try:
                logger.info("Rebooting PX4 to apply parameters that require restart...")
                self.tcp_conn.reboot_and_wait_for_ack()
            except Exception as e:
                logger.error(f"PX4 reboot failed: {e}")

    def run_sim(self):
        """Run the SITL simulation with the specified vehicle and parameters."""
        if self.autopilot_type == "ardupilot":
            logger.debug("Starting Ardupilot SITL simulation")
            sitl_args = ""
            home_location = " --home -35.362938,149.165085,585,354 "
            if self.vehicle == "copter":
                sitl_args = " -S --model + -w --speedup 1 -I0"
            elif self.vehicle == "plane":
                # "-w" "-S" "--home" "-35.362938,149.165085,585,354" "--model" "plane-elevrev"  "--defaults" "/Tools/autotest/default_params/plane-jsbsim.parm"
                sitl_args = " -S --model plane-elevrev -w --speedup 1 -I0"
            elif self.vehicle == "rover":
                # "-w" "-S" "--home" "40.071375,-105.229789,1583,246" "--model" "rover"
                sitl_args = " -S --model rover -w --speedup 1 -I0"
            self.sitl_cmd = (
                self.sitl_bin
                + home_location
                + sitl_args
                + " --defaults "
                + self.param_file
            )
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
                    shlex.split(self.sitl_cmd),
                    # Comment out to debug the original binary
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid,
                    cwd=self.fuzzer_temp_dir,
                    # Reboot to ensure we have reloaded the parameters
                )
                init_conn = TCPConn()
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
        elif self.autopilot_type == "px4":
            logger.debug("Starting PX4 SITL simulation")
            # PX4 uses make command: make px4_sitl jmavsim
            # Run from the PX4 directory
            current_env = os.environ.copy()
            current_env["HEADLESS"] = "1"
            current_env["PX4_SIM_MODEL"] = "jmavsim_iris"
            self.sitl_cmd = f"./build/px4_sitl_default/bin/px4 -d"
            logger.info(f"Starting PX4 SITL with command: {self.sitl_cmd}")
            if self.calibration_active:
                assert (
                    self.fuzzing_active is False
                ), "Cannot run calibration while fuzzing is active"
            # Note: PX4 parameter files work differently than ArduPilot
            try:
                self.sim_handle = subprocess.Popen(
                    shlex.split(self.sitl_cmd),
                    # stdout=subprocess.DEVNULL,
                    # stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid,
                    env=current_env,
                    cwd=self.src_dir,  # Run from PX4 directory
                )
                time.sleep(10)  # Give some time for the simulation to start
                # init_conn = TCPConn(autopilot_type="px4")
                # Reboot to ensure we have reloaded the parameters
                # init_conn.cleanup(shutdown=False)
                self.fuzzer_stats["current_mission_time"] = time.time()
                self.tcp_conn = TCPConn(autopilot_type="px4")
                self.tcp_conn.setup_threads()
                self.sim_ready = True
                # Start the monitoring thread
                self.monitor_thread = threading.Thread(
                    target=self._monitor_sim, daemon=True
                ).start()
            except Exception as e:
                # this would just kill the entire script, so need to handle it gracefully
                logger.error(f"Error starting PX4 simulation: {e}")
        else:
            raise ValueError("Unknown software system specified")

    def _monitor_auto_mission_calibration(self):
        # In this case we need to actually set the different modes one by one and then check?
        # The idea is to ensure that we run every possible mission change that can exist and is supported by the
        # configuration
        try:
            while not self.tcp_conn.rc_monitor:
                if self.error_sleep(1):
                    raise InternalError
            self.start_fuzzing()
            mode_ctr = 0
            prev_state = None
            if self.calibration_modes_ctr < len(self.calibration_mode_list):
                mode = self.calibration_mode_list[self.calibration_modes_ctr]
                self.calibration_modes_ctr += 1
            else:
                # At this point we have exhausted all the mode combination, so we can just randomly select one
                mode = random.choice(self.calibration_mode_list)
            while self.tcp_conn.rc_monitor and self.tcp_conn.drone_in_air:
                # Instead of actually setting things randomly, we select each mode from the generated modes
                if mode_ctr < MAX_MODE_CHANGES:
                    self.tcp_conn.set_mode(mode[mode_ctr])
                    logger.debug(f"Changing mode to: {mode[mode_ctr]}")
                    prev_state = mode[mode_ctr]
                    mode_ctr += 1
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

    def _monitor_auto_mission_fuzzing(self):
        try:
            while not self.tcp_conn.rc_monitor:
                if self.error_sleep(1):
                    raise InternalError
            self.start_fuzzing()
            mode_state = []
            mode_ctr = 0
            prev_state = None
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
                    mode_state.append(mode)
                    mode_ctr += 1
                    prev_state = mode
                elif mode_ctr >= MAX_MODE_CHANGES and prev_state != "AUTO":
                    logger.debug("Reached mode change limit, not changing mode anymore")
                    self.tcp_conn.set_mode("AUTO")
                    prev_state = "AUTO"
                    logger.debug("Resetting Setting mode to AUTO")
                if random.random() < self.random_param_prob:  # Randomly set a parameter
                    self.random_param_set()
                if self.error_sleep(3):
                    raise InternalError
            self.wait_for_condition(lambda: not self.tcp_conn.rc_monitor, timeout=60)
            while self.tcp_conn.drone_in_air:
                if self.error_sleep(1):
                    raise InternalError
            self.stop_fuzzing()
            # Check if we can figure out what StateEnum we are in
            selected_mode = self.enumerated_modes.get(tuple(mode_state))
            if not selected_mode:
                logger.warning("Could not find the selected mode in the enumeration")
            self.fuzz_enum_mode = selected_mode
            return
        except InternalError:
            logger.error("Warning detected, stopping fuzzing")
            if self.fuzzing_active:
                self.stop_fuzzing()
            error_queue.put(
                {
                    "type": "fuzzer_error",
                    "error": "Error inside error_sleep",
                    "component": "monitor_auto_mission_fuzzing",
                    "timestamp": time.time(),
                }
            )
            # self.handle_errors()
            return

    def monitor_auto_mission(self):
        """Monitor the drone during an automatic mission."""
        # Wait till the drone is in air
        # logger.info("Waiting till drone is in air")
        # self.wait_for_condition(lambda: self.tcp_conn.rc_monitor, timeout=60)
        if self.calibration_active:
            self._monitor_auto_mission_calibration()
        else:
            self._monitor_auto_mission_fuzzing()

    def upload_auto_mission(self, mission_file):
        """Upload a mission from a waypoint file using MAVProxy's waypoint module.
        Based on the implementation from mavproxy_oldwp.py

        Args:
            mission_file: Path to the mission file (.waypoints format).
        """
        # Load waypoints from file
        waypoints = mavwp.MAVWPLoader()
        waypoints.target_system = self.tcp_conn.conn.target_system  # type: ignore
        waypoints.target_component = self.tcp_conn.conn.target_component  # type: ignore

        try:
            # Remove leading and trailing quotes in filename
            waypoints.load(mission_file.strip('"'))
        except Exception as msg:
            logger.error(f"Unable to load {mission_file} - {msg}")
            return

        logger.info(f"Loaded {waypoints.count()} waypoints from {mission_file}")

        # Clear any existing mission
        self.tcp_conn.conn.waypoint_clear_all_send()  # type: ignore

        if waypoints.count() == 0:
            logger.warning("No waypoints to upload")
            return

        # Track upload progress
        upload_start = time.time()
        loading_waypoints = True
        loading_waypoint_lasttime = time.time()

        # Timeout for mission requests
        timeout = 4
        if self.autopilot_type == "px4":
            timeout = 3

        # Send waypoint count
        self.tcp_conn.conn.waypoint_count_send(waypoints.count())  # type: ignore
        logger.info(f"Sent waypoint count: {waypoints.count()}")

        # Respond to mission requests
        requested_waypoints = set()
        received_count = 0

        while loading_waypoints and received_count < waypoints.count():
            try:
                # Wait for mission request message (handle both MISSION_REQUEST and MISSION_REQUEST_INT)
                msg = self.tcp_conn.mission_msg_queue.get(timeout=timeout)

                # Check if we're still loading waypoints and within timeout
                if not loading_waypoints:
                    break
                if time.time() > loading_waypoint_lasttime + 10.0:
                    logger.error("Mission upload timeout exceeded")
                    loading_waypoints = False
                    break

                seq = msg.seq

                # Validate sequence number
                if seq >= waypoints.count():
                    logger.error(
                        f"Request for bad waypoint {seq} (max {waypoints.count() - 1})"
                    )
                    continue

                # Get the waypoint
                wp = waypoints.wp(seq)
                if wp is None:
                    logger.error(f"Could not get waypoint {seq}")
                    continue

                # Set target system and component
                wp.target_system = self.tcp_conn.conn.target_system  # type: ignore
                wp.target_component = self.tcp_conn.conn.target_component  # type: ignore

                # Check if we should use MISSION_ITEM_INT
                # For now, send as MISSION_ITEM (can be enhanced to support INT format)
                wp_send = wp

                # Send the waypoint
                self.tcp_conn.conn.mav.send(wp_send)  # type: ignore
                logger.info(f"Sending waypoint {seq}/{waypoints.count() - 1}")

                requested_waypoints.add(seq)
                received_count += 1
                loading_waypoint_lasttime = time.time()

                # Check if we've sent all waypoints
                if seq == waypoints.count() - 1:
                    loading_waypoints = False
                    logger.info(
                        f"Sent all {waypoints.count()} waypoints in {time.time() - upload_start:.2f}s"
                    )
                    break

            except Empty:
                # Queue timeout - check if overall timeout exceeded
                if time.time() > loading_waypoint_lasttime + 10.0:
                    logger.error("Mission upload timeout exceeded waiting for requests")
                    loading_waypoints = False
                    break
                # Otherwise continue waiting
                continue
            except Exception as e:
                logger.error(f"Error in mission upload: {e}")
                # Check timeout
                if time.time() > loading_waypoint_lasttime + 10.0:
                    logger.error("Mission upload timed out waiting for requests")
                    break

        if received_count < waypoints.count():
            logger.warning(
                f"Only sent {received_count} of {waypoints.count()} waypoints"
            )

    def standard_guided(self, fuzzing=True):
        """Perform a standard guided mission.

        Args:
            fuzzing: Whether to enable fuzzing during the mission.
        """
        self.tcp_conn.set_mode("GUIDED")
        if not self._wait_prearm_ok(timeout=180):
            # Mark this sim as failed-but-recoverable; don't run oracle on junk
            logger.error("PreArm check failed; aborting this mission cleanly.")
            self.tcp_conn.rc_monitor = False
            return  # caller will cleanup_sim(oracle=False) if you wire it that way

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
            if not self._wait_prearm_ok(timeout=180):
                logger.error("PreArm check failed before AUTO; aborting mission.")
                return
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

    def _wait_prearm_ok(self, timeout=180):
        """Block until ArduPilot reports it's ready to arm (prearm checks passed)."""
        ok = self.wait_for_condition(
            lambda: getattr(self.tcp_conn, "drone_ready", False), timeout=timeout
        )
        if not ok:
            logger.error(
                "PreArm never cleared within %ss (drone_ready stayed False).", timeout
            )
        return ok

    def _series_to_matrix(
        self, series, fields=("servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw")
    ):
        return np.array([[pkt[f] for f in fields] for pkt in series], dtype=np.float32)

    def _build_windows(self, M, T=None, stride=None):
        T = int(T or self.lstm_window)
        stride = int(stride if stride is not None else self.lstm_stride)
        if T <= 0:
            raise ValueError(f"lstm_window must be > 0, got {T}")
        if stride <= 0:
            # fallback to full-overlap if misconfigured
            logger.warning("lstm_stride was <= 0; defaulting to 1")
            stride = 1
        N, C = M.shape
        if N < T:
            pad = np.repeat(M[-1:], T - N, axis=0)
            M = np.vstack([M, pad])
            N = T

        starts = range(0, max(1, N - T + 1), stride)
        return np.stack([M[s : s + T] for s in starts], axis=0)

    def _normalize(self, X, mean=None, std=None, eps=1e-8):
        if mean is None or std is None:
            mean = X.mean(axis=(0, -2), keepdims=False)
            std = X.std(axis=(0, -2), keepdims=False)
        return (X - mean) / (std + eps), mean, std

    def _ssl_push(self, Wn, y, weight=1.0):
        pos = 0
        for i in range(Wn.shape[0]):
            self.ssl_buffer.append((Wn[i], int(y), float(weight)))
            pos += int(y)
        if len(self.ssl_buffer) > self.ssl_max_buf:
            self.ssl_buffer = self.ssl_buffer[-self.ssl_max_buf :]
        logger.debug(
            f"SSL push: y={y}, added={Wn.shape[0]}, pos_ratio~={pos/max(1,Wn.shape[0]):.2f}, buffer={len(self.ssl_buffer)}"
        )

    def _sigma_calc_lstm(self):
        """
        Train LSTM autoencoder on golden runs; set reconstruction-error band.
        Persists model + normalization + thresholds for reuse.
        Logs full training details every epoch.
        """
        import torch, torch.nn as nn

        # 1) Assemble all golden matrices and windows
        mats = [self._series_to_matrix(s) for s in self.golden_rc_vals]  # [Ni,4]
        T = int(self.config.get("lstm_window", 128))
        stride = int(self.config.get("lstm_stride", 64))
        wins = [
            self._build_windows(m, T=T, stride=stride) for m in mats
        ]  # list [Bi,T,4]
        X = np.concatenate(wins, axis=0)  # [B,T,4]

        # 2) Normalize (fit on goldens)
        Xn, mean, std = self._normalize(X)  # per-channel

        # 3) Torch dataset
        device = "cuda" if torch.cuda.is_available() else "cpu"
        Xt = torch.from_numpy(Xn).float().to(device)  # [B,T,4]

        # 4) Define tiny LSTM AE
        C = Xt.shape[-1]
        H = int(self.config.get("lstm_hidden", 32))

        class LSTMAE(nn.Module):
            def __init__(self, c=C, h=H):
                super().__init__()
                self.enc = nn.LSTM(
                    input_size=c, hidden_size=h, num_layers=1, batch_first=True
                )
                self.dec = nn.LSTM(
                    input_size=h, hidden_size=h, num_layers=1, batch_first=True
                )
                self.out = nn.Linear(h, c)

            def forward(self, x):
                z, _ = self.enc(x)  # [B,T,h]
                y, _ = self.dec(z)  # [B,T,h]
                return self.out(y)  # [B,T,c]

        model = LSTMAE().to(device)
        opt = torch.optim.Adam(
            model.parameters(), lr=float(self.config.get("lstm_lr", 1e-3))
        )
        loss_fn = nn.MSELoss(reduction="none")
        # Force 100 epochs (as requested); allow override through config if you want later.
        epochs = 500
        bs = int(self.config.get("lstm_batch", 64))

        B = Xt.size(0)
        logger.info(
            f"LSTM AE pretraining on goldens: B={B}, T={T}, C={C}, H={H}, epochs={epochs}, batch={bs}"
        )

        # 5) Train (with full logging each epoch)
        for ep in range(1, epochs + 1):
            perm = torch.randperm(B, device=device)
            running = 0.0
            steps = 0
            for i in range(0, B, bs):
                idx = perm[i : i + bs]
                batch = Xt[idx]
                recon = model(batch)
                mse = loss_fn(recon, batch).mean(dim=(1, 2))  # per-window scalar
                loss = mse.mean()
                opt.zero_grad()
                loss.backward()
                opt.step()
                running += float(loss.item())
                steps += 1
            ep_loss = running / max(1, steps)
            logger.info(f"[LSTM][pretrain] epoch {ep:03d}/{epochs} loss={ep_loss:.6e}")

        # 6) Compute golden recon error distribution for thresholds
        with torch.no_grad():
            recon = model(Xt)
            per_win = loss_fn(recon, Xt).mean(dim=(1, 2)).detach().cpu().numpy()
        mu, sigma = float(per_win.mean()), float(per_win.std())
        self.lstm_err_mean = mu
        self.lstm_err_std = sigma
        self.lstm_err_min = mu - 2 * sigma
        self.lstm_err_max = mu + 2 * sigma
        logger.info(
            f"[LSTM] recon mu={mu:.6e} std={sigma:.6e} band=[{self.lstm_err_min:.6e},{self.lstm_err_max:.6e}]"
        )

        # 7) Persist artifacts
        torch.save(model.state_dict(), os.path.join(self.fuzzer_temp_dir, "lstm_ae.pt"))
        np.save(os.path.join(self.fuzzer_temp_dir, "lstm_norm_mean.npy"), mean)
        np.save(os.path.join(self.fuzzer_temp_dir, "lstm_norm_std.npy"), std)

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
        self.dtw_mean = float(mean)
        self.dtw_std = float(std_dev)
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
            self._sigma_calc_dtw()
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

    def cleanup_sim(self, oracle=True):
        """Cleanup the simulation and reset states."""
        # Stop fuzzing first
        if self.fuzzing_active:
            self.stop_fuzzing()

        # Reset the time and state
        self.fuzzer_stats["current_mission_time"] = 0.0
        self.sim_ready = False

        # Step 1: Terminate TCP connection and collect RC values
        if self.tcp_conn:
            if self.fuzzer_shutdown_requested:
                # If shutdown requested, just cleanup without collecting data
                self.tcp_conn.cleanup()
                self.rcou_vals = []
            else:
                # Normal cleanup - collect RC channel data
                self.rcou_vals = self.tcp_conn.cleanup()
                if not self.rcou_vals:
                    self.rcou_vals = []

        # Step 2: Terminate the simulation
        if hasattr(self, "sim_handle") and self.sim_handle:
            self.sim_handle.terminate()
            logger.info("Simulation terminated.")

        # Give some time for cleanup to complete
        time.sleep(1)
        self.coverage_class.update()

        # Step 3: Call the oracle (only if we have data and not shutting down)
        if not self.fuzzer_shutdown_requested and self.rcou_vals and oracle:
            if self.calibration_active:
                # During calibration, just collect the golden values
                self.golden_rc_vals.append(self.rcou_vals)
                logger.debug(
                    f"Collected calibration data: {len(self.rcou_vals)} RC samples"
                )
            else:
                # During fuzzing, run anomaly detection
                if not self.min_fuzz_threshold:
                    # Calculate thresholds if not already done
                    self.sigma_calc()
                # Run the oracle to detect anomalies
                self.oracle()

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
            self.fuzzer_stats["bugs_dtw"] = self.fuzzer_stats.get("bugs_dtw", 0) + 1

            # Save inputs for later analysis
            fd, input_file = tempfile.mkstemp(
                suffix=".txt",
                prefix="inputs-anomalous-",
                dir=self.fuzzer_temp_input_dir,
            )
            # Save the coverage data
            self.coverage_class.archive_data(filename=log_content)
            # Create an image for later analysis
            save_diff_img(
                filename=log_content,
                fuzz_enum_mode=self.fuzz_enum_mode,
                script_dir=self.script_dir,
                output_dir=self.fuzzer_temp_dir,
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
        z = (
            (distance - self.dtw_mean) / (self.dtw_std + 1e-8)
            if getattr(self, "dtw_mean", None) is not None
            and getattr(self, "dtw_std", None) is not None
            else float("nan")
        )
        self.last_dtw_distance = float(distance)

        self.last_scores = {
            "dtw": float(distance),
            "z_dtw": z,
            "lstm_err": float("nan"),
            "p_anom": float("nan"),
        }

    def oracle_lstm(self):
        """Semi-supervised LSTM classifier on servo data with DTW weak labels + AE recon.
        - Uses DTW (via oracle_dtw) as weak supervisor (no DTW core changes).
        - Trains SSL head periodically; logs every epoch.
        - Saves viz: original vs reconstructed + metrics.
        """
        import torch, torch.nn as nn

        device = "cuda" if torch.cuda.is_available() else "cpu"

        # --- (A) Ensure up-to-date DTW score and side-effects (NO core change) ---
        prev_crashes = self.fuzzer_stats["potential_crashes"]
        dtw_dist = float(getattr(self, "last_dtw_distance", float("nan")))
        z_dtw = float(self.last_scores.get("z_dtw", float("nan")))
        dtw_already_flagged = self.fuzzer_stats["potential_crashes"] > prev_crashes
        is_dtw_band_anom = (dtw_dist < self.min_fuzz_threshold) or (
            dtw_dist > self.max_fuzz_threshold
        )

        # --- (B) Load AE + norm artifacts (trained in sigma_calc_lstm) ---
        C = 4
        H = self.lstm_hidden

        class LSTMAE(nn.Module):
            def __init__(self, c=C, h=H):
                super().__init__()
                self.enc = nn.LSTM(
                    input_size=c, hidden_size=h, num_layers=1, batch_first=True
                )
                self.dec = nn.LSTM(
                    input_size=h, hidden_size=h, num_layers=1, batch_first=True
                )
                self.out = nn.Linear(h, c)

            def forward(self, x):
                z, _ = self.enc(x)
                y, _ = self.dec(z)
                return self.out(y)

        model = LSTMAE().to(device)
        pt = os.path.join(self.fuzzer_temp_dir, "lstm_ae.pt")
        mean = np.load(os.path.join(self.fuzzer_temp_dir, "lstm_norm_mean.npy"))
        std = np.load(os.path.join(self.fuzzer_temp_dir, "lstm_norm_std.npy"))
        model.load_state_dict(torch.load(pt, map_location=device))
        model.eval()

        # --- (C) Window current mission servo data and score AE recon ---
        M = self._series_to_matrix(self.rcou_vals)  # [N,4]
        W = self._build_windows(
            M, T=self.lstm_window, stride=self.lstm_stride
        )  # [B,T,4]
        Wn, _, _ = self._normalize(W, mean=mean, std=std)
        Xt = torch.from_numpy(Wn).float().to(device)

        with torch.no_grad():
            recon = model(Xt)
            mse_per_win = ((recon - Xt) ** 2).mean(dim=(1, 2)).cpu().numpy()
        recon_np = recon.detach().cpu().numpy()  # [B,T,4] normalized recon
        lstm_err = float(np.mean(mse_per_win))  # mission-level AE score

        # --- (D) Weak labels via DTW confidence -> build SSL buffer ---
        if not np.isnan(z_dtw):
            if abs(z_dtw) < self.ssl_normal_z:
                self._ssl_push(Wn, y=0, weight=self.ssl_w_normal)  # confident normal
            elif z_dtw > self.ssl_anom_z:
                self._ssl_push(Wn, y=1, weight=self.ssl_w_anom)  # confident anomaly
            # else: ignore uncertain

        # --- (E) Periodic fine-tune of classifier head on AE embeddings (verbose) ---
        if not hasattr(self, "ssl_last_train_sims"):
            self.ssl_last_train_sims = -(10**9)
        if (
            len(self.ssl_buffer) >= self.ssl_min_train
            and (self.fuzzer_stats["simulations_completed"] - self.ssl_last_train_sims)
            >= self.ssl_train_every
        ):
            n_trained = _ssl_finetune_head(self, model, device)
            self.ssl_last_train_sims = self.fuzzer_stats["simulations_completed"]
            logger.info(
                f"[SSL] trained head on {n_trained} windows; buf={len(self.ssl_buffer)}"
            )

        # --- (F) Inference with head (probability) ---
        p_anom = None
        head = _LSTMHead(h=self.lstm_hidden).to(device)
        if os.path.exists(self.ssl_head_path):
            head.load_state_dict(torch.load(self.ssl_head_path, map_location=device))
            head.eval()
            with torch.no_grad():
                z_all, _ = model.enc(Xt)
                logits = head(z_all.mean(dim=1))
                p_anom = torch.sigmoid(logits).mean().item()
            logger.debug(f"SSL head inference ok; p_anom={p_anom:.4f}")
        else:
            logger.debug("SSL head not found yet; p_anom=None")

        # --- (G) Thresholding & fusion ---
        if (
            getattr(self, "lstm_err_min", None) is None
            or getattr(self, "lstm_err_max", None) is None
        ):
            logger.warning("LSTM thresholds missing; computing via sigma_calc_lstm.")
            self._sigma_calc_lstm()
        is_lstm_band_anom = (lstm_err < self.lstm_err_min) or (
            lstm_err > self.lstm_err_max
        )
        head_thresh = float(self.config.get("ssl_head_thresh", 0.6))
        head_ok = p_anom is not None and p_anom > head_thresh

        # Independent anomaly from LSTM pipeline:
        #   we let LSTM count a bug even if DTW didn't, by OR'ing the classifier,
        #   OR requiring (AE band & DTW band) if classifier not ready.
        is_anom_lstm = head_ok or (is_lstm_band_anom and is_dtw_band_anom)

        if is_anom_lstm and not dtw_already_flagged:
            self.fuzzer_stats["potential_crashes"] += 1
            logger.info(
                f"LSTM pipeline flags anomaly at sim at classifier = {head_ok} and {is_lstm_band_anom}!"
            )
            self.fuzzer_stats["bugs_lstm"] = self.fuzzer_stats.get("bugs_lstm", 0) + 1

            self.coverage_class.archive_data(filename="lstm_ssl_anom")
            logger.warning(
                f"[LSTM] anomaly: lstm_err={lstm_err:.6e} p_anom={p_anom} "
                f"dtw={dtw_dist:.6e} z_dtw={z_dtw:.2f}"
            )
        else:
            logger.debug(
                f"[LSTM] normal or DTW already flagged: err={lstm_err:.6e} "
                f"p={p_anom} dtw={dtw_dist:.6e} z={z_dtw:.2f}"
            )

        # recon is a Tensor on device; get numpy in normalized space [B,T,4]
        recon_np = recon.detach().cpu().numpy()

        # figure out current BIN index (like DTW path)
        log_file_path = os.path.join(self.fuzzer_temp_dir, "logs", "LASTLOG.TXT")
        log_idx = None
        if os.path.exists(log_file_path):
            with open(log_file_path, "r") as f:
                log_idx = int(f.read().strip())

        save_lstm_bin_triptych(
            filename_idx=log_idx,
            recon_norm=recon_np,
            mean=mean,  # np.ndarray shape [4]
            std=std,  # np.ndarray shape [4]
            stride=self.lstm_stride,
            output_dir=self.fuzzer_temp_dir,
            title="LSTM Reconstruction vs BIN (raw units)",
            channels=("C1", "C2", "C3", "C4"),
            rc_log_filter=True,
        )

        self.last_scores = {
            "dtw": dtw_dist,
            "z_dtw": z_dtw,
            "lstm_err": float(lstm_err),
            "p_anom": float(p_anom) if p_anom is not None else float("nan"),
        }

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
            self.oracle_dtw()
            self.oracle_lstm()
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

            elif field_type == "param_range_auto" or field_type == "param_range_float":
                min_val = field.get("range_min", -10.0)
                max_val = field.get("range_max", 10.0)
                increment = field.get("increment", 0.1)
                if type(min_val) is not float:
                    field_values[field_name] = gen_int_step(min_val, max_val, increment)
                else:
                    field_values[field_name] = random.uniform(
                        min_val, max_val
                    )  # TODO: Handle float range properly

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
        msg_entry = random.choice(msg_list)
        msg_name = msg_entry[1]
        if msg_name == "PARAM_SET":
            logger.debug("Skipping PARAM_SET message for mutation")
            return None, None, None
        else:
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
            # Check the current state
            self.manage_fuzzer_state()
            # Now figure out what we need to do
            if self.fuzzer_state == FuzzState.Init:
                msg_name, msg_id, field_values = self.init_generate_message()
            elif self.fuzzer_state in [FuzzState.Bitflip, FuzzState.Arithmetic]:
                msg_name, msg_id, field_values = self.mutate_msg()
                if field_values is None:
                    # If we don't have a message to mutate, go back to init state
                    logger.debug(
                        "No message to mutate, falling back to generating a message"
                    )
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
        for field_name in list(field_values):
            # Check if we have a field name that contains "time"
            if "time_boot_ms" in field_name:
                current_time = round(
                    (time.time() - self.fuzzer_stats["current_mission_time"]) * 1000
                )
                # Wrap the value at 2**32 - 1
                if current_time > 2**32 - 1:
                    current_time = current_time % (2**32 - 1)
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
            # 2025-07-25T16:52:48-0400: silipwn: Creates issues when actually using for any other peripheral
            # Proximity sensor have deviations here :|
            if "min_distance" in field_name:
                # Check if the existing value is float, then set it to float(0)
                if type(field_values[field_name]) is float:
                    field_values[field_name] = 0.0
                elif type(field_values[field_name]) is int:
                    field_values[field_name] = 0
            if "max_distance" in field_name:
                if type(field_values[field_name]) is float:
                    field_values[field_name] = 12.0
                elif type(field_values[field_name]) is int:
                    field_values[field_name] = 12
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
            if "thrust_body" in field_name:
                # Remove the field
                field_values.pop(field_name, None)

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

        The method continuously processes all errors in the queue until its empty.
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
                    logger.error(
                        "\nSITL error during calibration phase - this is fatal"
                    )
                    self.cleanup_and_exit()
                    exit(1)
                else:
                    self.cleanup_sim(oracle=False)  # No need to run the oracle again

            if error.get("type") == "fuzzer_error":
                component = error.get("component", "unknown")
                logger.error(
                    f"\nNot recoverable state, fatal error inside {component}, exiting..."
                )
                logger.error(error["error"])
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
            raise e

    def replay_messages(self, messages, start_time=None):
        """
        Replay a list of messages at the correct time offsets.
        Each message: [timestamp, msg_name, msg_id, dict{field_values}]
        """
        if not messages:
            if logger:
                logger.warning("No messages to replay.")
            return

        while not self.tcp_conn.rc_monitor and self.tcp_conn.drone_in_air:
            time.sleep(1)

        # Sort messages by timestamp
        messages = sorted(messages, key=lambda x: x[0])
        base_time = start_time if start_time is not None else time.time()
        first_msg_time = messages[0][0]

        for msg in messages:
            msg_time, msg_name, msg_id, field_values = msg
            # Calculate when to send this message
            send_at = base_time + (msg_time - first_msg_time)
            now = time.time()
            sleep_time = send_at - now
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Send the message using the TCPConn infrastructure
            if hasattr(self, "tcp_conn") and hasattr(self.tcp_conn, "custom_msg_send"):
                self.tcp_conn.custom_msg_send(msg_name, field_values)
                if logger:
                    logger.info(
                        f"Replayed {msg_name} at {msg_time} with fields {field_values}"
                    )
            else:
                if logger:
                    logger.error("TCP connection or msg_send not available.")
                else:
                    print("TCP connection or msg_send not available.")

        return


# Misc utilities and sanity checks
def file_exists(file_o_dir):
    # Handle special case of having null
    if not file_o_dir:
        raise FileNotFoundError(f"File or directory {file_o_dir} does not exist.")
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
            "--src_dir", type=str, help="Source directory", required=False
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
            if not args.src_dir:
                missing_args.append("--src_dir")

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
        cfg = FuzzConfig(args, logger)

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
                        logger.error(
                            "Error encountered during calibration waiting phase"
                        )
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
            cfg.coverage_class.calibration_save()  # Save the current coverage metrics
            cfg.coverage_class.reset()
        else:
            logger.debug("Calibration values already set, skipping calibration")

        # Run the simulation with fuzzing
        # Main loop with progress tracking and stats
        fuzzing_iterations = 0
        pbar = tqdm(desc="Fuzzing Progress")

        # Function to update tqdm with stats
        def update_tqdm_postfix():
            # thresholds (band learned from DTW sigma-calc)
            dtw_min_val = getattr(cfg, "min_fuzz_threshold", None)
            dtw_max_val = getattr(cfg, "max_fuzz_threshold", None)

            # last mission scores (DTW + LSTM)
            s = getattr(cfg, "last_scores", {}) or {}

            def fmt(x, prec):
                try:
                    return f"{float(x):.{prec}f}"
                except Exception:
                    return "NaN"

            pbar.set_postfix(
                {
                    "time": f"{cfg.fuzzer_stats['last_mission_time']:.1f}s",
                    "state": f"{cfg.fuzzer_state}",
                    "sims": cfg.fuzzer_stats["simulations_completed"],
                    "msgs": cfg.fuzzer_stats["messages_sent"],
                    # thresholds (consider renaming to dtw_lo_thr/dtw_hi_thr later)
                    "dtw_min": "NaN" if dtw_min_val is None else f"{dtw_min_val:.6f}",
                    "dtw_max": "NaN" if dtw_max_val is None else f"{dtw_max_val:.6f}",
                    # live scores from last run
                    "dtw": fmt(s.get("dtw", float("nan")), 4),
                    "z_dtw": fmt(s.get("z_dtw", float("nan")), 2),
                    "lstm": fmt(s.get("lstm_err", float("nan")), 5),
                    "p_anom": fmt(s.get("p_anom", float("nan")), 2),
                    "bugs": f"{cfg.fuzzer_stats['potential_crashes']}",
                    "bugs_dtw": cfg.fuzzer_stats.get("bugs_dtw", 0),
                    "bugs_lstm": cfg.fuzzer_stats.get("bugs_lstm", 0),
                }
            )

        while not cfg.fuzzer_shutdown_requested:
            fuzzing_iterations += 1
            logger.info(f"Starting fuzzing iteration {fuzzing_iterations}")
            cfg.run_sim()

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
            cfg.coverage_class.reset()
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
