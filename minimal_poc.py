# Basically a singular file to do the fuzzing loop
# Start the SITL binary
# Establish the TCP connection
# Set the guided mission
# Fuzz some messages based on the XML loading
# Check the status after the mission finishes
import argparse
import time
import re
import yaml
import os
import random
import signal
import tempfile
import logging
import datetime
import pickle
from lxml import etree
from dtw import dtw
from sklearn.preprocessing import StandardScaler
from contextlib import redirect_stdout
import numpy as np
from tqdm import tqdm

# Set the mavlink version to 2
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil, mavwp
import subprocess
from queue import Queue
import threading
import copy


# Setup logging
def setup_logging():
    """Setup logging with timestamp in filename"""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"pgfuzz_{timestamp}.log"

    # Create logger
    logger = logging.getLogger("pgfuzz")
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
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
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


class TCPConn:
    def __init__(self):
        # Python inits
        self.shutdown_requested = False
        self.connected = threading.Event()
        self.connected.set()
        self.msg_queue = Queue()
        self.loc_queue = Queue()
        self.rcou_queue = Queue()
        self.mission_msg_queue = Queue()
        self.drone_ready = False  # Drone ready with GPS lock
        self.drone_in_air = False
        self.rc_monitor = False  # Flag to monitor RC channel (ideally we want only after takeoff/and before landing)
        self.drone_state = mavutil.mavlink.MAV_STATE_UNINIT  # Initial state
        # GPS location
        # Connection details
        with open(os.devnull, "w") as fnull:
            with redirect_stdout(fnull):
                self.conn = mavutil.mavlink_connection("tcp:localhost:5760")
        self.conn.wait_heartbeat()
        self.internal_error = False
        self.location_waiting = threading.Condition()
        # Start a thread to keep sending heartbeats
        threading.Thread(target=self.send_heartbeat, daemon=True).start()
        # Start a thread to monitor communications
        threading.Thread(target=self.monitor_comms, daemon=True).start()
        self.setup_streams()

    def setup_streams(self):
        self.conn.mav.request_data_stream_send(
            self.conn.target_system,  # target system
            self.conn.target_component,  # target component
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  # Stream ID
            4,  # Rate in Hz
            1,  # Start/Stop (1=start, 0=stop)
        )

    # TODO: Maybe make this modular
    def apply_throttle(self, throttle_pwm=1500, duration=1.0):
        end_time = time.time() + duration
        while time.time() < end_time:
            self.conn.mav.rc_channels_override_send(
                self.conn.target_system,
                self.conn.target_component,
                0,  # chan1
                0,  # chan2
                throttle_pwm,  # chan3
                0,  # chan4
                0,  # chan5
                0,  # chan6
                0,  # chan7
                0,  # chan8
            )
            time.sleep(0.1)
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system, self.conn.target_component, 0, 0, 0, 0, 0, 0, 0, 0
        )

    def send_heartbeat(self):
        while self.connected.is_set() and not self.shutdown_requested:
            try:
                self.conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,  # Ground Control Station
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                time.sleep(1)  # Sleep for a second before sending the next heartbeat
            except Exception as e:
                logger.error(f"Error in send_heartbeat: {e}")
                if not self.shutdown_requested:
                    time.sleep(1)
        logger.info("Connection closed, stopping heartbeat thread.")

    def _monitor_flags(self, msg):
        if "is using GPS" in msg.text:
            logger.info("Drone is ready with gps_lock")
            self.drone_ready = True
        if re.search(r"disarm\w*", msg.text, re.IGNORECASE):
            logger.info("Mission ended, vehicle is disarmed.")
            self.drone_in_air = False
        if re.search(r"takeoff\w*", msg.text, re.IGNORECASE):
            logger.info("AUTO Mission started, takeoff.")
            self.drone_in_air = True
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
            self.drone_in_air = False
            self.rc_monitor = False
            self.drone_ready = False
            self.st_msg_send("STOP RC")

    def monitor_comms(self):
        while self.connected.is_set() and not self.shutdown_requested:
            try:
                msg = self.conn.recv_match(blocking=True)
                if msg:
                    self.msg_queue.put(msg)
                    if msg.get_type() == "STATUSTEXT":
                        logger.debug(msg.text)
                        # Crazy check because pymavlink lock doesn't work
                        self._monitor_flags(msg)
                    if msg.get_type() == "COMMAND_ACK":
                        if msg.result is not mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            # Only create an error if the command was a arming/land/takeoff/auto
                            # Upload mission for rest of the commands just warn
                            if msg.command in [
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                mavutil.mavlink.MAV_CMD_NAV_LAND,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                mavutil.mavlink.MAV_CMD_MISSION_START,
                            ]:
                                logger.error(
                                    f"Command failed: {msg.command} with result: {msg.result}"
                                )
                                self.internal_error = True
                                self.shutdown_requested = True
                            logger.warning(
                                f"Command failed: {msg.command} with result: {msg.result}"
                            )
                    if msg.get_type() == "GLOBAL_POSITION_INT":
                        # Update the drone's GPS location state
                        drone_loc_state = {}
                        drone_loc_state["lat"] = msg.lat / 1e7  # Convert to degrees
                        drone_loc_state["lon"] = msg.lon / 1e7  # Convert to degrees
                        drone_loc_state["alt"] = msg.alt / 1e3  # Convert to meters
                        drone_loc_state["rel_alt"] = (
                            msg.relative_alt / 1e3
                        )  # Convert to meters
                        self.loc_queue.put(drone_loc_state)
                    if (
                        msg.get_type() == "SERVO_OUTPUT_RAW"
                    ):  # Only when drone is in air
                        if self.rc_monitor:
                            self.rcou_queue.put(msg.to_dict())
                    if msg.get_type() == "MISSION_REQUEST":
                        self.mission_msg_queue.put(msg)
                    if msg.get_type() == "HEARTBEAT":
                        self.drone_state = msg.system_status
            except Exception as e:
                logger.error(f"Error in monitor_comms: {e}")
                self.shutdown_requested = True
                if not self.shutdown_requested:
                    time.sleep(1)  # Wait before retrying
                logger.info("Connection closed, stopping monitor thread.")

    def msg_recv(self, msg_type, timeout=mavlink_timeout):
        return self.conn.recv_match(type=msg_type, timeout=timeout, blocking=True)

    def msg_send(self):
        msg = mavutil.mavlink.MAVLink_statustext_message()
        return self.conn.mav.send(msg)

    def st_msg_send(self, text):
        msg = self.conn.mav.statustext_encode(
            mavutil.mavlink.MAV_SEVERITY_INFO, text.encode()
        )
        self.conn.mav.send(msg)

    def set_mode(self, mode):
        # Check if the mode exists in the vehicle mapping
        mode_mapping = self.conn.mode_mapping()
        set_mode = mode_mapping.get(mode, None)
        if not set_mode:
            # TODO Figure out how to properly tear down everything
            logger.error("Error: Invalid mode specified")
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,  # Base mode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            set_mode,
            0,
            0,
            0,
            0,
            0,
        )
        logger.info("Setting mode to: " + mode)

    def land(self):
        """Land the vehicle."""
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        while True:
            loc = self.loc_queue.get(timeout=mavlink_timeout)
            if loc["rel_alt"] <= altitude_threshold:
                logger.info(
                    f"Drone reached a relative altitude: {loc['rel_alt']} meters"
                )
                self.drone_in_air = False
                break
            time.sleep(0.1)

    def arm(self):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1 to arm
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def takeoff(self, altitude):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude,
        )
        # Check if the drone state is within the altitude range
        logger.debug("Waiting for location to be within the altitude range")
        while True:
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
            self.conn.target_system,
            self.conn.target_component,
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
        )
        while True:
            loc = self.loc_queue.get(timeout=mavlink_timeout)
            if (lat - approx_threshold <= loc["lat"] <= lat + approx_threshold) and (
                lon - approx_threshold <= loc["lon"] <= lon + approx_threshold
            ):
                logger.info(f"Reached waypoint: lat={loc['lat']}, lon={loc['lon']}")
                break

    def cleanup(self):
        self.shutdown_requested = True
        rcou_list = []
        logger.info(f"{self.msg_queue.qsize()} messages in the queue")
        while not self.rcou_queue.empty():
            rcou_list.append(self.rcou_queue.get())
        self.connected.clear()
        time.sleep(0.5)  # Give some time for the threads to finish
        # Clear all the queues
        while not self.msg_queue.empty():
            self.msg_queue.get()
        while not self.loc_queue.empty():
            self.loc_queue.get()
        if self.conn:
            self.conn.close()
            logger.info("TCP connection closed.")
        if rcou_list:
            logger.info(f"Received {len(rcou_list)} RC channel updates.")
        return rcou_list


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
                enum_name = child.get("enum")
                entry = {"name": name, "type": typ, "desc": desc}

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
                mn = param.get("minValue", "float")
                mx = param.get("maxValue", "float")
                inc = param.get("increment")
                desc = (param.text or "").strip()
                fields.append(
                    {
                        "name": label,
                        "type": [mn, mx, inc],
                        "desc": desc,
                    }
                )

        messages.append({"msg_id": msg_id, "msg_name": msg_name, "fields": fields})

    return messages


def generate_field_value(field_type):
    """Generate a random value for a field based on its type."""
    # TODO Figure out how to handle arrays
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
            generated_value.append(generate_field_value(field_type))
        return generated_value
    elif field_type.startswith("uint8"):
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
        return random.uniform(-100, 100)
    elif field_type.startswith("char"):
        return random.randint(0, 255)
    else:
        return 0


class FuzzConfig:
    def __init__(self, args):
        # Register signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self.shutdown_requested = False

        # Load configuration from config YAML file first
        self.config_file = args.config if args.config else None
        self.config = {}
        if self.config_file and os.path.exists(self.config_file):
            with open(self.config_file, "r") as f:
                self.config = yaml.safe_load(f)
                logger.info(f"Loaded configuration from {self.config_file}")
        # Just check if the file contains atleast sitl_bin and ap_dir
        if (
            not self.config.get("sitl_bin")
            or not self.config.get("ap_dir")
            or not self.config.get("peripheral_file")
        ):
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

        # Setup files - command line args override yaml config
        self.sitl_bin = args.bin if args.bin else self.config.get("sitl_bin")
        self.ap_dir = (
            args.ap_dir if args.ap_dir else self.config.get("ap_dir", "/ardupilot")
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
        self.vehicle = (
            args.vehicle if args.vehicle else self.config.get("vehicle", "copter")
        )

        # Mission control
        self.timeout = self.config.get("timeout", 1000)
        # MAVLink check for https://mavlink.io/en/guide/routing.html
        self.target_system = self.config.get("target_system", 255)
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
        self.param_file = os.path.join(
            self.ap_dir, "Tools/autotest/default_params/", f"{self.vehicle}.parm"
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
        self.setup()
        if self.msg_freq:
            logger.info("Setting the fuzzing interval to match message frequency")
            self.fuzz_interval = self.msg_freq

    def periodic_send(self, frequency, xml_msg, default_values):
        logger.info(f"Starting periodic send for {xml_msg} every {frequency} seconds")
        while True:
            if not self.fuzzing_active and self.sim_ready:
                try:
                    # Check if the default values have time
                    # Replace with current time
                    if "time" in default_values:
                        time_idx = default_values.index("time")
                        default_values[time_idx] = round(
                            (time.time() - self.fuzzer_stats["current_mission_time"])
                            * 1000
                        )
                    self.send_fuzzed_message(
                        xml_msg["msg_name"], xml_msg["msg_id"], default_values
                    )
                except Exception as e:
                    logger.error(f"Error sending message: {e}")
                    self.shutdown_requested = True

                time.sleep(1 / frequency)

    def setup(self):
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

        # Find the filter from the peripheral mapping
        msg_filter = self.peripheral_config.get("msg_type", [])
        logger.info(f"Using message filter: {msg_filter}")

        # Check if we have CMD messages enabled
        if self.peripheral_config.get("cmd_msgs"):
            # Add the command messages to the filter
            msg_filter += self.cmd_params()

        # Load XML message definitions if provided
        if self.xml_file and os.path.exists(self.xml_file):
            logger.info(f"Loading MAVLink message definitions from {self.xml_file}")
            self.xml_messages = load_xml_messages(self.xml_file, filter_list=msg_filter)
            logger.info(f"Loaded {len(self.xml_messages)} message definitions")
        # Check if the peripheral mapping has a frequency associated with it
        # If each peripheral has a frequency, spawn a new thread for each peripheral
        self.periodic_thread = {}
        self.default_msg = {}

        # Iterate through each msg_type and check the associated frequency
        selected_msgs = self.peripheral_config.get("msg_type", [])
        selected_freq = self.peripheral_config.get("frequency", [])
        logger.info(f"Selected messages for fuzzing: {selected_msgs}")
        logger.info(f"Selected frequencies for fuzzing: {selected_freq}")
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
            self.fuzzer_param_file = tempfile.mkstemp(".parm", "pgfuzz", "/tmp")[1]
            with open(self.fuzzer_param_file, "w") as f:
                for parameter, values in self.peripheral_config["parameters"].items():
                    f.write(f"{parameter} {values}\n")
        # Create a temporary folder for the fuzzed messages locally in the same directory that we are running
        self.fuzzer_temp_dir = tempfile.mkdtemp("pgfuzz", "fuzzing_data", os.getcwd())
        logger.info(
            f"Creating temporary directory for fuzzed messages: {self.fuzzer_temp_dir}"
        )

    def sim_params(self):
        """Add the SIM parameters from the PGFUZZ database"""
        # TODO
        pass

    def cmd_params(self):
        """Add the mission parameters from the PGFUZZ database"""
        cmds = ["MAV_CMD_DO_SET_MODE"]
        return cmds

    def run_sim(self):
        sitl_args = ""
        if self.vehicle == "copter":
            sitl_args = " -S --model + --speedup 1 -I0"
        elif self.vehicle == "plane":
            sitl_args = " -S --model plane --speedup 1 -I0"
        self.sitl_cmd = self.sitl_bin + sitl_args + " --defaults " + self.param_file
        logger.info(f"Starting SITL with command: {self.sitl_cmd}")
        if self.calibration_active:
            assert (
                self.fuzzing_active is False
            ), "Cannot run calibration while fuzzing is active"
        if self.fuzzer_param_file:
            self.sitl_cmd += "," + self.fuzzer_param_file
        try:
            self.sim_handle = subprocess.Popen(
                ["bash", "-c", self.sitl_cmd],
                # Temporarily commented out to debug
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=False,
                preexec_fn=os.setsid,
            )
            self.fuzzer_stats["current_mission_time"] = time.time()
            self.tcp_conn = TCPConn()
            self.sim_ready = True
        except Exception as e:
            # this would just kill the entire script, so need to handle it gracefully
            logger.error(f"Error starting simulation: {e}")

    def monitor_auto_mission(self):
        # Wait till the drone is in air
        logger.info("Waiting till drone is in air")
        random_modes = ["AVOID_ADSB", "LOITER"]  # Can be patched for specific testing
        while not self.tcp_conn.rc_monitor:
            time.sleep(1)
        self.start_fuzzing()
        mode_ctr = 0
        while self.tcp_conn.rc_monitor:
            mode = random.choice(random_modes)
            if (
                mode_ctr < 3 and self.tcp_conn.rc_monitor
            ):  # 2025-05-26T15:41:06-0400: silipwn: To ensure we only change modes couple of times
                self.tcp_conn.set_mode(mode)
                logger.debug(f"Changing mode to: {mode}")
                time.sleep(5)
                self.tcp_conn.set_mode("AUTO")
                logger.debug("Reset mode to AUTO")
                mode_ctr += 1
            else:
                time.sleep(3)
        self.stop_fuzzing()

    def upload_auto_mission(self, mission_file):
        """
        Upload a mission from a waypoint file using MAVProxy's waypoint module

        Args:
            mission_file: Path to the mission file (.waypoints format)
        """
        waypoints = mavwp.MAVWPLoader()
        _ = waypoints.load(mission_file.strip('"'))

        # Clear any existing mission
        self.tcp_conn.conn.waypoint_clear_all_send()

        # Send waypoint count
        self.tcp_conn.conn.waypoint_count_send(waypoints.count())

        # Respond to mission requests
        for _ in range(waypoints.count()):
            try:
                # Wait for mission request message
                msg = self.tcp_conn.mission_msg_queue.get(timeout=4)

                logger.info(f"Received MISSION_REQUEST for sequence {msg.seq}")

                # Send the requested waypoint
                self.tcp_conn.conn.mav.send(waypoints.wp(msg.seq))
                logger.info(f"Sending waypoint {msg.seq}")

            except Exception as e:
                logger.error(f"Error in mission upload: {e}")

    def standard_guided(self, fuzzing=True):
        self.tcp_conn.set_mode("GUIDED")
        self.tcp_conn.arm()
        # Monitor
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
        self.tcp_conn.set_mode("LOITER")
        time.sleep(10)  # This becomes a blocking sleep, so we get stuck here :|
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
        self.tcp_conn.land()

    def send_mission(self, fuzzing=True):
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
        """Handle shutdown signals gracefully"""
        logger.error("\nShutdown requested...")
        logger.error("Will terminate after current execution\n")
        self.shutdown_requested = True
        # Figure out a better way to terminate things and exit quickly

    def cleanup_and_exit(self):
        """Perform cleanup and print summary before exit"""
        logger.info("\nPerforming cleanup...")

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

    def sigma_calc(self):
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
        # Save the calibration values for faster reload next time
        try:
            with open(self.config_file, "r+") as f:
                config = yaml.safe_load(f)
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
        # Save the RC values as pickle file to later use in the current directory
        pickle_file = os.path.join(os.getcwd(), "rcou_vals.pkl")
        with open(pickle_file, "wb") as f:
            f.write(pickle.dumps(self.golden_rc_vals))

    def cleanup_sim(self):
        # Stop fuzzing first
        if self.fuzzing_active:
            self.stop_fuzzing()

        # Stop the periodic threads
        if self.periodic_thread:
            for thread in self.periodic_thread.values():
                thread.join(timeout=2)
            logger.info("Periodic threads stopped")

        # Reset the time
        self.fuzzer_stats["current_mission_time"] = 0.0

        # Then cleanup TCP connection
        if self.tcp_conn:
            if self.shutdown_requested:
                self.tcp_conn.cleanup()
            elif self.rcou_vals:
                prev_rcou_vals = copy.deepcopy(self.rcou_vals)
                self.rcou_vals = self.tcp_conn.cleanup()
                if self.calibration_active:
                    _, distance = self.calculate_dtw(prev_rcou_vals, self.rcou_vals)
                    logger.info(f"DTW distance calculated: {distance}")
                    self.fuzzer_stats["dtw_threshold"].append(distance)
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
        """Return a formatted string with current fuzzing stats"""
        return (
            f"Sims: {self.fuzzer_stats['simulations_completed']} | "
            f"Msgs: {self.fuzzer_stats['messages_sent']} | "
            f"Last time: {self.fuzzer_stats['last_mission_time']:.2f}s | "
            # f"DTW threshold: {self.fuzzer_stats['dtw_threshold']:.2f}"
        )

    def oracle(self):
        combined_distance = 0.0
        for golden_rc_vals in self.golden_rc_vals:
            _, distance = self.calculate_dtw(golden_rc_vals, self.rcou_vals)
            logger.info(
                f"DTW distance calculated: {distance} len: {len(self.golden_rc_vals)}"
            )
            combined_distance += distance
        distance = combined_distance / len(self.golden_rc_vals)
        assert (
            self.min_fuzz_threshold is not None or self.max_fuzz_threshold is not None
        )
        if (distance < self.min_fuzz_threshold) or (distance > self.max_fuzz_threshold):
            logger.info(
                f"DTW distance {distance} exceeds {self.min_fuzz_threshold} or is way below threshold {self.max_fuzz_threshold}, potential anomaly detected! at simulation {self.fuzzer_stats['simulations_completed']}"
            )
            # Try to open the LASTLOG.TXT
            log_file_path = os.path.join(os.getcwd(), "logs/LASTLOG.TXT")
            log_content = "N/A"
            try:
                with open(log_file_path, "r") as log_file:
                    log_content = int(log_file.read().strip())
            except FileNotFoundError:
                logger.warning("Can't find LASTLOG.TXT file")
            logger.debug(f"Please refer to the {log_content:08d}.BIN for more details")
            self.fuzzer_stats["potential_crashes"] += 1
            # Save inputs for later analysis
            input_file = tempfile.mkstemp(
                suffix=".txt", prefix="inputs-anomalous-", dir=self.fuzzer_temp_dir
            )[1]
        else:
            input_file = tempfile.mkstemp(
                suffix=".txt", prefix="inputs", dir=self.fuzzer_temp_dir
            )[1]
        logger.info(f"Saving inputs to {input_file}")
        with open(input_file, "w") as f:
            # Dump all the values inside the fuzz_msgs
            for msg in self.fuzz_msgs:
                f.write(f"{msg}\n")
        # Clean up the fuzz_msgs
        self.fuzz_msgs = []

    def calculate_dtw(self, series1, series2):
        """Calculate the DTW distance between two time series."""
        # We get a list of dictionaries, so we need to convert them to numpy arrays
        # Assuming series1 and series2 are lists of dictionaries with keys "chan1_raw","chan2_raw", etc.
        fields = ["servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw"]
        s1 = np.array([[pkt[f] for f in fields] for pkt in series1])
        s2 = np.array([[pkt[f] for f in fields] for pkt in series2])
        # distance, path = fastdtw(s1, s2, dist=euclidean)
        data_standardized_all = StandardScaler().fit_transform(s1)
        test_data_standardized = StandardScaler().fit_transform(s2)
        # Compute DTW with Euclidean distance
        alignments = dtw(
            data_standardized_all,
            test_data_standardized,
            dist_method="euclidean",
            distance_only=True,
        )
        return alignments.distance, alignments.normalizedDistance

    def start_fuzzing(self):
        """Start the fuzzing thread."""
        if not self.xml_messages:
            logger.warning("No XML message definitions loaded. Cannot start fuzzing.")
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

    def fuzz_loop(self):
        """Main fuzzing loop that runs in a separate thread."""
        while self.fuzzing_active:
            # Generate random values for each field
            msg_def = random.choice(self.xml_messages)
            field_values = {}
            for field in msg_def["fields"]:
                field_name = field["name"]
                if "enum_vals" in field:
                    field_values[field_name] = int(random.choice(field["enum_vals"]))
                elif "time" in field_name:
                    current_time = round(
                        (time.time() - self.fuzzer_stats["current_mission_time"]) * 1000
                    )
                    field_values[field_name] = current_time
                elif type(field["type"]) is list:
                    min, max, inc = field["type"]
                    field_value = None
                    if (min == "float") & (max == "float") & (inc is None):
                        # Handling a specific case of params
                        field_value = generate_field_value("float")
                        field_values[field_name] = field_value
                else:
                    field_values[field_name] = generate_field_value(field["type"])
            # If we are in calibration mode, just send the same values over for the fields
            msg_dict = [msg_def["msg_name"], field_values]
            if self.calibration_active:
                if self.calibration_vals is None:
                    self.calibration_vals = field_values
                    self.fuzz_msgs.append(msg_dict)
                    logger.debug(f"The message for calibration is {msg_dict}")
                else:
                    field_values = self.calibration_vals
            # Send the fuzzed message
            self.send_fuzzed_message(
                msg_def["msg_name"], msg_def["msg_id"], field_values
            )
            # To ensure we only save fuzzed message
            if not self.calibration_active:
                self.fuzz_msgs.append(msg_dict)
            self.fuzzer_stats["messages_sent"] += 1

            time.sleep(1 / self.fuzz_interval)

    def send_fuzzed_message(self, msg_name, msg_id, field_values):
        """Send a fuzzed message using the MAVLink connection."""
        try:
            # Get the message class from mavutil
            msg_class = getattr(mavutil.mavlink, f"MAVLink_{msg_name.lower()}_message")

            msg = msg_class(**field_values)
            # Create the message instance with the fuzzed values

            # Send the message
            self.tcp_conn.conn.mav.send(msg)
        except AttributeError:
            # If the field_values are not correct in length (7), we add the message with 0s
            if len(field_values) < 7:
                field_values += [0] * (7 - len(field_values))
            packed_msg = mavutil.mavlink.MAVLink_command_long_message(
                self.target_system,  # target_system
                self.target_component,  # target_component
                int(msg_id),  # command
                0,  # confirmation
                **field_values,  # parameters
            )
            self.tcp_conn.conn.mav.send(packed_msg)
        except Exception as e:
            logger.error(
                f"Error sending message {msg_name} with ID {msg_id} and values {field_values}: {e}"
            )


# Misc utilities and sanity checks
def file_exists(file_o_dir):
    if os.path.exists(file_o_dir):
        return True
    else:
        logger.error(f"File or directory {file_o_dir} does not exist.")
        raise FileNotFoundError(f"File or directory {file_o_dir} does not exist.")


if __name__ == "__main__":
    cfg = None
    try:
        argument_parser = argparse.ArgumentParser(
            description="PGFUZZ++ - A fuzzer for ArduPilot peripherals"
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

        logger = setup_logging()
        cfg = FuzzConfig(args)

        if not cfg.calibration_threshold:
            # Establish the threshold for the fuzzing runs
            # Run the mission with simulations and default parameters
            # Check if we already have calibration values
            logger.info("\nBeginning calibration")
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
                while not cfg.tcp_conn.drone_ready and not cfg.shutdown_requested:
                    time.sleep(1)
                    calib_pbar.refresh()  # Keep progress bar visible during waiting

                if cfg.shutdown_requested:
                    cfg.cleanup_and_exit()
                    exit(0)
                else:
                    cfg.send_mission()

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
                    "sims": cfg.fuzzer_stats["simulations_completed"],
                    "msgs": cfg.fuzzer_stats["messages_sent"],
                    "time": f"{cfg.fuzzer_stats['last_mission_time']:.1f}s",
                    # "dtw_avg": f"{cfg.fuzzer_stats['dtw_threshold']:.1f}",  # dtw_threshold is now an average
                    "dtw_min": f"{cfg.min_fuzz_threshold:.1f}",
                    "dtw_max": f"{cfg.max_fuzz_threshold:.1f}",
                    "bugs": f"{cfg.fuzzer_stats['potential_crashes']}",
                }
            )
            # Example of printing an important message during fuzzing
            # if cfg.fuzzer_stats['potential_crashes'] > 0:
            #     tqdm.write(f"Potential crash detected! Count: {cfg.fuzzer_stats['potential_crashes']}")

        while not cfg.shutdown_requested:
            fuzzing_iterations += 1
            logger.info(f"Starting fuzzing iteration {fuzzing_iterations}")
            tqdm.write(f"Fuzzing Iteration: {fuzzing_iterations}")
            cfg.run_sim()

            tqdm.write("Waiting for drone GPS lock...")
            while not cfg.tcp_conn.drone_ready and not cfg.shutdown_requested:
                time.sleep(1)
                pbar.refresh()  # Keep progress bar visible during waiting

            if not cfg.shutdown_requested:
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
