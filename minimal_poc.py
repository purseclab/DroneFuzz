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
from lxml import etree
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
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
approx_threshold = 0.1  # Threshold for approximate location matching


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

    def monitor_comms(self):
        while self.connected.is_set() and not self.shutdown_requested:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1)
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
                        if self.drone_in_air:
                            self.rcou_queue.put(msg.to_dict())
                    if msg.get_type() == "MISSION_REQUEST":
                        self.mission_msg_queue.put(msg)
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
            if loc["rel_alt"] <= approx_threshold:
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
        while True:
            loc = self.loc_queue.get(timeout=mavlink_timeout)
            if (
                altitude - approx_threshold
                <= loc["rel_alt"]
                <= altitude + approx_threshold
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

            # Replace the include element with the contents of the included file
            parent = include.getparent()
            index = parent.index(include)
            parent.remove(include)
            for child in reversed(list(include_root)):
                parent.insert(index, child)


def load_xml_messages(file_path: str, filter: list) -> list:
    xml_msg = []
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(file_path, parser)
    root = tree.getroot()
    include_xml(root, os.path.dirname(os.path.abspath(file_path)))
    # Find the 'msg' element
    msg_elements = root.xpath("//messages/message")  # Default messages
    msg_elements += root.xpath(
        "//entry"
    )  # Handle the CMD scenarios which exist as enums
    if msg_elements is not None:
        for msg in msg_elements:
            msg_id = msg.get("id") or msg.get("value")
            msg_name = msg.get("name")
            # Check if the message name is inside the filter list
            if msg_name in filter:
                fields = []
                logging.debug("Found the message {}".format(msg_name))
                if msg.xpath(".//field"):
                    for entry in msg.xpath(".//field"):
                        entry_name = entry.get("name")
                        entry_value = entry.get("type")
                        entry_desc = entry.text
                        entry_enum = entry.get("enum", None)
                        enum_vals = None
                        if entry_enum:
                            enum_vals = get_enum(root, entry_enum)
                        field_entry = {
                            "name": entry_name,
                            "type": entry_value,
                            "desc": entry_desc,
                        }
                        if enum_vals:
                            field_entry["enum_vals"] = enum_vals
                        fields.append(field_entry)
                elif msg.xpath(".//param"):
                    for param in msg.xpath(".//param"):
                        param_name = param.get("label")
                        if param_name is None:
                            continue
                        param_min = param.get("minValue", "float")
                        param_max = param.get("maxValue", "float")
                        param_inc = param.get("increment", None)
                        param_text = param.text
                        fields.append(
                            {
                                "name": param_name,
                                "type": [param_min, param_max, param_inc],
                                "desc": param_text,
                            }
                        )
                xml_msg.append(
                    {"msg_id": msg_id, "msg_name": msg_name, "fields": fields}
                )

    else:
        print("No msgs found in the XML file.")

    return xml_msg


def generate_field_value(field_type):
    """Generate a random value for a field based on its type."""
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
        calibration_rounds = (
            args.calibration_rounds
            if args.calibration_rounds
            else self.config.get("calibration_rounds", 10)
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
        self.calibration_vals = None
        self.calibration_rounds = calibration_rounds

        # Fuzzing related attributes
        self.xml_messages = []
        self.rcou_vals = []
        self.golden_rc_vals = []
        self.start_time = time.time()  # Rough estimate only
        self.fuzzing_active = False
        self.fuzzing_thread = None
        self.fuzzer_dtw_threshold = (
            args.dtw_threshold
            if args.dtw_threshold
            else self.config.get("dtw_threshold", 100.00)
        )
        self.min_fuzz_threshold = None
        self.max_fuzz_threshold = None
        self.sim_ready = False
        self.fuzz_interval = self.config.get(
            "fuzz_interval", 0.5
        )  # Send a fuzzed message every 0.5 seconds
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
            logger.warning(
                f"No peripheral mapping found for {self.peripheral_under_test}"
            )
            self.peripheral_config = {}

        # Find the filter from the peripheral mapping
        msg_filter = self.peripheral_config.get("msg_type", [])
        logger.info(f"Using message filter: {msg_filter}")

        # Load XML message definitions if provided
        if self.xml_file and os.path.exists(self.xml_file):
            logger.info(f"Loading MAVLink message definitions from {self.xml_file}")
            self.xml_messages = load_xml_messages(self.xml_file, filter=msg_filter)
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
            msg_freq = selected_freq[msg_idx] if msg else -1
            logger.info(f"Message: {msg}, Frequency: {msg_freq}")
            xml_msg = load_xml_messages(self.xml_file, filter=[msg])
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

    def run_sim(self):
        sitl_args = " -S --model + --speedup 1 -I0"
        self.sitl_cmd = self.sitl_bin + sitl_args + " --defaults " + self.param_file
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
        self.start_fuzzing()
        while self.tcp_conn.drone_in_air:
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
        self.fuzzer_stats["simulations_completed"] += 1
        self.fuzzer_stats["last_mission_time"] = (
            time.time() - self.fuzzer_stats["current_mission_time"]
        )

    def signal_handler(self, _signum, _frame):
        """Handle shutdown signals gracefully"""
        print("\nShutdown requested...")
        print("Will terminate after current execution\n")
        self.shutdown_requested = True

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
        # Get the threshold values for 3 sigma
        mean = np.mean(self.fuzzer_stats["dtw_threshold"])
        std_dev = np.std(self.fuzzer_stats["dtw_threshold"])
        self.min_fuzz_threshold = mean - (3 * std_dev)
        self.max_fuzz_threshold = mean + (3 * std_dev)

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
                    distance, _path = self.calculate_dtw(prev_rcou_vals, self.rcou_vals)
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
            distance, _ = self.calculate_dtw(golden_rc_vals, self.rcou_vals)
            logger.info(
                f"DTW distance calculated: {distance} len: {len(self.golden_rc_vals)}"
            )
            combined_distance += distance
        distance = combined_distance / len(self.golden_rc_vals)
        logger.debug("Final DTW distance calculated: {}".format(distance))
        assert (
            self.min_fuzz_threshold is not None or self.max_fuzz_threshold is not None
        )
        if (distance < self.min_fuzz_threshold) or (distance > self.max_fuzz_threshold):
            logger.info(
                f"DTW distance {distance} exceeds {self.min_fuzz_threshold} or is way below threshold {self.max_fuzz_threshold}, potential anomaly detected! at simulation {self.fuzzer_stats['simulations_completed']}"
            )
            self.fuzzer_stats["potential_crashes"] += 1
            # Save inputs for later analysis
        input_file = tempfile.mkstemp(
            suffix=".txt", prefix="inputs", dir=self.fuzzer_temp_dir
        )[1]
        logger.info(f"Saving inputs to {input_file}")
        with open(input_file, "w") as f:
            # Dump all the values inside the fuzz_msgs
            for msg in self.fuzz_msgs:
                f.write(f"{msg}\n")

    def calculate_dtw(self, series1, series2):
        """Calculate the DTW distance between two time series."""
        # We get a list of dictionaries, so we need to convert them to numpy arrays
        # Assuming series1 and series2 are lists of dictionaries with keys "chan1_raw","chan2_raw", etc.
        fields = ["servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw"]
        s1 = np.array([[pkt[f] for f in fields] for pkt in series1])
        s2 = np.array([[pkt[f] for f in fields] for pkt in series2])
        distance, path = fastdtw(s1, s2, dist=euclidean)
        return distance, path

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
            field_values = []
            for field in msg_def["fields"]:
                """if "frame" in field["name"]:
                    field_values.append(12)
                elif "obstacle_id" in field["name"]:
                    obstacle_id = 65535
                    field_values.append(obstacle_id)
                """  # If we have enum values, pick a random one
                if "enum_vals" in field:
                    field_values.append(int(random.choice(field["enum_vals"])))
                elif "time" in field["name"]:
                    current_time = round(
                        (time.time() - self.fuzzer_stats["current_mission_time"]) * 1000
                    )
                    field_values.append(current_time)
                elif type(field["type"]) is list:
                    min, max, inc = field["type"]
                    field_value = None
                    if (min == "float") & (max == "float") & (inc is None):
                        # Handling a specific case of params
                        field_value = generate_field_value("float")
                        field_values.append(field_value)
                # Else use the generate_field_value function
                else:
                    field_values.append(generate_field_value(field["type"]))
            # If we are in calibration mode, just send the same values over for the fields
            if self.calibration_active:
                if self.calibration_vals is None:
                    self.calibration_vals = field_values
                else:
                    field_values = self.calibration_vals
            # Send the fuzzed message
            try:
                self.send_fuzzed_message(
                    msg_def["msg_name"], msg_def["msg_id"], field_values
                )
                msg_dict = [msg_def["msg_name"], field_values]
                self.fuzz_msgs.append(msg_dict)
                self.fuzzer_stats["messages_sent"] += 1
            except Exception as e:
                logger.error(f"Error sending fuzzed message: {e}")

            time.sleep(1 / self.fuzz_interval)

    def send_fuzzed_message(self, msg_name, msg_id, field_values):
        """Send a fuzzed message using the MAVLink connection."""
        try:
            # Get the message class from mavutil
            msg_class = getattr(mavutil.mavlink, f"MAVLink_{msg_name.lower()}_message")

            # Create the message instance with the fuzzed values
            msg = msg_class(*field_values)

            # Send the message
            self.tcp_conn.conn.mav.send(msg)
        except AttributeError:
            # If the message class doesn't exist, use a more generic approach
            packed_msg = mavutil.mavlink.MAVLink_command_long_message(
                0,  # target_system
                0,  # target_component
                int(msg_id),  # command
                0,  # confirmation
                *field_values,  # parameters
            )
            self.tcp_conn.conn.mav.send(packed_msg)
        except Exception as e:
            logger.error(f"Error sending message {msg_name}: {e}")


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

        # Establish the threshold for the fuzzing runs
        # Run the mission with simulations and default parameters
        logger.info("\nBeginning calibration")
        cfg.calibration_active = True

        # Create tqdm progress bar for calibration
        calib_pbar = tqdm(range(cfg.calibration_rounds), desc="Calibration Progress")

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
            logger.info(f"Starting calibration round {i+1}/{cfg.calibration_rounds}")
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
        # logger.info(
        #     f"The DTW threshold for fuzzing is set to {cfg.fuzzer_stats['dtw_threshold']:.2f}"
        # )
        # cfg.fuzzer_dtw_threshold = 0.15 * cfg.fuzzer_stats["dtw_threshold"]
        # Reset all the stats
        cfg.fuzzer_stats["current_mission_time"] = 0.0
        cfg.fuzzer_stats["simulations_completed"] = 0
        cfg.fuzzer_stats["messages_sent"] = 0
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
