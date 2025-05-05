# Basically a singular file to do the fuzzing loop
# Start the SITL binary
# Establish the TCP connection
# Set the guided mission
# Fuzz some messages based on the XML loading
# Check the status after the mission finishes
import argparse
import time
import yaml
import os
import random
import signal
import tempfile
from lxml import etree
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import numpy as np

# Set the mavlink version to 2
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
import subprocess
from queue import Queue
import threading
import copy

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
        self.drone_ready = False  # Drone ready with GPS lock
        self.drone_in_air = False
        # GPS location
        # Connection details
        self.conn = mavutil.mavlink_connection(
            "tcp:localhost:5760", auto_reconnect=True
        )
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
                print(f"Error in send_heartbeat: {e}")
                if not self.shutdown_requested:
                    time.sleep(1)
        print("Connection closed, stopping heartbeat thread.")

    def monitor_comms(self):
        while self.connected.is_set() and not self.shutdown_requested:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1)
                if msg:
                    self.msg_queue.put(msg)
                    if msg.get_type() == "STATUSTEXT":
                        # print(msg.text)
                        # Crazy check because pymavlink lock doesn't work
                        if "is using GPS" in msg.text:
                            print("Drone is ready with gps_lock ")
                            self.drone_ready = True
                    if msg.get_type() == "COMMAND_ACK":
                        if msg.result is not mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            print(f"Command failed with result: {msg.result}")
                            self.shutdown_requested = True
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
            except Exception as e:
                print(f"Error in monitor_comms: {e}")
                self.shutdown_requested = True
                if not self.shutdown_requested:
                    time.sleep(1)  # Wait before retrying
                print("Connection closed, stopping monitor thread.")

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
            print("Error: Invalid mode specified")
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
        print("Setting mode to: " + mode)

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
                print(f"Drone reached an relative altitude: {loc['rel_alt']} meters")
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
                print(f"Drone has taken off to altitude: {loc['rel_alt']} meters")
                self.drone_in_air = True
                break
            time.sleep(0.1)

    def go_to_waypoint(self, lat, lon, alt):
        """Navigate to a specified waypoint."""
        print(f"Navigating to waypoint: lat={lat}, lon={lon}, alt={alt}")
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
                print(f"Reached waypoint: lat={loc['lat']}, lon={loc['lon']}")
                break

    def upload_mission(self, mission_file):
        pass

    def cleanup(self):
        self.shutdown_requested = True
        rcou_list = []
        print(self.msg_queue.qsize(), " messages in the queue")
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
            print("TCP connection closed.")
        if rcou_list:
            print(f"Received {len(rcou_list)} RC channel updates.")
        return rcou_list


def include_xml(elem, base_path, processed_files=None):
    if processed_files is None:
        processed_files = set()

    for include in elem.xpath(".//include"):
        filename = include.text
        filepath = os.path.join(base_path, filename)
        # print(f"Processing include: {filepath}")

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
                # print("Debug: Found the message {}".format(msg_name))
                if msg.xpath(".//field"):
                    for entry in msg.xpath(".//field"):
                        entry_name = entry.get("name")
                        entry_value = entry.get("type")
                        entry_desc = entry.text
                        fields.append(
                            {
                                "name": entry_name,
                                "type": entry_value,
                                "desc": entry_desc,
                            }
                        )
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
    def __init__(
        self,
        bin=None,
        src_dir="/ardupilot",
        vehicle="copter",
        xml_file=None,
        peripheral=None,
        yaml_file=None,
        calibration_rounds=3,
        dtw_threshold=100.00,
    ):
        self.shutdown_requested = False
        # Register signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        if file_exists(bin) and file_exists(src_dir):
            print(f"Using SITL binary: {bin}")
            print(f"Using Ardupilot directory: {src_dir}")
        self.sitl_bin = bin
        self.ap_dir = src_dir
        self.vehicle = vehicle
        self.timeout = 1000  # TODO: Eventually figure out how to set this
        self.peripheral_under_test = peripheral
        self.param_file = os.path.join(
            self.ap_dir + "Tools/autotest/default_params/" + self.vehicle + ".parm"
        )
        if file_exists(self.param_file):
            print("Using parameter file: " + self.param_file)
        self.calibration_active = False
        self.calibration_rounds = calibration_rounds
        # Fuzzing related attributes
        self.fuzzing_active = False
        self.fuzzing_thread = None
        self.fuzzer_dtw_threshold = dtw_threshold
        self.sim_ready = False
        self.xml_file = xml_file
        self.xml_messages = []
        self.rcou_vals = []
        self.fuzz_interval = 0.5  # Send a fuzzed message every 0.5 seconds
        self.setup(yaml_file=yaml_file)

    def periodic_send(self, frequency, xml_msg, default_values):
        print(f"Starting periodic send for {xml_msg} every {frequency} seconds")
        while True:
            if not self.fuzzing_active and self.sim_ready:
                try:
                    # Check if the default values have time
                    # Replace with current time
                    if "time" in default_values:
                        time_idx = default_values.index("time")
                        default_values[time_idx] = int(
                            time.time() - self.fuzzer_stats["current_mission_time"]
                        )
                    self.send_fuzzed_message(
                        xml_msg["msg_name"], xml_msg["msg_id"], default_values
                    )
                except Exception as e:
                    print(f"Error sending message: {e}")
                    self.shutdown_requested = True

                time.sleep(1 / frequency)

    def setup(self, yaml_file=None):
        self.fuzzer_stats = {
            "simulations_completed": 0,
            "messages_sent": 0,
            "last_mission_time": 0.0,
            "current_mission_time": 0.0,
            "dtw_threshold": 0.0,
        }
        # Load the YAML Peripheral mapping
        with open(yaml_file, "r") as f:
            peripheral_mapping = yaml.safe_load(f)
        self.peripheral_mapping = peripheral_mapping["sensors"].get(
            self.peripheral_under_test, {}
        )
        print(f"Using peripheral mapping from {yaml_file}:")
        print(
            f"Peripheral mapping for {self.peripheral_under_test}: {self.peripheral_mapping}"
        )

        # Find the filter from the peripheral mapping
        msg_filter = self.peripheral_mapping.get("msg_type", [])
        print(f"Using message filter: {msg_filter}")

        # Load XML message definitions if provided
        if self.xml_file and os.path.exists(self.xml_file):
            print(f"Loading MAVLink message definitions from {self.xml_file}")
            self.xml_messages = load_xml_messages(self.xml_file, filter=msg_filter)
            print(f"Loaded {len(self.xml_messages)} message definitions")
        # Check if the peripheral mapping has a frequency associated with it
        # If each peripheral has a frequency, spawn a new thread for each peripheral
        self.periodic_thread = {}
        self.default_msg = {}

        # Iterate through each msg_type and check the associated frequency
        selected_msgs = self.peripheral_mapping.get("msg_type", [])
        selected_freq = self.peripheral_mapping["frequency"]
        print("Selected messages for fuzzing:", selected_msgs)
        print("Selected frequencies for fuzzing:", selected_freq)
        for msg in selected_msgs:
            # Get the frequency from the same index
            msg_idx = selected_msgs.index(msg)
            msg_freq = selected_freq[msg_idx] if msg else -1
            print(f"Message: {msg}, Frequency: {msg_freq}")
            xml_msg = load_xml_messages(self.xml_file, filter=[msg])
            # We will get a list of frequencies and then spawn a thread for each peripheral
            if msg_freq != -1:
                try:
                    self.default_msg[msg_idx] = self.peripheral_mapping.get(
                        "default_msg", {}
                    )
                    print(
                        f"Default message for {msg_freq}: {self.default_msg[msg_idx]}"
                    )
                except KeyError:
                    print("Can't find default msg for frequency, Not spawning threads")
                    return

                self.periodic_thread[msg_freq] = threading.Thread(
                    target=self.periodic_send,
                    args=(msg_freq, xml_msg[0], self.default_msg[msg_idx]),
                    daemon=True,
                )
                self.periodic_thread[msg_freq].start()
                print("Periodic thread started")
        # Also if we have a parameter file, create a temporary one and send it to the simulator
        if self.peripheral_mapping.get("parameters"):
            self.fuzzer_param_file = tempfile.mkstemp(".parm", "pgfuzz", "/tmp")[1]
            with open(self.fuzzer_param_file, "w") as f:
                for parameter, values in self.peripheral_mapping["parameters"].items():
                    f.write(f"{parameter} {values}\n")

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
            print(f"Error starting simulation: {e}")

    def send_mission(self, fuzzing=True):
        self.tcp_conn.set_mode("GUIDED")
        self.tcp_conn.arm()
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
        print("Finished mission")
        self.fuzzer_stats["simulations_completed"] += 1
        self.fuzzer_stats["last_mission_time"] = (
            time.time() - self.fuzzer_stats["current_mission_time"]
        )

    def signal_handler(self, _signum, _frame):
        """Handle shutdown signals gracefully"""
        print("Shutdown requested...")
        print("Will terminate after current execution")
        self.shutdown_requested = True

    def cleanup_and_exit(self):
        """Perform cleanup and print summary before exit"""
        print("\nPerforming cleanup...")

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
            print("Simulation terminated.")

        # Print summary
        print("\nFuzzing Session Summary:")
        print("-" * 30)
        print(f"Simulations completed: {self.fuzzer_stats['simulations_completed']}")
        print(f"Messages sent: {self.fuzzer_stats['messages_sent']}")
        print(
            f"Last mission time: {self.fuzzer_stats['last_mission_time']:.2f} seconds"
        )
        print("-" * 30)

    def cleanup_sim(self):
        # Stop fuzzing first
        if self.fuzzing_active:
            self.stop_fuzzing()

        # Stop the periodic threads
        if self.periodic_thread:
            for thread in self.periodic_thread.values():
                thread.join(timeout=2)
            print("Periodic threads stopped")

        # Reset the time
        self.fuzzer_stats["current_mission_time"] = 0.0

        # Then cleanup TCP connection
        if self.tcp_conn:
            if self.rcou_vals:
                prev_rcou_vals = copy.deepcopy(self.rcou_vals)
                self.rcou_vals = self.tcp_conn.cleanup()
                distance, _path = self.calculate_dtw(prev_rcou_vals, self.rcou_vals)
                print("DTW distance calculated: ", distance)
                if self.calibration_active:
                    self.fuzzer_stats["dtw_threshold"] = (
                        self.fuzzer_stats["dtw_threshold"] + distance
                    )
                if self.fuzzing_active:
                    min_fuzz_threshold = (
                        self.fuzzer_stats["dtw_threshold"] + self.fuzzer_dtw_threshold
                    )
                    max_fuzz_threshold = (
                        self.fuzzer_stats["dtw_threshold"] + self.fuzzer_dtw_threshold
                    )
                    if min_fuzz_threshold > distance > max_fuzz_threshold:
                        print(
                            f"DTW distance {distance} exceeds threshold {self.fuzzer_stats['dtw_threshold']}, potential anomaly detected!"
                        )
            else:
                self.rcou_vals = self.tcp_conn.cleanup()

        # Finally terminate the simulation
        if hasattr(self, "sim_handle") and self.sim_handle:
            self.sim_handle.terminate()
            print("Simulation terminated.")
        time.sleep(1)  # Give some time for the threads to finish

        # TODO Check if we actually have a SITL binary running

    def summary(self):
        print(self.fuzzer_stats)

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
            print("No XML message definitions loaded. Cannot start fuzzing.")
            return

        self.fuzzing_active = True
        self.fuzzing_thread = threading.Thread(target=self.fuzz_loop, daemon=True)
        self.fuzzing_thread.start()
        print("Fuzzing thread started")

    def stop_fuzzing(self):
        """Stop the fuzzing thread."""
        self.fuzzing_active = False
        if self.fuzzing_thread:
            self.fuzzing_thread.join(timeout=2)
            print("Fuzzing thread stopped")

    def fuzz_loop(self):
        """Main fuzzing loop that runs in a separate thread."""
        while self.fuzzing_active:
            # Generate random values for each field
            msg_def = random.choice(self.xml_messages)
            field_values = []
            for field in msg_def["fields"]:
                field_values.append(generate_field_value(field["type"]))

            # Send the fuzzed message
            try:
                self.send_fuzzed_message(
                    msg_def["msg_name"], msg_def["msg_id"], field_values
                )
                self.fuzzer_stats["messages_sent"] += 1
            except Exception as e:
                print(f"Error sending fuzzed message: {e}")

            time.sleep(self.fuzz_interval)

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
            print(f"Message class for {msg_name} not found, using raw send")
            self.tcp_conn.conn.mav.send_raw_mavlink(msg_id, *field_values)
        except Exception as e:
            print(f"Error sending message {msg_name}: {e}")


# Misc utilities and sanity checks
def file_exists(file_o_dir):
    if os.path.exists(file_o_dir):
        return True
    else:
        raise FileNotFoundError(f"File or directory {file_o_dir} does not exist.")


if __name__ == "__main__":
    cfg = None
    try:
        argument_parser = argparse.ArgumentParser()
        argument_parser.add_argument(
            "--bin", type=str, help="Path to the SITL binary", required=True
        )
        argument_parser.add_argument(
            "--peripheral", type=str, help="Peripheral to fuzz", required=True
        )
        argument_parser.add_argument(
            "--ap_dir", type=str, help="Ardupilot directory", required=True
        )
        argument_parser.add_argument(
            "--xml_file",
            type=str,
            help="Path to MAVLink XML definition file",
            required=True,
        )
        argument_parser.add_argument(
            "--yaml", type=str, help="YAML file for peripheral mapping", required=True
        )
        args = argument_parser.parse_args()

        # Sanity check for all files
        for arg in [args.bin, args.ap_dir, args.xml_file, args.yaml]:
            file_exists(arg)

        cfg = FuzzConfig(
            bin=args.bin,
            src_dir=args.ap_dir,
            xml_file=args.xml_file,
            peripheral=args.peripheral,
            yaml_file=args.yaml,
        )

        # Establish the threshold for the fuzzing runs
        # Run the mission with simulations and default parameters
        cfg.calibration_active = True
        for _ in range(0, cfg.calibration_rounds):
            cfg.run_sim()
            while not cfg.tcp_conn.drone_ready and not cfg.shutdown_requested:
                print("Waiting for drone to be ready with GPS lock...")
                time.sleep(3)
            if not cfg.shutdown_requested:
                cfg.send_mission(fuzzing=False)
            cfg.cleanup_sim()
        cfg.fuzzer_stats["dtw_threshold"] = (
            cfg.fuzzer_stats["dtw_threshold"] / cfg.calibration_rounds
        )
        cfg.calibration_active = False
        print(
            f"The DTW threshold for fuzzing is set to {cfg.fuzzer_stats['dtw_threshold']:.2f}"
        )
        # Reset all the stats
        cfg.fuzzer_stats["current_mission_time"] = 0.0
        cfg.fuzzer_stats["simulations_completed"] = 0
        cfg.fuzzer_stats["messages_sent"] = 0
        # Run the simulation with fuzzing
        # Main loop
        while not cfg.shutdown_requested:
            # try:
            cfg.run_sim()
            while not cfg.tcp_conn.drone_ready and not cfg.shutdown_requested:
                print("Waiting for drone to be ready with GPS lock...")
                time.sleep(3)
            if not cfg.shutdown_requested:
                cfg.send_mission()
            cfg.cleanup_sim()
            # except Exception as e:
            #     print(f"Error in main loop: {e}")
            #     if not cfg.shutdown_requested:
            #         print("Attempting to restart simulation...")
            #         time.sleep(5)  # Wait before retrying
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        # Ensure cleanup happens even if there's an unhandled exception
        if cfg:
            cfg.cleanup_and_exit()
