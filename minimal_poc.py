# Basically a singular file to do the fuzzing loop
# Start the SITL binary
# Establish the TCP connection
# Set the guided mission
# Fuzz some messages based on the XML loading
# Check the status after the mission finishes
import argparse
import time
import os
import random
import json
from lxml import etree

# Set the mavlink version to 2
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
import subprocess
from queue import Queue
import threading

# from fastdtw import fastdtw

mavlink_timeout = 5
approx_threshold = 0.1  # Threshold for approximate location matching


class TCPConn:
    def __init__(self):
        # Python inits
        self.connected = True
        self.lock = threading.Lock()
        self.msg_queue = Queue()
        self.loc_queue = Queue()
        self.drone_ready = False  # Drone ready with GPS lock
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
        while self.connected:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,  # Ground Control Station
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            time.sleep(1)  # Sleep for a second before sending the next heartbeat

    def monitor_comms(self):
        while self.connected:
            msg = self.conn.recv_match(blocking=True, timeout=1)
            if msg:
                with self.lock:
                    self.msg_queue.put(msg)
                    if msg.get_type() == "STATUSTEXT":
                        # Crazy check because pymavlink lock doesn't work
                        print(msg.text)
                        if "is using GPS" in msg.text:
                            print("Drone is ready with gps_lock ")
                            self.drone_ready = True
                    if msg.get_type() == "COMMAND_ACK":
                        if msg.result is not mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            print(f"Command failed with result: {msg.result}")
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
        if self.conn:
            self.conn.close()
            print("TCP connection closed.")


def include_xml(elem, base_path, processed_files=None):
    if processed_files is None:
        processed_files = set()

    for include in elem.xpath(".//include"):
        filename = include.text
        filepath = os.path.join(base_path, filename)
        print(f"Processing include: {filepath}")

        # Check if the file has already been processed
        if filepath in processed_files:
            print(f"Skipping already processed file: {filepath}")
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


def load_xml_messages(file_path: str, filter: list = ["OPTICAL_FLOW"]) -> list:
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
                print("Debug: Found the message {}".format(msg_name))
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
    ):
        if file_exists(bin) and file_exists(src_dir):
            print(f"Using SITL binary: {bin}")
            print(f"Using Ardupilot directory: {src_dir}")
        self.sitl_bin = bin
        self.ap_dir = src_dir
        self.vehicle = vehicle
        self.timeout = 1000  # TODO: Eventually figure out how to set this
        self.msg_def = peripheral
        self.param_file = os.path.join(
            self.ap_dir + "Tools/autotest/default_params/" + self.vehicle + ".parm"
        )
        if file_exists(self.param_file):
            print("Using parameter file: " + self.param_file)

        # Fuzzing related attributes
        self.fuzzing_active = False
        self.fuzzing_thread = None
        self.xml_file = xml_file
        self.xml_messages = []
        self.fuzz_interval = 0.5  # Send a fuzzed message every 0.5 seconds

        # Load XML message definitions if provided
        if xml_file and os.path.exists(xml_file):
            print(f"Loading MAVLink message definitions from {xml_file}")
            self.xml_messages = load_xml_messages(xml_file)
            print(f"Loaded {len(self.xml_messages)} message definitions")

        self.setup()

        self.run_sim()
        self.tcp_conn = TCPConn()

    def setup(self):
        # Load the JSON Peripheral mapping
        self.peripheral_mapping = {}

    def run_sim(self):
        sitl_args = " -S --model + --speedup 1 -I0"
        self.sitl_cmd = self.sitl_bin + sitl_args + " --defaults " + self.param_file
        try:
            self.sim_handle = subprocess.Popen(
                ["bash", "-c", self.sitl_cmd],
                # Temporarily commented out to debug
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=False,
                preexec_fn=os.setsid,
            )
        except Exception as e:
            raise Exception(f"Simulation errored with {e}")

    def send_mission(self):
        self.tcp_conn.set_mode("GUIDED")
        self.tcp_conn.arm()
        self.tcp_conn.takeoff(50)

        # Start fuzzing after takeoff
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
        self.stop_fuzzing()

        # Land
        self.tcp_conn.land()
        print("Finished mission")

    def cleanup_sim(self):
        self.tcp_conn.cleanup()
        self.sim_handle.terminate()
        print("Simulation terminated.")

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
                print(f"Sent fuzzed message: {msg_def['msg_name']}")
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
    args = argument_parser.parse_args()

    cfg = FuzzConfig(args.bin, args.ap_dir, xml_file=args.xml_file)

    # Register the signal handler for cleanup
    while not cfg.tcp_conn.drone_ready:
        print("Waiting for drone to be ready with GPS lock...")
        time.sleep(3)
    cfg.send_mission()
    cfg.cleanup_sim()
