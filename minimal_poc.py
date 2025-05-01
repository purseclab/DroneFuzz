# Basically a singular file to do the fuzzing loop
# Start the SITL binary
# Establish the TCP connection
# Set the guided mission
# Fuzz some messages based on the XML loading
# Check the status after the mission finishes
import argparse
import time
import os

# Set the mavlink version to 2
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
import subprocess
from queue import Queue
import threading

# from fastdtw import fastdtw

mavlink_timeout = 5


class TCPConn:
    def __init__(self):
        print(os.environ["MAVLINK20"])
        self.conn = mavutil.mavlink_connection(
            "tcp:localhost:5760", auto_reconnect=True
        )
        self.connected = True
        self.lock = threading.Lock()
        self.msg_queue = Queue()
        self.drone_ready = False  # Drone ready with GPS lock
        self.conn.wait_heartbeat()
        self.internal_error = False
        # Start a thread to keep sending heartbeats
        threading.Thread(target=self.send_heartbeat, daemon=True).start()
        # Start a thread to monitor communications
        threading.Thread(target=self.monitor_comms, daemon=True).start()

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

    def upload_mission(self, mission_file):
        pass

    def cleanup(self):
        if self.conn:
            self.conn.close()
            print("TCP connection closed.")


class FuzzConfig:
    def __init__(self, sitl_bin, ap_dir, vehicle="copter"):
        if file_exists(sitl_bin) and file_exists(ap_dir):
            print(f"Using SITL binary: {sitl_bin}")
            print(f"Using Ardupilot directory: {ap_dir}")
        self.sitl_bin = sitl_bin
        self.ap_dir = ap_dir
        self.vehicle = vehicle
        self.timeout = 1000  # TODO: Eventually figure out how to set this
        self.param_file = os.path.join(
            self.ap_dir + "Tools/autotest/default_params/" + self.vehicle + ".parm"
        )
        if file_exists(self.param_file):
            print("Using parameter file: " + self.param_file)
        self.run_sim()
        self.tcp_conn = TCPConn()

    def setup(self):
        pass

    def run_sim(self):
        sitl_args = " -S --model + --speedup 1 -I0"
        self.sitl_cmd = self.sitl_bin + sitl_args + " --defaults " + self.param_file
        try:
            self.sim_handle = subprocess.Popen(
                ["bash", "-c", self.sitl_cmd],
                # Temporarily commented out to debug
                # stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE,
                shell=False,
                preexec_fn=os.setsid,
            )
        except Exception as e:
            raise Exception(f"Simulation errored with {e}")

    def send_mission(self):
        self.tcp_conn.set_mode("GUIDED")
        self.tcp_conn.arm()
        self.tcp_conn.takeoff(100)  # Take off to 10 meters

    def cleanup_sim(self):
        self.tcp_conn.cleanup()
        self.sim_handle.terminate()
        print("Simulation terminated.")

    def start_fuzzing(self):
        # This is where the fuzzing logic will go
        # For now, just a placeholder
        print("Starting Fuzzing")


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
    argument_parser.add_argument("--peripheral", type=str, help="Peripheral to fuzz")
    argument_parser.add_argument(
        "--ap_dir", type=str, help="Ardupilot directory", required=True
    )

    args = argument_parser.parse_args()
    # TODO Add sanity checks for the input files and folders
    # Check if the bin exists
    cfg = FuzzConfig(args.bin, args.ap_dir)
    while not cfg.tcp_conn.drone_ready:
        print("Waiting for drone to be ready with GPS lock...")
        time.sleep(3)
    cfg.send_mission()
    time.sleep(100)
    cfg.cleanup_sim()
