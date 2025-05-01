# Basically a singular file to do the fuzzing loop
# Start the SITL binary
# Establish the TCP connection
# Set the guided mission
# Fuzz some messages based on the XML loading
# Check the status after the mission finishes
import argparse
import time
import os
from pymavlink import mavutil
import subprocess
from queue import Queue
import threading

mavlink_timeout = 5


class TCPConn:
    def __init__(self):
        self.conn = mavutil.mavlink_connection("tcp:localhost:5760")
        self.lock = threading.Lock()
        self.conn.wait_heartbeat()
        self.connected = True
        self.msg_queue = Queue()
        # Start a thread to keep sending heartbeats
        threading.Thread(self.send_heartbeat(), daemon=True).start()
        # Start a thread to monitor communications
        threading.Thread(self.monitor_comms(), daemon=True).start()

    def send_heartbeat(self):
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,  # Ground Control Station
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        time.sleep(1)  # Sleep for a second before sending the next heartbeat

    def monitor_comms(self):
        msg = self.conn.recv_match(blocking=True, timeout=1)
        print(msg.to_dict())
        self.msg_queue.put(msg)

    def msg_recv(self, msg_type, timeout=mavlink_timeout):
        return self.conn.recv_match(type=msg_type, timeout=timeout, blocking=True)

    def msg_send(self):
        msg = mavutil.mavlink.MAVLink_statustext_message()
        return self.conn.mav.send(msg)

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

    def tcp_conn(self):
        pass

    def setup(self):
        pass

    def run_sim(self):
        sitl_args = " -S --model + --speedup 1 -I0"
        self.sitl_cmd = self.sitl_bin + sitl_args + "--defaults " + self.param_file
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

    def cleanup_sim(self):
        self.sim_handle.terminate()
        print("Simulation terminated.")

    def guide_mission(self):
        pass


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
    argument_parser.add_argument("--msg", type=str, help="Message to fuzz")
    argument_parser.add_argument(
        "--ap_dir", type=str, help="Ardupilot directory", required=True
    )

    args = argument_parser.parse_args()
    # TODO Add sanity checks for the input files and folders
    # Check if the bin exists
    cfg = FuzzConfig(args.bin, args.ap_dir)
    cfg.run_sim()
    tcp_conn = TCPConn()
    # Debugging test
    time.sleep(30)
    #
    cfg.cleanup_sim()
    tcp_conn.cleanup()
