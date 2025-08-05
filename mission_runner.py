import argparse
import logging
import random
import time
import yaml
import os
import threading
import ast
import subprocess
import shlex
from pymavlink import mavutil, mavwp

# Setup logging
logger = logging.getLogger("mission_runner")
logger.setLevel(logging.DEBUG)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)


class TCPConn:
    """A simplified TCP connection handler for MAVLink."""

    def __init__(self, connection_string="tcp:localhost:5760"):
        self.conn = mavutil.mavlink_connection(connection_string)
        self.conn.wait_heartbeat()
        logger.info("Heartbeat from system (system %u component %u)" % (self.conn.target_system, self.conn.target_component))
        self.drone_in_air = False
        self.rc_monitor = False
        self.supported_modes = ["GUIDED", "AUTO"] # Default modes
        
        # Start a thread to monitor communications
        self.monitor_thread = threading.Thread(target=self.monitor_comms, daemon=True)
        self.monitor_thread.start()

    def monitor_comms(self):
        """Monitors incoming MAVLink messages."""
        while True:
            msg = self.conn.recv_match(blocking=True)
            if not msg:
                continue
            if msg.get_type() == "STATUSTEXT":
                logger.debug(f"DRONE_MSG: {msg.text}")
                if "Mission Complete" in msg.text or "Reached destination" in msg.text:
                    self.drone_in_air = False
                    self.rc_monitor = False
                if "takeoff" in msg.text.lower():
                    self.drone_in_air = True
                if "Mission: 2 WP" in msg.text: # A bit of a magic string, but based on logs
                    self.rc_monitor = True


    def set_mode(self, mode):
        if mode not in self.conn.mode_mapping():
            logger.error(f"Mode {mode} not supported.")
            return
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.set_mode(mode_id)
        logger.info(f"Changing mode to: {mode}")

    def set_param(self, param_id, param_value):
        self.conn.mav.param_set_send(
            self.conn.target_system,
            self.conn.target_component,
            param_id.encode(),
            param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        logger.debug(f"Setting param {param_id} to {param_value}")

    def arm(self):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        self.conn.motors_armed_wait()
        logger.info("Vehicle armed")

    def upload_mission(self, mission_file):
        """Upload a mission from a file."""
        wploader = mavwp.MAVWPLoader()
        with open(mission_file, "r") as f:
            for i, line in enumerate(f):
                if i == 0:
                    if "QGC WPL 110" not in line:
                        raise Exception("File is not in QGC WPL 110 format")
                else:
                    line_split = line.split('\t')
                    seq = int(line_split[0])
                    current = int(line_split[1])
                    frame = int(line_split[2])
                    command = int(line_split[3])
                    param1 = float(line_split[4])
                    param2 = float(line_split[5])
                    param3 = float(line_split[6])
                    param4 = float(line_split[7])
                    param5 = float(line_split[8])
                    param6 = float(line_split[9])
                    param7 = float(line_split[10])
                    autocontinue = int(line_split[11].strip())
                    wp = mavutil.mavlink.MAVLink_mission_item_message(self.conn.target_system, self.conn.target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, param5, param6, param7)
                    wploader.add(wp)

        self.conn.waypoint_clear_all_send()
        self.conn.waypoint_count_send(wploader.count())

        for i in range(wploader.count()):
            msg = self.conn.recv_match(type=['MISSION_REQUEST'], blocking=True)
            self.conn.mav.send(wploader.wp(msg.seq))
            logger.info(f"Sending waypoint {msg.seq}")
        logger.info("Mission uploaded")

    def send_custom_message(self, msg_name, field_values):
        """Sends a MAVLink message with the given field values."""
        try:
            # The message name needs to be in the format MAVLink_{message_name}_message
            message_constructor = getattr(mavutil.mavlink, f"MAVLink_{msg_name.lower()}_message")
            
            # Create the message with the provided values
            # Note: This assumes field_values keys match the message constructor's arguments
            msg = message_constructor(**field_values)
            
            self.conn.mav.send(msg)
            logger.debug(f"Sent message: {msg_name} with values {field_values}")
        except AttributeError:
            logger.error(f"Could not find a MAVLink message constructor for '{msg_name}'")
        except Exception as e:
            logger.error(f"Error sending custom message {msg_name}: {e}")

    def wait_for_condition(self, condition, timeout=60):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if condition():
                return True
            time.sleep(0.5)
        logger.warning("Timeout waiting for condition")
        return False

    def stop_fuzzing(self):
        # This is a placeholder
        logger.info("Fuzzing stopped.")

    def random_param_set(self):
        # This is a placeholder for setting random parameters
        logger.debug("Setting a random parameter.")

    def error_sleep(self, duration):
        # This is a placeholder
        time.sleep(duration)
        return False # No error

    def reboot_and_wait_for_ack(self):
        """Reboots the vehicle and waits for it to be ready."""
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1, 0, 0, 0, 0, 0, 0)
        logger.info("Reboot command sent. Waiting for vehicle to reconnect...")
        self.conn.close()
        time.sleep(5) # Wait for SITL to restart
        self.__init__() # Re-initialize connection
        self.conn.wait_heartbeat()
        logger.info("Vehicle reconnected.")

    def cleanup(self, shutdown=True):
        if shutdown:
            self.conn.close()
            logger.info("Connection closed.")


def send_sensor_messages(tcp_conn, sensor_messages):
    """Sends sensor messages based on their timestamps."""
    logger.info("Starting to send sensor messages from file.")
    # Sort messages by timestamp
    sorted_messages = sorted(sensor_messages, key=lambda x: x[0])
    
    if not sorted_messages:
        logger.warning("Sensor file is empty, no messages to send.")
        return

    start_time = time.time()
    last_msg_timestamp = 0

    for msg_data in sorted_messages:
        timestamp, msg_name, _, values = msg_data
        
        # Calculate delay from the last message
        delay = timestamp - last_msg_timestamp
        if delay > 0:
            time.sleep(delay)
        
        tcp_conn.send_custom_message(msg_name, values)
        last_msg_timestamp = timestamp
    
    logger.info("Finished sending all sensor messages from file.")


def run_mission(args):
    """Connects, uploads mission, and runs the main loop."""
    config = None
    if args.config:
        with open(args.config, 'r') as f:
            config = yaml.safe_load(f)
    
    # Start the mission
    logger.info("Starting the mission")
    
    # Start the simulation with subprocess
    sitl_bin = os.path.join(args.ap_dir, 'build', 'sitl', 'bin', 'ardupilot')
    home_location = " --home -35.362938,149.165085,585,354 "
    sitl_args = ""
    if args.vehicle == "copter":
        sitl_args = " -S --model + -w --speedup 1 -I0"
    elif args.vehicle == "plane":
        sitl_args = " -S --model plane-elevrev -w --speedup 1 -I0"
    elif args.vehicle == "rover":
        sitl_args = " -S --model rover -w --speedup 1 -I0"
    
    sitl_cmd_str = sitl_bin + home_location + sitl_args
    if args.param_file:
        sitl_cmd_str += " --defaults " + args.param_file

    logger.info(f"Starting SITL with command: {sitl_cmd_str}")
    
    sim_handle = subprocess.Popen(
        shlex.split(sitl_cmd_str),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid
    )
    
    time.sleep(5) # Give some time for SITL to initialize

    try:
        init_conn = TCPConn()
        init_conn.reboot_and_wait_for_ack()
        init_conn.cleanup(shutdown=False)
        
        tcp_conn = TCPConn()
    except Exception as e:
        logger.error(f"Error starting simulation or connecting: {e}")
        sim_handle.terminate()
        return

    sensor_thread = None
    if args.sensor_file:
        if os.path.exists(args.sensor_file):
            sensor_messages = []
            with open(args.sensor_file, 'r') as f:
                for line in f:
                    try:
                        # Assuming each line is a list literal
                        sensor_messages.append(ast.literal_eval(line.strip()))
                    except (ValueError, SyntaxError) as e:
                        logger.error(f"Could not parse line in sensor file: {line.strip()} - {e}")
            
            if sensor_messages:
                sensor_thread = threading.Thread(target=send_sensor_messages, args=(tcp_conn, sensor_messages), daemon=True)
                sensor_thread.start()
            else:
                logger.warning("Sensor file was empty or contained no valid messages.")
        else:
            logger.error(f"Sensor file not found: {args.sensor_file}")

    if config and 'supported_modes' in config:
        tcp_conn.supported_modes = config['supported_modes']
    logger.info(f"Loaded modes from config: {tcp_conn.supported_modes}")

    if args.mission_file:
        tcp_conn.upload_mission(args.mission_file)
        tcp_conn.arm()
        tcp_conn.set_mode("AUTO")
    else:
        logger.warning("No mission file provided. The drone will not execute a mission.")
        # Even without a mission, we might want to arm and switch modes
        tcp_conn.arm()
        tcp_conn.set_mode("GUIDED")
        tcp_conn.drone_in_air = True # Assume in air for testing without mission
        tcp_conn.rc_monitor = True


    mode_state = []
    mode_ctr = 0
    prev_state = None
    MAX_MODE_CHANGES = args.max_mode_changes

    try:
        if args.fixed_modes:
            logger.info(f"Using fixed modes: {args.fixed_modes}")
            for mode in args.fixed_modes:
                if not (tcp_conn.rc_monitor and tcp_conn.drone_in_air):
                    logger.warning("Exiting mode change loop as drone is not in mission.")
                    break
                tcp_conn.set_mode(mode)
                if tcp_conn.error_sleep(5): # Wait for 5 seconds in each mode
                    raise Exception("Internal Error during fixed mode sequence.")
        else:
            logger.info("Using random modes.")
            while tcp_conn.rc_monitor and tcp_conn.drone_in_air:
                mode = random.choice(tcp_conn.supported_modes)
                if (
                    mode_ctr < MAX_MODE_CHANGES
                ):
                    tcp_conn.set_mode(mode)
                    mode_state.append(mode)
                    mode_ctr += 1
                    prev_state = mode
                elif mode_ctr >= MAX_MODE_CHANGES and prev_state != "AUTO":
                    logger.debug("Reached mode change limit, not changing mode anymore")
                    tcp_conn.set_mode("AUTO")
                    prev_state = "AUTO"
                    logger.debug("Resetting Setting mode to AUTO")
                if random.random() < 0.1:
                    tcp_conn.random_param_set()
                if tcp_conn.error_sleep(3):
                    raise Exception("Internal Error")
        
        tcp_conn.wait_for_condition(lambda: not tcp_conn.rc_monitor, timeout=60)
        
        while tcp_conn.drone_in_air:
            if tcp_conn.error_sleep(1):
                raise Exception("Internal Error")
        
        tcp_conn.stop_fuzzing()
        logger.info("Mission finished.")

    except Exception as e:
        logger.error(f"An error occurred: {e}")
    finally:
        logger.info("Cleaning up.")
        if 'sim_handle' in locals() and sim_handle.poll() is None:
            os.killpg(os.getpgid(sim_handle.pid), 15) # signal.SIGTERM
            sim_handle.wait()
            logger.info("Simulation terminated.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a drone mission with mode changes.")
    parser.add_argument("--mission_file", type=str, help="Path to the mission file.")
    parser.add_argument("--config", type=str, help="Path to a YAML config file for supported modes.")
    parser.add_argument("--sensor_file", type=str, help="Path to a YAML file with sensor messages to send.")
    parser.add_argument("--max_mode_changes", type=int, default=3, help="Maximum number of random mode changes.")
    parser.add_argument("--fixed_modes", nargs='+', help="A fixed list of modes to execute in sequence.")
    args = parser.parse_args()

    run_mission(args)
