import time
import signal
import subprocess
import json
import configparser
import logging

# from subprocess import *
import os
import psutil


# List to keep track of child processes
child_processes = []

# Setup the file for logging
logger = logging.getLogger("pgfuzz")
logger.setLevel(logging.INFO)
# Create a custom formatter
formatter = logging.Formatter(
    "%(asctime)s | Thread: %(threadName)s | PID: %(process)d | %(levelname)s | %(filename)s:%(lineno)d  | %(message)s"
)
# Create handlers
# Always add a stream handler to print to console
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
logger.addHandler(stream_handler)
# If a log file is specified, add a file handler
file_handler = logging.FileHandler("pgfuzz.log")
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

# class PGFUZZ_info:
#     def __init__(self, config_path=None):
#         self.config = {}
#         map_config = read_config(config_path)
#         try:
#             config = {
#                 "cur_pol_p_len": map_config["Required"]["Current_policy_P_length"],
#                 "PGFUZZHome": map_config["Required"]["PGFUZZHome"],
#                 "ArduPilotHome": map_config["Required"]["ArduPilotHome"],
#                 "SensorMapPath": map_config["Required"]["SensorMapPath"],
#                 "Sensor": map_config["Required"]["Sensor"],
#                 "TelegramToken": map_config["Required"]["TelegramToken"],
#                 "TelegramChatID": map_config["Required"]["TelegramChatID"],
#                 "MavlinkXMLFile": map_config["Required"]["MavlinkXMLFile"],
#                 "MissionDisabled": map_config["Required"]["MissionDisabled"],
#             }
#         except KeyError as e:
#             log("KeyError: {0}".format(e))
#
#         global Current_policy_P_length
#         global Current_policy
#         global ardupilot_dir
#         global pgfuzz_dir
#         global SUT
#         global telegram_token
#         global telegram_chat_id
#         global mavlink_xml_file
#         global msg_list
#         global mission_disabled
#         return self.config
#


def goodbye():
    logger.info("SIGINT received, terminator program is now active (`ー´)...")
    # use tmux kill-window to kill the tmux windows with pgfuzz in the name
    # List all tmux windows
    result = subprocess.check_output(["tmux", "list-windows"])
    session_name = tmux_session_info()
    for window in result.decode("utf-8").split("\n"):
        if "pgfuzz" in window:
            window_id = window.split(":")[0]
            window_name = window.split(" ")[1]
            window_target = "{0}:{1}".format(session_name, window_name)
            cmd = ["tmux", "send-keys", "-t", window_target, "C-c", "C-m"]
            subprocess.call(cmd)
            time.sleep(1)  # Give the process some time to terminate
            logger.info("Closing window: {0}".format(window_name))
            subprocess.call(["tmux", "kill-window", "-t", window_id])
        # Find Xterm process running ArduCopter and kill it
        arducopter_pid = None
        for proc in psutil.process_iter():
            if "xterm" in proc.name():
                # See if the process is ArduCopter
                if "ArduCopter" in proc.cmdline():
                    arducopter_pid = proc.pid
                    logger.info("Terminating random ArduCopter")
                    logger.info("ArduCopter PID: " + str(arducopter_pid))
                    proc.kill()
            if "ruby" in proc.name():
                gz_match = [x for x in proc.cmdline() if "gz" in x]
                if gz_match:
                    logger.info("Terminating random gazebo process")
                    proc.kill()


def sigint_handler(signum, _frame):
    if signum == signal.SIGINT:
        goodbye()
        exit(0)
    else:
        logger.info("Received signal: {0}".format(signum))
        logger.info("Not sure what to do, bailing out for now")


def tmux_session_info() -> str | None:
    tmux_env = os.getenv("TMUX")
    if tmux_env:
        # The TMUX variable is set to a value like "/tmp/tmux-1000/default,1234,0"
        # We need to get the session name which is usually part of the path
        result = subprocess.check_output(["tmux", "display-message", "-p", "#S"])
        session_name = result.strip().decode(
            "utf-8"
        )  # XXX: For now stick to UTF-8 names only
        return session_name
    else:
        return None


def tmux_window_exists(session_name: str, window_name: str) -> bool:
    result = subprocess.Popen(
        ["tmux", "list-windows", "-t", session_name],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    data = result.communicate()
    if window_name in data[0].decode("utf-8"):
        return True
    else:
        return False


def spawn_tmux_window(session_name="pgfuzz++", window_name="", command=""):
    """
    Spawns a new tmux window in the specified session and runs an optional command.

    :param session_name: Name of the tmux session to create the window in.
    :param window_name: Name of the new window.
    :param command: Command to run in the new window.
    """
    try:
        session_name = tmux_session_info()
        if not session_name:
            session_name = "pgfuzz++"
        else:
            # Check if the session exists
            result = subprocess.Popen(
                ["tmux", "has-session", "-t", session_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            result.communicate()
            if result.returncode != 0:
                # Create the session if it doesn't exist
                subprocess.call(["tmux", "new-session", "-d", "-s", session_name])
                logger.info(("Created new session: {0}".format(session_name)))

        if window_name == "":
            window_name = "default" + str(int(time.time()))
        # Create a new window in the specified session
        new_window_command = [
            "tmux",
            "new-window",
            "-d",
            "-t",
            session_name,
            "-n",
            window_name,
        ]
        # Create the window
        subprocess.call(new_window_command)

        # Run the command in the new window if provided
        if command:
            target = "{0}:{1}".format(session_name, window_name)
            # Log the PID of the child process
            cmd = ["tmux", "send-keys", "-t", target, command, "C-m"]
            process = subprocess.Popen(cmd)
            child_processes.append(process.pid)
            logger.info(("Running command {0}".format(command)))

        logger.info(("New window created in session {0}.".format(session_name)))

    except Exception as e:
        logger.info(("An error occurred: {0}".format(e)))


def read_config(config_path="pgfuzz.ini"):
    config = configparser.ConfigParser()
    # Check if the file exists
    if not os.path.exists(config_path):
        raise Exception("Config file pgfuzz.ini not found!")
    config.read("pgfuzz.ini")
    # If config doesn't have the necessary sections, raise an exception
    if "Required" not in config.sections():
        raise Exception("pgfuzz required section not found in pgfuzz.ini")
    return config


def set_sensor_parm(config):
    ap_home = config["Required"]["ArduPilotHome"]
    ap_param_file = ap_home + "./sensor.parm"
    ap_sensor = config["Required"]["Sensor"]
    # Load the sensor value from the file
    sensor_mapping_file = config["Required"]["SensorMapPath"]
    with open(sensor_mapping_file, "r") as f:
        sensor_map = json.load(f)
    # Find the sensor in the map
    sensor_row = None
    parameters = None
    for sensor_row in sensor_map:
        if sensor_row["sensor_type"] == ap_sensor:
            parameters = sensor_row["parameters"]
            break
    if parameters is None:
        logger.info("Sensor not found in the mapping file!")
        exit(1)
    # Write the sensor value with the parameter
    with open(ap_param_file, "w") as f:
        for keys in parameters.keys():
            # logger.info("Writing {0} {1}".format(keys, parameters[keys]))
            f.write("{0} {1:0.5f}\n".format(keys, float(parameters[keys])))


# PGFUZZ_HOME = os.getenv("PGFUZZ_HOME")
#
# if PGFUZZ_HOME is None:
#     raise Exception("PGFUZZ_HOME environment variable is not set!")
#
# ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")
#
# if ARDUPILOT_HOME is None:
#     raise Exception("ARDUPILOT_HOME environment variable is not set!")
if __name__ == "__main__":
    config = read_config()
    set_sensor_parm(config)
    pgfuzz_home = config["Required"]["PGFUZZHome"]

    open("restart.txt", "w").close()

    # Save the sensor value

    # Files to open
    working_dir = pgfuzz_home + "/ArduPilot/"
    open_simulator = working_dir + "open_simulator.py"
    # setup_sh = working_dir + "setup.sh"
    fuzzing_py = working_dir + "fuzzing.py"

    # Register the SIGINT handler
    signal.signal(signal.SIGINT, sigint_handler)

    cmd = "python3 " + open_simulator + "; exit"
    prg_name = "pgfuzz-sitl-" + str(int(time.time()))
    spawn_tmux_window(window_name=prg_name, command=cmd)

    time.sleep(20)  # NOTE: Time reduced for testing
    cmd = "python3 -i " + fuzzing_py
    prg_name = "pgfuzz-fuzzing-" + str(int(time.time()))
    spawn_tmux_window(window_name=prg_name, command=cmd)

    while True:
        time.sleep(1)
        f = open("restart.txt", "r")
        if not tmux_window_exists("", "fuzzing"):
            logger.info("Fuzzing window closed, adios!")
            goodbye()
            exit(0)
        if f.read() == "restart":
            f.close()
            time.sleep(2)  # Sleep for a while to kill everyone
            open("restart.txt", "w").close()
            cmd = "python3 " + open_simulator + "; exit"
            # Get the current datetime in Unix seconds
            prg_name = "pgfuzz-sitl-" + str(int(time.time()))
            spawn_tmux_window(window_name=prg_name, command=cmd)
        # Also check if the fuzzing window is open, if closed exit
