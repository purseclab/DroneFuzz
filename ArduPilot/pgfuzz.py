import time
import signal
import subprocess
import configparser

# from subprocess import *
import os
import psutil

# List to keep track of child processes
child_processes = []


def goodbye():
    print("SIGINT received, terminator program is now active (`ー´)...")
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
            print("Closing window: {0}".format(window_name))
            subprocess.call(["tmux", "kill-window", "-t", window_id])
        # Find Xterm process running ArduCopter and kill it
        arducopter_pid = None
        for proc in psutil.process_iter():
            if "xterm" in proc.name():
                # See if the process is ArduCopter
                if "ArduCopter" in proc.cmdline():
                    arducopter_pid = proc.pid
                    print("Terminating random ArduCopter")
                    print("ArduCopter PID: " + str(arducopter_pid))
                    proc.kill()
            if "ruby" in proc.name():
                gz_match = [x for x in proc.cmdline() if "gz" in x]
                if gz_match:
                    print("Terminating random gazebo process")
                    proc.kill()


def sigint_handler(signum, _frame):
    if signum == signal.SIGINT:
        goodbye()
        exit(0)
    else:
        print("Received signal: {0}".format(signum))
        print("Not sure what to do, bailing out for now")


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
                print(("Created new session: {0}".format(session_name)))

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
            print(("Running command {0}".format(command)))

        print(("New window created in session {0}.".format(session_name)))

    except Exception as e:
        print(("An error occurred: {0}".format(e)))


def read_config():
    config = configparser.ConfigParser()
    # Check if the file exists
    if not os.path.exists("pgfuzz.ini"):
        raise Exception("Config file pgfuzz.ini not found!")
    config.read("pgfuzz.ini")
    # If config doesn't have the necessary sections, raise an exception
    if "Required" not in config.sections():
        raise Exception("pgfuzz required section not found in pgfuzz.ini")
    return config


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
    pgfuzz_home = config["Required"]["PGFUZZHome"]

    open("restart.txt", "w").close()

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
    cmd = "python3 " + fuzzing_py
    prg_name = "pgfuzz-fuzzing-" + str(int(time.time()))
    spawn_tmux_window(window_name=prg_name, command=cmd)

    while True:
        time.sleep(1)
        f = open("restart.txt", "r")
        if not tmux_window_exists("", "fuzzing"):
            print("Fuzzing window closed, adios!")
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
