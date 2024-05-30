import time
import datetime
import signal
import psutil
import subprocess
# from subprocess import *
from pdb import set_trace as bp
import os

# List to keep track of child processes
child_processes = []


def sigint_handler(signum, frame):
    print("SIGINT received, terminating child processes...")
    # TODO Actually kill the processes?
    exit(0)


def spawn_tmux_window(session_name='pgfuzz++', window_name='', command=''):
    """
    Spawns a new tmux window in the specified session and runs an optional command.

    :param session_name: Name of the tmux session to create the window in.
    :param window_name: Name of the new window.
    :param command: Command to run in the new window.
    """
    try:
        tmux_env = os.getenv('TMUX')
        if tmux_env:
            # The TMUX variable is set to a value like "/tmp/tmux-1000/default,1234,0"
            # We need to get the session name which is usually part of the path
            result = subprocess.check_output(
                ['tmux', 'display-message', '-p', '#S'])
            session_name = result.strip()
        else:
            # Check if the session exists
            result = subprocess.Popen(
                ['tmux', 'has-session', '-t', session_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
            result.communicate()
            if result.returncode != 0:
                # Create the session if it doesn't exist
                subprocess.call(
                    ['tmux', 'new-session', '-d', '-s', session_name])
                print("Created new session: {0}".format(session_name))

        # Create a new window in the specified session
        new_window_command = ['tmux', 'new-window', '-t', session_name]
        if window_name:
            new_window_command.extend(['-n', window_name])
        subprocess.call(new_window_command)

        # Run the command in the new window if provided
        if command:
            target = "{0}:{1}".format(
                session_name, window_name) if window_name else session_name
            subprocess.call(
                ['tmux', 'send-keys', '-t', target, command, 'C-m'])

        print("New window created in session {0}.".format(session_name))

    except Exception as e:
        print("An error occurred: {0}".format(e))


# except Exception as e:
#     print(f"An error occurred: {e}")

PGFUZZ_HOME = os.getenv("PGFUZZ_HOME")

if PGFUZZ_HOME is None:
    raise Exception("PGFUZZ_HOME environment variable is not set!")

ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")

if ARDUPILOT_HOME is None:
    raise Exception("ARDUPILOT_HOME environment variable is not set!")

open("restart.txt", "w").close()

# Files to open
working_dir = PGFUZZ_HOME + "ArduPilot/"
open_simulator = working_dir + "open_simulator.py"
setup_sh = working_dir + "setup.sh"
fuzzing_py = working_dir + "fuzzing.py"

# Register the SIGINT handler
signal.signal(signal.SIGINT, sigint_handler)

cmd = 'source ' + setup_sh + '; python2 ' + open_simulator + '; exit'
spawn_tmux_window(window_name='sitl', command=cmd)


time.sleep(90)
cmd = 'source ' + setup_sh + '; python2 ' + fuzzing_py + ' | tee fuzzing.log'
spawn_tmux_window(window_name='fuzzing', command=cmd)

while True:
    time.sleep(1)
    f = open("restart.txt", "r")
    if f.read() == "restart":
        f.close()
        open("restart.txt", "w").close()
        cmd = 'source ' + setup_sh + '; python2 ' + open_simulator + '; exit'
        # Get the current datetime in Unix seconds
        window_name = 'sitl-' + str(int(time.time()))
        spawn_tmux_window(window_name=window_name, command=cmd)
