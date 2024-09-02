from subprocess import Popen, PIPE

# import subprocess,time,psutil
import psutil

import time
import os
import signal

PR_SET_PDEATHSIG = 1  # This constant is for PR_SET_PDEATHSIG
PR_SET_PDEATHSIG_VALUE = signal.SIGTERM  # The signal to send when the parent dies


def terminate_process_tree(pid, timeout=5):
    try:
        parent = psutil.Process(pid)
        children = parent.children(recursive=True)

        # Send SIGTERM to parent and all children
        parent.terminate()
        for child in children:
            child.terminate()

        # Wait for processes to terminate
        gone, alive = psutil.wait_procs(children + [parent], timeout=timeout)

        # If any processes are still alive, send SIGKILL
        if alive:
            for p in alive:
                p.kill()
    except psutil.NoSuchProcess:
        pass


ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")

print((os.getcwd()))
if ARDUPILOT_HOME is None:
    raise Exception("ARDUPILOT_HOME environment variable is not set!")

c = (
    ARDUPILOT_HOME
    + "Tools/autotest/sim_vehicle.py -v ArduCopter -D "  # Enable debug
    + "--out=udp:127.0.0.1:1337 "  # Sensor thread
    + "--out=udp:127.0.0.1:14555 "  # QGC port
    + "--out=udpout:127.0.0.1:1338 "  # Monitoring thread
    + "--add-param-file="  # Always load PRX parameters
    + ARDUPILOT_HOME
    + "sensor.parm"
)
cmd_ap_gz = ARDUPILOT_HOME + "Tools/autotest/sim_vehicle.py -v ArduCopter -D "
cmd_ap_gz += "-f gazebo-iris --model JSON "
cmd_ap_gz += "--out=udp:127.0.0.1:1337 "  # Sensor thread
cmd_ap_gz += "--out=udpout:127.0.0.1:1338 "  # Monitoring thread
cmd_ap_gz += "--out=udp:127.0.0.1:14555 "  # QGC port
cmd_ap_gz += "--add-param-file=" + ARDUPILOT_HOME + "sensor.parm"

# TODO: Take the path from a config
GZ_PLUGIN_PATH = "/home/silipwn/Documents/ardupilot_gazebo/"
GZ_SRC_PATH = "/home/silipwn/Documents/gz-src/"
# Setup gazebo
cmd_gz = "export DISPLAY=:0 "
cmd_gz += "&& source " + GZ_SRC_PATH + "install/setup.bash "
cmd_gz += "&& source /home/silipwn/Documents/GAZEBO_ENV.sh "
cmd_gz += "&& gz sim --verbose 4 -r iris_runway.sdf"
# Enable camera via gz topic
cmd_gz_topic = "source " + GZ_SRC_PATH + "install/setup.bash "
cmd_gz_topic += "&& gz topic -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming -m gz.msgs.Boolean -p 'data: 1'"

# handle = Popen(c, shell=True)
# handle = Popen(cmd_gz, shell=True)
handle = Popen(
    ["bash", "-c", cmd_gz], stdout=PIPE, stderr=PIPE, shell=False, preexec_fn=os.setsid
)
time.sleep(5)
handle_topic = Popen(
    ["bash", "-c", cmd_gz_topic], stdout=PIPE, stderr=PIPE, shell=False
)
time.sleep(1)
handle_ap_gz = Popen(
    ["bash", "-c", cmd_ap_gz],
    stdout=PIPE,
    stderr=PIPE,
    shell=False,
    preexec_fn=os.setsid,
)
# Print return output from the command
# print(handle_topic.communicate())
# # Print return code from the command
# print(handle_topic.returncode)
# handle_topic = Popen(cmd_gz_topic, shell=True)
print("The PID for the gazebo is: " + str(handle.pid))
print("The PID for the ardupilot is: " + str(handle_ap_gz.pid))

# os.killpg(os.getpgid(handle.pid), signal.SIGTERM)

while True:
    f = open("shared_variables.txt", "r")

    if f.read() == "reboot":
        open("shared_variables.txt", "w").close()

        fi = open("restart.txt", "w")
        fi.write("restart")
        fi.close()
        print("[WOAH] About to kill some parents :|")

        # Kill all the children as well
        # os.killpg(os.getpgid(handle.pid), signal.SIGTERM)
        terminate_process_tree(handle.pid)
        print("Terminated gazebo")

        # os.killpg(os.getpgid(handle_ap_gz.pid), signal.SIGTERM)
        terminate_process_tree(handle_ap_gz.pid)

        # Find Xterm process running ArduCopter and kill it
        arducopter_pid = None
        for proc in psutil.process_iter():
            if "xterm" in proc.name():
                # See if the process is ArduCopter
                if "ArduCopter" in proc.cmdline():
                    arducopter_pid = proc.pid
                    print("Terminating ArduCopter")
                    print("ArduCopter PID: " + str(arducopter_pid))
                    proc.kill()
        if arducopter_pid is None:
            print("Warning: ArduCopter process not found")
            print("Might create issues with the next simulation")

        print("Terminated AP processes")
        exit(0)

    time.sleep(1)
