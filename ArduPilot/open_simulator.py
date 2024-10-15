from subprocess import Popen, PIPE

# import subprocess,time,psutil
import psutil

import time
import os
import signal
import logging

from pgfuzz import read_config

PR_SET_PDEATHSIG = 1  # This constant is for PR_SET_PDEATHSIG
PR_SET_PDEATHSIG_VALUE = signal.SIGTERM  # The signal to send when the parent dies
logger = logging.getLogger("pgfuzz")


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


config = read_config()
ardupilot_home = config["Required"]["ArdupilotHome"]
sim = config["Required"]["Simulator"]
handle = None

if ardupilot_home is None:
    raise Exception("config doesn't contain required info")

cmd_ap_sitl = (
    ardupilot_home
    + "Tools/autotest/sim_vehicle.py -v ArduCopter -D "  # Enable debug
    + "--out=udp:127.0.0.1:1337 "  # Sensor thread
    + "--out=udp:127.0.0.1:14551 "  # Monitoring
    + "--out=udp:127.0.0.1:14555 "  # QGC port
    + "--out=udpout:127.0.0.1:1338 "  # Monitoring thread
    + "--add-param-file="  # Always load PRX parameters
    + ardupilot_home
    + "sensor.parm"
)
cmd_ap_gz = ardupilot_home + "Tools/autotest/sim_vehicle.py -v ArduCopter -D "
cmd_ap_gz += "-f gazebo-iris --model JSON "
cmd_ap_gz += "--out=udp:127.0.0.1:1337 "  # Sensor thread
cmd_ap_gz += "--out=udpout:127.0.0.1:1338 "  # Monitoring thread
cmd_ap_gz += "--out=udp:127.0.0.1:14555 "  # QGC port
cmd_ap_gz += "--out=udpout:127.0.0.1:1339 "  # Plotting thread
cmd_ap_gz += "--add-param-file=" + ardupilot_home + "sensor.parm"

# TODO: Take the path from a config
# Setup gazebo
cmd_gz = "export DISPLAY=:1 "
# cmd_gz += "&& source " + GZ_SRC_PATH + "install/setup.bash "
cmd_gz += "&& source /home/silipwn/Documents/Drone/ardupilot_gazebo/setup.sh "
cmd_gz += "&& gz sim --verbose 4 -r iris_runway.sdf"
# Enable camera via gz topic
# cmd_gz_topic = "source " + GZ_SRC_PATH + "install/setup.bash "
cmd_gz_topic = "gz topic -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming -m gz.msgs.Boolean -p 'data: 1'"

# Depending upon simulation type, spawn instances
if sim == "Gazebo":
    handle_gz_sim = Popen(
        ["bash", "-c", cmd_gz],
        stdout=PIPE,
        stderr=PIPE,
        shell=False,
        preexec_fn=os.setsid,
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
    handle = handle_gz_sim  # XXX: Assuming that the gazebo process is the main process
    logger.info("The PID for the gazebo is: " + str(handle_gz_sim.pid))
    logger.info("The PID for the ardupilot is: " + str(handle_ap_gz.pid))

elif sim == "SITL":
    logger.info("Starting SITL")
    logger.info("Command: " + cmd_ap_sitl)
    handle_ap_sitl = Popen(
        ["bash", "-c", cmd_ap_sitl],
        stdout=PIPE,
        stderr=PIPE,
        shell=False,
        preexec_fn=os.setsid,
    )
    handle = handle_ap_sitl
    time.sleep(1)
    logger.info("The PID for the ardupilot is: " + str(handle_ap_sitl.pid))

while True:
    f = open("shared_variables.txt", "r")

    if handle is None:
        logger.info("Can't find a process handle")
        logger.info("Exiting the process 0")
        exit(0)

    # Check if the process is still running
    if handle.poll() is not None:
        logger.info("ERROR: Process has terminated unexpectedly")
        exit(0)

    if f.read() == "reboot":
        open("shared_variables.txt", "w").close()

        fi = open("restart.txt", "w")
        fi.write("restart")
        fi.close()
        logger.info("[WOAH] About to kill some parents :|")

        # Kill all the children as well
        if sim == "Gazebo":
            terminate_process_tree(handle_gz_sim.pid)
            logger.info("Terminated gazebo")
            terminate_process_tree(handle_ap_gz.pid)
            logger.info("Terminated ArduCopter")
        elif sim == "SITL":
            terminate_process_tree(handle_ap_sitl.pid)
            logger.info("Terminated ArduCopter")

        # Find Xterm process running ArduCopter and kill it
        arducopter_pid = None
        for proc in psutil.process_iter():
            if "xterm" in proc.name():
                # See if the process is ArduCopter
                if "ArduCopter" in proc.cmdline():
                    arducopter_pid = proc.pid
                    logger.info("Terminating ArduCopter")
                    logger.info("ArduCopter PID: " + str(arducopter_pid))
                    proc.kill()
        if arducopter_pid is None:
            logger.info("Warning: ArduCopter process not found")
            logger.info("Might create issues with the next simulation")

        logger.info("Terminated AP processes")
        exit(0)

    time.sleep(1)
