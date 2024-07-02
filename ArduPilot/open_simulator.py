from subprocess import *
# import subprocess,time,psutil

import time
import os
import signal
import psutil

# subprocess.call(['~/ardupilot_4_0_3/Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w'], shell=True)

ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")

print((os.getcwd()))
if ARDUPILOT_HOME is None:
    raise Exception("ARDUPILOT_HOME environment variable is not set!")

# c = ARDUPILOT_HOME + "Tools/autotest/sim_vehicle.py -v ArduCopter -w --out=udp:127.0.0.1:1337 -D"
c = (
    ARDUPILOT_HOME
    + "Tools/autotest/sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:1337 -D --gdb"
)
# "Tools/autotest/sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:1337 -D --gdb --out=tcpin:127.0.0.1:14560"
# c = '~/ardupilot_pgfuzz/Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w'

# handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
handle = Popen(c, shell=True)

# os.killpg(os.getpgid(handle.pid), signal.SIGTERM)

while True:
    f = open("shared_variables.txt", "r")

    if f.read() == "reboot":
        open("shared_variables.txt", "w").close()

        fi = open("restart.txt", "w")
        fi.write("restart")
        fi.close()
        print("[WOAH] About to kill some parents :|")

        os.killpg(os.getpgid(handle.pid), signal.SIGTERM)

    time.sleep(1)

"""
print(os.getpgid(handle.pid))
print(handle.pid)

parent_pid = handle.pid
parent = psutil.Process(parent_pid)
for child in parent.children(recursive=True):  # or parent.children() for recursive=False
    child.kill()
    print("Kill:%d" %child)

parent.kill()
"""
