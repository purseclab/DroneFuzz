import time
from subprocess import *
import os

PGFUZZ_HOME = os.getenv("PGFUZZ_HOME")

if PGFUZZ_HOME is None:
    raise Exception("PGFUZZ_HOME environment variable is not set!")

ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")

if ARDUPILOT_HOME is None:
    raise Exception("ARDUPILOT_HOME environment variable is not set!")

if (os.getenv('TOKEN') or os.getenv('CHAT_ID')) is None:
    raise Exception("Telegram notification/chat_id not set")

open("restart.txt", "w").close()

c = "gnome-terminal -- bash -c 'python2 " + PGFUZZ_HOME + "ArduPilot/open_simulator.py; exec bash'"
handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
print(c)

time.sleep(90)
c = """gnome-terminal -- bash -c 'python2 $PGFUZZ_HOME/ArduPilot/fuzzing.py; \
bash $PGFUZZ_HOME/ArduPilot/notify.sh ; exec bash'"""
handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
print(c)

while True:
    time.sleep(1)

    f = open("restart.txt", "r")

    if f.read() == "restart":
        f.close()
        open("restart.txt", "w").close()
        c = "gnome-terminal -- bash -c 'python2 " + PGFUZZ_HOME + "ArduPilot/open_simulator.py; exec bash'"
        handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
