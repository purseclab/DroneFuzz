"""
Author: Hyungsub Kim
Date: 05/20/2020
Name of file: fuzzing.py
Goal: Main loop of PGFUZZ
"""

# !onsusr/bin/env python

import sys
import os

import time
import datetime
import random
import numpy
import threading
import queue
import subprocess
import requests
import multiprocessing

# Tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../mavlink"))
# TODO: Figure out Ruff issue with following lines
import pymavlink
from pymavlink import mavutil, mavwp  # noqa: F401
from pymavlink import mavextra  # noqa: F401
from pymavlink import mavexpression  # noqa: F401
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
# NOTE: pymavlink needs to be an old version 2.4.37 (Cause it doesn't support Py2 :|)
# pip2 install --force-reinstall -v "pymavlink==2.4.37"

import read_inputs
import shared_variables

import timeit
import re
import math

# ------------------------------------------------------------------------------------
# Global variables
# master = mavutil.mavlink_connection("127.0.0.1:14551")
home_altitude = 0
home_lat = 0
home_lon = 0

current_rc_1 = 0
current_rc_2 = 0
current_rc_3 = 0
current_rc_4 = 0

takeoff = 0
drone_status = 0
wp_cnt = 0
lat = 0
lon = 0
# if 0: up / 1: down
target_throttle = 0
goal_throttle = 0
executing_commands = 0
PARAM_MIN = 1
PARAM_MAX = 10000
MISSION_ATTITUDE = 100
required_min_thr = 975

current_roll = 0.0
current_pitch = 0.0
current_heading = 0.0

alt_error = 0.0
roll_error = 0.0
pitch_error = 0.0
heading_error = 0.0
stable_counter = 0

alt_series = 0.0
roll_series = 0.0
pitch_series = 0.0
heading_series = 0.0

roll_series_flip = 0.0
pitch_series_flip = 0.0
heading_series_flip = 0.0
stable_counter_flip = 0

flip_start_time = 0.0

roll_initial = 0.0
pitch_initial = 0.0
yaw_initial = 0.0
ground_speed = 0.0

position_cnt = 0
lat_series = 0.0
lon_series = 0.0
lat_current = 0.0
lat_previous = 0.0
lon_current = 0.0
lon_previous = 0.0
vertical_speed = 0
relative_alt = 0

circle_radius_current = 0.0
circle_radius_previous = 0.0

rollspeed_current = 0.0
rollspeed_previous = 0.0
pitchspeed_current = 0.0
pitchspeed_previous = 0.0
yawspeed_current = 0.0
yawspeed_previous = 0.0

# Custom policy addition
yaw_min = 0.0
yaw_max = 0.0
roll_min = 0.0
roll_max = 0.0
pitch_min = 0.0
pitch_max = 0.0

# states - 0: turn off, 1: turn on
Parachute_on = 0
Armed = 0
current_flight_mode = ""
previous_flight_mode = ""
previous_altitude = 0
current_altitude = 0
current_alt = 0.0
previous_alt = 0.0
GPS_status = 1
Accel_status = 1
Gyro_status = 1
Baro_status = 1
PreArm_error = 0
failsafe_error = 0
hit_ground = 0
RC_failsafe_error = 0

system_time = 0
mission_cnt = 0
num_GPS = 0
alt_GPS_series = 0.0
vertical_speed_series = 0.0
gps_message_cnt = 0
actual_throttle = 0

# Sensor metrics
start_time = int(round(time.time() * 1000))
DEPTH_RANGE = [0.3, 12]  # depth range, to be changed as per requirements
depth_range_x = depth_range_y = depth_range_z = [0.00] * 9
mavlink_lock = threading.Lock()
mavlink_msg_queue = queue.Queue()
mavlink_pause_event = threading.Event()
reboot_pause_event = threading.Event()


# Distance
P = []
Global_distance = 0
Previous_distance = []

gps_failsafe_cnt = 0
brake_cnt = 0

target_param = ""
target_param_ready = 0
target_param_value = 0

REBOOT_START = 0

# A pair of input (input, value)
Current_input = ""
Current_input_val = ""
Guidance_decision = None
Policy_violation_cnt = 0
count_main_loop = 0

# Heartbeat
heartbeat_cnt = 1
RV_alive = 0

# Policy
Precondition_path = ""
# Current_policy = "A.CHUTE"
# Current_policy_P_length = 5
# Current_policy = "A.RTL1"
# Current_policy_P_length = 4
# Current_policy = "A.RTL2"
# Current_policy_P_length = 5
# Current_policy = "A.RTL3"
# Current_policy_P_length = 4
# Current_policy = "A.RTL4"
# Current_policy_P_length = 3
Current_policy = "A.RANGEFINDER"
Current_policy_P_length = 3

# Debug parameter
PRINT_DEBUG = 0


# Send a curl request to telegram
def send_telegram_message(message):
    # Get hostname
    hostname = os.uname()[1]
    # Get the os env commands to get the token and chat_id
    token = os.getenv("TOKEN")
    chat_id = os.getenv("CHAT_ID")
    if token is None or chat_id is None:
        print("TOKEN and CHAT_ID environment variables not set")
        return
    final_msg = "pgfuzz++ raised an exception on " + hostname + ": " + message
    # Send a message to the telegram using requests
    requests.post(
        "https://api.telegram.org/bot{}/sendMessage".format(token),
        data={"chat_id": chat_id, "text": final_msg},
    )


# Print with time
def log(message, filename="fuzzing.log"):
    # Get the current time
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Get the current thread name
    thread_name = threading.current_thread().name

    # Get the current process ID
    process_id = os.getpid()

    # Format the log message
    log_message = "[{}] [Thread: {}] [Process: {}] {}".format(
        current_time, thread_name, process_id, message
    )
    print(log_message)
    # Append the message to the file
    with open(filename, "a") as log_file:
        log_file.write(log_message + "\n")


## Reboot the Vehicle via MAVLINK
def reboot_vehicle():
    log("Rebooting vehicle")
    mav_conn = mavutil.mavlink_connection(":127.0.0.1:14551")
    # Send a reboot command to the vehicle
    mav_conn.mav.command_long_send(
        mav_conn.target_system,
        mav_conn.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    # Check for the response back
    log("Waiting for response")
    while True:
        ack_msg = mav_conn.recv_match(type="COMMAND_ACK", blocking=True)
        resp = ack_msg.to_dict()
        if resp["command"] != mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            continue
        log("Received response back")
        break
    mav_conn.close()


# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
# If the RV does not response within 5 seconds, we consider the RV's program crashed.
def check_liveness():
    # TODO: 2024-07-03T18:55:16-0400: silipwn: Maybe we can rewrite this?
    # Cause there's already a loop checking for existing MAVLINK messages?
    # Another test could be that the messages have stopped?
    global heartbeat_cnt

    end_flag = 0
    while end_flag == 0:
        while reboot_pause_event.is_set():
            log("Pausing liveness for 10 seconds")
            time.sleep(10)
        if heartbeat_cnt < 1:
            log(
                "The value of heartbeat_cnt is {0}, will store inputs".format(
                    heartbeat_cnt
                )
            )
            store_mutated_inputs()
            # The RV software is crashed
            f = open("shared_variables.txt", "w")
            f.write("reboot")
            f.close()
            end_flag = 1
        else:
            heartbeat_cnt = 0

        time.sleep(5)


# ------------------------------------------------------------------------------------
def set_preconditions(filepath):
    mav_conn = mavutil.mavlink_connection("127.0.0.1:14551")
    for line in open(filepath, "r").readlines():
        # 2024-06-25T10:45:07-0400: silipwn: Check if this approach would work,
        # basically all reads are as bytes
        row = line.rstrip().split(" ")
        # master.mav.param_set_send(
        #     master.target_system,
        #     master.target_component,
        #     row[0],
        #     row[1],
        #     mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        # )
        # time.sleep(1)
        #
        # log(("[Set_preconditions] %s = %s" % (row[0], row[1])))
        log("[Set_preconditions] currently ignored: TODO Fix this")
    mav_conn.close()


# ------------------------------------------------------------------------------------
def write_guidance_log(print_log, action):
    # Log mutated user command
    if action == "append":
        guidance_log = open("guidance_log.txt", "a")
        guidance_log.write(print_log)
        guidance_log.close()
        log(("[write_guidance_log] appending: %s" % print_log))

    elif action == "write":
        guidance_log = open("guidance_log.txt", "w")
        guidance_log.close()

        guidance_log = open("guidance_log.txt", "w")
        guidance_log.write(print_log)
        guidance_log.close()
        log(("[write_guidance_log] re-writing: %s" % print_log))


# ------------------------------------------------------------------------------------
def write_log(print_log):
    # Log mutated user command
    mutated_log = open("mutated_log.txt", "a")
    mutated_log.write(print_log)
    mutated_log.close()


# ------------------------------------------------------------------------------------
def re_launch():
    global goal_throttle
    global home_altitude
    global current_altitude
    global current_flight_mode
    global Parachute_on
    global count_main_loop
    global GPS_status
    global Accel_status
    global Gyro_status
    global Baro_status
    global PreArm_error
    global RV_alive

    RV_alive = 0
    count_main_loop = 0
    Parachute_on = 0
    GPS_status = 1
    Accel_status = 1
    Gyro_status = 1
    Baro_status = 1
    PreArm_error = 0

    reboot_pause_event.set()
    log("Enabled reboot_pause_event")

    # Reset the prx_values
    for i in range(9):
        depth_range_x[i] = random.uniform(-DEPTH_RANGE[1], DEPTH_RANGE[1])
        depth_range_y[i] = random.uniform(-DEPTH_RANGE[1], DEPTH_RANGE[1])
        depth_range_z[i] = random.uniform(-DEPTH_RANGE[1], DEPTH_RANGE[1])

    log(
        "#------------------------- RE-LAUNCH the vehicle -----------------------------"
    )

    mav_conn = mavutil.mavlink_connection("127.0.0.1:14551")
    _ = mav_conn.recv_match(type="HEARTBEAT", blocking=True)
    # Step 1. land the vehicle
    # master.mav.set_mode_send(
    #     master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
    # )
    mode_id = mav_conn.mode_mapping()["RTL"]
    mav_conn.set_mode_rtl()

    # Wait for the copter to go into the RTL mode
    while True:
        hb = mav_conn.recv_match(type="HEARTBEAT", blocking=True)
        hb = hb.to_dict()
        if hb["custom_mode"] == mode_id:
            break

    home_altitude = current_altitude

    time.sleep(5)

    # Initializing RC channels
    global required_min_thr
    # Refer to https://discuss.ardupilot.org/t/arming-problem-fs-thr-value/806/2
    goal_throttle = required_min_thr + 20

    log(
        (
            "[re-launch] min_thr:%d, target throttle:%d"
            % (required_min_thr, goal_throttle)
        )
    )
    set_rc_channel_pwm(1, mav_conn, 1500)
    set_rc_channel_pwm(2, mav_conn, 1500)
    set_rc_channel_pwm(4, mav_conn, 1500)

    time.sleep(3)

    # Step 2. Reboot the RV's control program
    f = open("shared_variables.txt", "w")
    f.write("reboot")
    f.close()

    time.sleep(48)
    # Restablish the connection since it would be rebooted now
    mav_conn = mavutil.mavlink_connection("127.0.0.1:14551")

    mutated_log = open("mutated_log.txt", "w")
    mutated_log.close()

    # Step 3. reset preconditions to fuzz the target policy
    global Precondition_path
    set_preconditions(Precondition_path)
    mavlink_pause_event.set()
    log("Setting event")
    # reboot_vehicle()

    # Try to read the STATUSTEXT msgs till we get the GPS usage
    # while True:
    #     msg = master.recv_match(type="STATUSTEXT", blocking=True)
    #     if "EKF3 IMU0 is using GPS" in msg.text:
    #         log("Got GPS usage message")
    #         break

    log("Clearing event")
    mavlink_pause_event.clear()
    time.sleep(5)

    # Step 4. re-take off the vehicle
    # master.mav.set_mode_send(
    #     master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4
    # )
    mode_id = mav_conn.mode_mapping()["GUIDED"]
    mav_conn.set_mode(mode_id)

    # Wait for finishing the landing
    while True:
        if current_flight_mode == "GUIDED":
            break
        time.sleep(0.2)

    time.sleep(3)

    # Arming
    mav_conn.mav.command_long_send(
        mav_conn.target_system,
        mav_conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    time.sleep(3)

    mav_conn.mav.command_long_send(
        mav_conn.target_system,  # target_system
        mav_conn.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
        0,  # confirmation
        0,  # param1
        0,  # param2
        0,  # param3
        0,  # param4
        0,  # param5
        0,  # param6
        100,
    )  # param7- altitude

    time.sleep(25)

    reboot_pause_event.clear()
    log("Cleared reboot_pause_event")
    mav_conn.close()
    goal_throttle = 1500


# ------------------------------------------------------------------------------------
def verify_real_number(item):
    """Method to find if an 'item'is real number"""

    item = str(item).strip()
    if not (item):
        return False
    elif item.isdigit():
        return True
    elif re.match(r"\d+\.*\d*", item) or re.match(r"-\d+\.*\d*", item):
        return True
    else:
        return False


def current_milli_time(start_time):
    return int(round(time.time() * 1000) - start_time)


def send_msg_rangefinder():
    conn_rangefinder = mavutil.mavlink_connection("127.0.0.1:1337")
    conn_rangefinder.recv_match(type="HEARTBEAT", blocking=True)
    log("Got heartbeat")
    hz = 25
    log("Starting while True")
    while True:
        try:
            mutated_msg = mavlink_msg_queue.get(timeout=1)
            depth_range_x = mutated_msg[0]
            depth_range_y = mutated_msg[1]
            depth_range_z = mutated_msg[2]
        except queue.Empty:
            depth_range_x = numpy.random.uniform(9, 12, 9)
            depth_range_y = numpy.random.uniform(9, 12, 9)
            depth_range_z = numpy.random.uniform(9, 12, 9)
        cur_ms_time = current_milli_time(start_time)
        for i in range(9):
            msg = mavlink2.MAVLink_obstacle_distance_3d_message(
                cur_ms_time,  # us Timestamp (UNIX time or time since system boot)
                0,  # not implemented in ArduPilot
                12,  # Set the frame to MAV_FRAME_BODY_FRD
                65535,  # unknown ID of the object. We are not really detecting the type of obstacle
                float(depth_range_x[i]),  # X in NEU body frame
                float(depth_range_y[i]),  # Y in NEU body frame
                float(depth_range_z[i]),  # Z in NEU body frame
                float(DEPTH_RANGE[0]),  # min range of sensor
                float(DEPTH_RANGE[1]),  # max range of sensor
            )
            conn_rangefinder.mav.send(msg)
            time.sleep(1 / hz)
        time.sleep(1)


def randomize_msg_rangefinder():
    """
    Should ideally randomize the Rangefinder depth values
    """
    # 2024-05-29T09:45:14-0400: silipwn: Split up the randomization in order to ensure the values are
    # not repeated
    # XXX: Why does the randomization fail in a loop?
    depth_range_x = numpy.random.uniform(-(DEPTH_RANGE[1] / 2), (DEPTH_RANGE[1] / 2), 9)
    depth_range_y = numpy.random.uniform(-(DEPTH_RANGE[1] / 2), (DEPTH_RANGE[1] / 2), 9)
    depth_range_z = numpy.random.uniform(-(DEPTH_RANGE[1] / 2), (DEPTH_RANGE[1] / 2), 9)
    # depth_range_1 = [12, 12, 12, 12, 12, 12, 12, 12, 12]
    # depth_range_2 = [-7.171149112500025, 3.468601209163399, 5.695193805774039, -9.266968362509475, 11.667519738575635, 5.1041530139455205, 0.45212570746143044, -7.864746056681961, -2.337234663244491]
    # depth_range_3 = [9.170353381650724, 8.632255888031327, 3.9505021451729547, -6.620548981848094, -0.6074463507713102, 2.6709162689533255, 3.3385703109376905, 3.4818020706529715, 9.12005049574292]
    # val = random.choice([depth_range_1])
    # print("Selected value: ", val)
    # depth_range_x = depth_range_y = depth_range_z = val
    combined_msg = [depth_range_x, depth_range_y, depth_range_z]
    mavlink_msg_queue.put(combined_msg)

    depth_range_x_str = numpy.reshape(depth_range_x, (1, len(depth_range_x)))
    depth_range_y_str = numpy.reshape(depth_range_y, (1, len(depth_range_y)))
    depth_range_z_str = numpy.reshape(depth_range_z, (1, len(depth_range_z)))

    print_param = ""
    print_param += "R "
    print_param += numpy.array2string(depth_range_x_str, separator=",")
    print_param += "|"
    print_param += numpy.array2string(depth_range_y_str, separator=",")
    print_param += "|"
    print_param += numpy.array2string(depth_range_z_str, separator=",")
    print_param += "\n"

    log("Generated random msgs for rangefinder")
    write_log(print_param)


# ------------------------------------------------------------------------------------
def change_parameter(selected_param):
    global Guidance_decision
    global Current_input
    global Current_input_val

    log(
        ("# [Change_parameter()] selected params: %s")
        % read_inputs.param_name[selected_param]
    )

    no_range = 0
    mav_conn = mavutil.mavlink_connection("127.0.0.1:14551")
    param_name = read_inputs.param_name[selected_param]

    if Guidance_decision == True:
        Current_input_val = match_cmd(cmd=param_name)

    range_min = read_inputs.param_min[selected_param]
    range_max = read_inputs.param_max[selected_param]
    param_value = 0

    # Step 1. Check whether the selected parameter has an valid range or not
    if range_min == "X":
        no_range = 1
        param_value = random.randint(PARAM_MIN, PARAM_MAX)
        log(
            (
                "[param] selected params: %s, there is no min of valid range, random param value:%d"
                % (read_inputs.param_name[selected_param], param_value)
            )
        )

    elif verify_real_number(range_min) == True:
        no_range = 0
        if range_min.isdigit() == True and range_max.isdigit() == True:
            param_value = random.randint(int(range_min), int(range_max))
            log(
                (
                    "# [Change_parameter()] selected params: %s, min: %f, max: %f, random digit param value:%d"
                    % (
                        read_inputs.param_name[selected_param],
                        float(range_min),
                        float(range_max),
                        param_value,
                    )
                )
            )

        elif range_min.isdigit() == False or range_max.isdigit() == False:
            param_value = random.uniform(float(range_min), float(range_max))
            log(
                (
                    "# [Change_parameter()] selected params: %s, min: %f, max: %f, random real param value:%f"
                    % (
                        read_inputs.param_name[selected_param],
                        float(range_min),
                        float(range_max),
                        param_value,
                    )
                )
            )

    # Step 2. Change the parameter value

    # 1) Request parameter
    global required_min_thr
    if param_name == "FS_THR_VALUE":
        param_value = random.randint(925, 975)
        required_min_thr = param_value
        log(("# Required minimum throttle is %d" % param_value))

    if Current_input_val != "null":
        param_value = float(Current_input_val)
        log(
            (
                "@@@[Reuse stored input pair] (%s, %s)@@@"
                % (param_name, Current_input_val)
            )
        )

    # 2) Set parameter value
    mav_conn.mav.param_set_send(
        mav_conn.target_system,
        mav_conn.target_component,
        param_name,
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    # Log change parameter values

    Current_input = param_name
    Current_input_val = str(param_value)

    print_param = ""
    print_param += "P "
    print_param += param_name
    print_param += " "
    print_param += str(param_value)
    print_param += "\n"

    write_log(print_param)
    mav_conn.close()

    time.sleep(3)


# ------------------------------------------------------------------------------------
# --------------------- (Start) READ Robotic Vehicle's states ------------------------
def handle_heartbeat(msg):
    global heartbeat_cnt
    heartbeat_cnt += 1

    # log("Just got a heartbeat : {0}".format(heartbeat_cnt))
    global current_flight_mode
    global previous_flight_mode

    if previous_flight_mode != mavutil.mode_string_v10(msg):
        previous_flight_mode = current_flight_mode

    current_flight_mode = mavutil.mode_string_v10(msg)

    global drone_status
    drone_status = msg.system_status
    # print("Drone status: %d, mavlink version: %d" % (drone_status, msg.mavlink_version))

    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED


# print("Mode: %s" % current_flight_mode)


# ------------------------------------------------------------------------------------
def handle_rc_raw(msg):
    global current_rc_1
    global current_rc_2
    global current_rc_3
    global current_rc_4

    current_rc_1 = msg.chan1_raw
    current_rc_2 = msg.chan2_raw
    current_rc_3 = msg.chan3_raw
    current_rc_4 = msg.chan4_raw

    # print("RC1:%d, RC2:%d, RC3:%d, RC3:%d" %(current_rc_1, current_rc_2, current_rc_3, current_rc_4))

    channels = (
        msg.chan1_raw,
        msg.chan2_raw,
        msg.chan3_raw,
        msg.chan4_raw,
        msg.chan5_raw,
        msg.chan6_raw,
        msg.chan7_raw,
        msg.chan8_raw,
    )


# ------------------------------------------------------------------------------------
def handle_hud(msg):
    hud_data = (
        msg.airspeed,
        msg.groundspeed,
        msg.heading,
        msg.throttle,
        msg.alt,
        msg.climb,
    )
    # print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
    # print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

    # print("[Debug] Alt: %f, Throttle:%f" %(msg.alt, msg.throttle))

    global current_altitude
    global previous_altitude
    global current_heading
    global actual_throttle
    global current_flight_mode
    global yaw_initial
    global ground_speed

    ground_speed = msg.groundspeed  # m/s
    previous_altitude = current_altitude
    current_altitude = msg.alt

    current_heading = msg.heading - 359
    actual_throttle = msg.throttle

    if current_flight_mode == "FLIP":
        if yaw_initial == 0:
            yaw_initial = current_heading


# ------------------------------------------------------------------------------------
def handle_attitude(msg):
    global current_roll
    global current_pitch

    global current_flight_mode

    global roll_initial
    global pitch_initial
    global current_flight_mode

    global rollspeed_current
    global rollspeed_previous
    global pitchspeed_current
    global pitchspeed_previous
    global yawspeed_current
    global yawspeed_previous

    attitude_data = (
        msg.roll,
        msg.pitch,
        msg.yaw,
        msg.rollspeed,
        msg.pitchspeed,
        msg.yawspeed,
    )
    # print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
    # print "%0.6f\t%0.6f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data

    rollspeed_previous = rollspeed_current
    pitchspeed_previous = pitchspeed_current
    yawspeed_previous = yawspeed_current

    rollspeed_current = msg.rollspeed
    pitchspeed_current = msg.pitchspeed
    yawspeed_current = msg.yawspeed

    current_roll = (msg.roll * 180) / math.pi
    current_pitch = (msg.pitch * 180) / math.pi

    if current_flight_mode == "FLIP":
        if roll_initial == 0:
            roll_initial = current_roll
            pitch_initial = current_pitch


# ------------------------------------------------------------------------------------
def handle_target(msg):
    global current_roll
    global current_pitch
    global current_altitude
    global alt_error
    global roll_error
    global pitch_error
    global heading_error

    global alt_series
    global roll_series
    global pitch_series
    global heading_series
    global stable_counter

    global roll_series_flip
    global pitch_series_flip
    global heading_series_flip
    global stable_counter_flip
    global flip_start_time
    global current_flight_mode

    # print("altitude error:%f" %msg.alt_error)
    # msg.alt_error
    """
	nav_roll	float	deg	Current desired roll
	nav_pitch	float	deg	Current desired pitch
	nav_bearing	int16_t	deg	Current desired heading
	target_bearing	int16_t	deg	Bearing to current waypoint/target
	wp_dist	uint16_t	m	Distance to active waypoint
	alt_error	float	m	Current altitude error
	aspd_error	float	m/s	Current airspeed error
	xtrack_error	float	m	Current crosstrack error on x-y plane
	"""
    reference = (
        msg.nav_roll,
        msg.nav_pitch,
        msg.nav_bearing,
        msg.alt_error,
        msg.aspd_error,
        msg.xtrack_error,
    )
    # print "\nRF_Roll\tRF_Pitch\tRF_Head\tRF_Alt\tRF_Spd\tRF_XY"
    # print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % reference

    # print("\nRoll error:%f, pitch error:%f, heading error:%f\n" %(abs(msg.nav_roll - current_roll), abs(msg.nav_pitch - current_pitch), abs(msg.nav_bearing - current_heading)))

    if current_flight_mode != "FLIP":
        alt_error += msg.alt_error
        roll_error += abs(msg.nav_roll - current_roll)
        pitch_error += abs(msg.nav_pitch - current_pitch)
        heading_error += abs(msg.nav_bearing - current_heading)
        stable_counter += 1

        # Compute moving average of each state
        alt_series += current_altitude + msg.alt_error
        roll_series += msg.nav_roll
        pitch_series += msg.nav_pitch
        heading_series += msg.nav_bearing

    elif current_flight_mode == "FLIP":
        flip_start_time = timeit.default_timer()

        roll_series_flip += msg.nav_roll
        pitch_series_flip += msg.nav_pitch
        heading_series_flip += msg.nav_bearing
        stable_counter_flip += 1


# print("Desired heading:%f, current heading:%f" %(msg.nav_bearing, current_heading))


# ------------------------------------------------------------------------------------
def handle_param(msg):
    global target_param
    global target_param_ready
    global target_param_value

    message = msg.to_dict()
    if message["param_id"] == target_param:
        target_param_ready = 1
        target_param_value = message["param_value"]
    else:
        target_param_ready = 0


# ------------------------------------------------------------------------------------
def handle_position(msg):
    position_data = (msg.lat, msg.lon)

    # print(msg)
    # print("lat:%f, lon:%f, vertical speed (cm/s):%d" %(float(msg.lat), float(msg.lon), int(msg.vz)))

    global position_cnt
    global lat_series
    global lon_series
    global lat_current
    global lat_previous
    global lon_current
    global lon_previous
    global vertical_speed
    global relative_alt

    position_cnt += 1
    lat_series += msg.lat
    lon_series += msg.lon

    lat_previous = lat_current
    lon_previous = lon_current
    lat_current = msg.lat
    lon_current = msg.lon

    vertical_speed = msg.vz
    relative_alt = msg.relative_alt / 1000  # convert it from mm to meters


# ------------------------------------------------------------------------------------
def handle_status(msg):
    global Parachute_on
    global GPS_status
    global Accel_status
    global Gyro_status
    global PreArm_error
    global Baro_status
    global failsafe_error
    global hit_ground
    global RC_failsafe_error
    global takeoff
    global mission_cnt

    status_data = (msg.severity, msg.text)
    log(("[status_text] %s" % msg.text))

    # Detecting a depolyed parachute
    if "Parachute: Released" in msg.text:
        Parachute_on = 1

    # Detecting an error on GPS
    elif "NavEKF" in msg.text and "lane switch" in msg.text:
        GPS_status = 0

    # Detecting an error on gyro sensor
    elif "Vibration compensation ON" in msg.text:
        Gyro_status = 0

    # Detecting an error on accelerometer sensor
    elif "EKF primary changed" in msg.text:
        Accel_status = 0

    # Detecting error messages related to a barometer sensor
    elif "PreArm: Waiting for Nav Checks" in msg.text:
        Baro_status = 0

    # Detecting a PreArm check error
    elif "PreArm: Check" in msg.text:
        PreArm_error = 1

    # Detecting a failsafe logic
    elif ("EKF Failsafe" in msg.text) and ("EKF Failsafe Cleared" not in msg.text):
        failsafe_error = 1

    # Detecting a failsafe logic clear
    elif "EKF Failsafe Cleared" in msg.text:
        failsafe_error = 0

    # Detecting landing on the ground
    elif "SIM Hit ground at" in msg.text:
        hit_ground = 1

    # Detecting RC failsafe
    elif ("Radio Failsafe" in msg.text) and ("Radio Failsafe Cleared" not in msg.text):
        RC_failsafe_error = 1

    # Detecting RC failsafe clear
    elif "Radio Failsafe Cleared" in msg.text:
        RC_failsafe_error = 0

    # Detecting takeoff
    elif "Got COMMAND_ACK: NAV_TAKEOFF: ACCEPTED" in msg.text:
        takeoff = 1

    # Detecting an uploaded mission
    elif "Got MISSION_ACK" in msg.text:
        mission_cnt = 1


# ------------------------------------------------------------------------------------
def handle_time(msg):
    global system_time

    # time_data = (msg.time_unix_usec, msg.time_boot_ms)
    system_time = msg.time_unix_usec


# print "[system_time] UNIX time:%d, Boot time:%d" % time_data


# ------------------------------------------------------------------------------------
def handle_mission(msg):
    global mission_cnt

    mission_cnt = msg.count
    log(("[Debug][MISSION_COUNT]%d" % mission_cnt))


# ------------------------------------------------------------------------------------
def handle_gps(msg):
    global num_GPS
    global alt_GPS_series
    global gps_message_cnt

    gps_message_cnt += 1
    num_GPS = int(msg.satellites_visible)
    alt_GPS_series += int(msg.alt / 1000)  # convert mm unit to meter unit


# print("# of GPS:%d, GPS altitude:%d"%(num_GPS, alt_GPS))
# ------------------------------------------------------------------------------------
def handle_circle_status(msg):
    # Positive values orbit clockwise, negative values orbit counter-clockwise.
    ## https://mavlink.io/en/messages/common.html#ORBIT_EXECUTION_STATUS
    global circle_radius_current
    global circle_radius_previous

    circle_radius_previous = circle_radius_current
    circle_radius_current = msg.radius

    # print("[Debug-circle mode] radius:%f, X:%d, Y:%d, Z:%f" % (msg.radius, msg.lon, msg.x, msg.y, msg.z))


# ------------------------------------------------------------------------------------
def read_loop():
    while True:
        monitor_conn = mavutil.mavlink_connection("udp:127.0.0.1:14551")
        # while mavlink_pause_event.is_set():
        #     log("Pausing reading loop for 10 seconds")
        #     time.sleep(10)
        # # current types
        # types_msg = [
        #     "BAD_DATA",
        #     "RC_CHANNELS",
        #     "VFR_HUD",
        #     "ATTITUDE",
        #     "NAV_CONTROLLER_OUTPUT",
        #     "GLOBAL_POSITION_INT",
        #     "STATUSTEXT",
        #     "SYSTEM_TIME",
        #     "MISSION_COUNT",
        #     "PARAM_VALUE",
        #     "GPS_RAW_INT",
        #     "HEARTBEAT",
        #     "ORBIT_EXECUTION_STATUS",
        # ]
        # # grab a mavlink message
        try:
            msg = monitor_conn.recv_match(blocking=True)
        except Exception as e:
            print("An exception occurred:", str(e))
            exit(0)

        # handle the message based on its type
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        elif msg_type == "RC_CHANNELS":
            handle_rc_raw(msg)
        elif msg_type == "VFR_HUD":
            handle_hud(msg)
        elif msg_type == "ATTITUDE":
            handle_attitude(msg)
        elif msg_type == "NAV_CONTROLLER_OUTPUT":
            handle_target(msg)
        elif msg_type == "GLOBAL_POSITION_INT":
            handle_position(msg)
        elif msg_type == "STATUSTEXT":
            handle_status(msg)
        elif msg_type == "SYSTEM_TIME":
            handle_time(msg)
        elif msg_type == "MISSION_COUNT":
            handle_mission(msg)
        elif msg_type == "PARAM_VALUE":
            handle_param(msg)
        elif msg_type == "GPS_RAW_INT":
            handle_gps(msg)
        elif msg_type == "HEARTBEAT":
            handle_heartbeat(msg)
        elif msg_type == "ORBIT_EXECUTION_STATUS":
            handle_circle_status(msg)


# --------------------- (End) READ Robotic Vehicle's states ------------------------
# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
def store_mutated_inputs():
    global Policy_violation_cnt
    global count_main_loop
    Policy_violation_cnt += 1

    log("***************Policy violation!***************")

    f1 = open("mutated_log.txt", "r")
    lines = f1.readlines()

    # Store the mutated inputs as a txt file
    # './policies/chute/*.txt'
    file_name = ""
    file_name += "./policy_violations/"
    file_name += str(Policy_violation_cnt)
    file_name += ".txt"

    try:
        f2 = open(file_name, "w")
    except IOError:
        # Create the directory
        os.mkdir("./policy_violations/")
        log("Create the policy_violations dir as not found")
        f2 = open(file_name, "w")
    f2.writelines(lines)
    f1.close()
    f2.close()

    mutated_log = open("mutated_log.txt", "w")
    mutated_log.close()

    log("Restarting the vehicle : Policy violation logged")
    re_launch()
    count_main_loop = 0


# ------------------------------------------------------------------------------------
def print_distance(G_dist, P_dist, length, policy, guid):
    # Print distances
    # print(
    #     (
    #         "#---------------------------------%s-------------------------------------------"
    #         % policy
    #     )
    # )
    log("[Distance] ")
    for i in range(length):
        log("P%d: %f " % (i + 1, P_dist[i]))

    # print("")
    log(("[Distance] Global distance: %f" % Global_distance))
    # print(
    #     "#-----------------------------------------------------------------------------"
    # )

    if G_dist < 0:
        log("The value of distance is low")
        store_mutated_inputs()

    global Current_policy_P_length
    global Previous_distance

    if policy == Current_policy:
        # Previous distances
        if guid == "false":
            for i in range(Current_policy_P_length):
                Previous_distance[i] = P_dist[i]

        elif guid == "true":
            global Current_input
            global Current_input_val

            for i in range(Current_policy_P_length):
                log_flag = -1
                guide_line = ""
                fp = open("guidance_log.txt", "r")

                if Previous_distance[i] < P_dist[i]:
                    log_flag = 0
                    # a) Checking whether there is already stored same input pair
                    for line in fp.readlines():
                        guide_line += line

                        if Current_input in line:
                            row = line.rstrip().split(" ")
                            if int(row[2]) == i + 1:
                                log_flag = 1
                                log(
                                    (
                                        "[Redundant input] {} {} {} {}".format(
                                            row[0], row[1], row[2], row[3]
                                        )
                                    )
                                )
                                log(
                                    (
                                        "[Redundant input] old:%f - new:%f"
                                        % (
                                            float(row[3]),
                                            (P_dist[i] - Previous_distance[i]),
                                        )
                                    )
                                )
                                if float(row[3]) <= (P_dist[i] - Previous_distance[i]):
                                    log_flag = 2
                                    print_input = ""
                                    print_input += Current_input  # 0
                                    print_input += " "
                                    print_input += Current_input_val  # 1
                                    print_input += " "
                                    print_input += str(i + 1)  # 2
                                    print_input += " "
                                    print_input += str(
                                        P_dist[i] - Previous_distance[i]
                                    )  # 3
                                    print_input += "\n"
                                    guide_line = guide_line.replace(line, print_input)
                                    log(
                                        (
                                            "[Redundant input] we need to log %s because it increase more propositional distance %d"
                                            % (Current_input, i + 1)
                                        )
                                    )
                # Append a new input
                if log_flag == 0:
                    log(
                        (
                            "[*Distance*] propositional distance %d is increased (input: %s, %s)"
                            % (i + 1, Current_input, Current_input_val)
                        )
                    )

                    print_input = ""
                    print_input += Current_input
                    print_input += " "
                    print_input += Current_input_val
                    print_input += " "
                    print_input += str(i + 1)
                    print_input += " "
                    print_input += str(P_dist[i] - Previous_distance[i])
                    print_input += "\n"
                    fp.close()
                    write_guidance_log(print_input, action="append")

                elif log_flag == 1 or log_flag == -1:
                    fp.close()

                elif log_flag == 2:
                    fp.close()
                    write_guidance_log(guide_line, action="write")


# ------------------------------------------------------------------------------------
# ---------------(Start) Calculate propositional and global distances-----------------
def calculate_distance(guidance):
    # State global variables
    global alt_series
    global roll_series
    global pitch_series
    global heading_series
    global stable_counter

    global roll_series_flip
    global pitch_series_flip
    global heading_series_flip
    global stable_counter_flip

    global roll_initial
    global pitch_initial
    global yaw_initial
    global ground_speed

    global flip_start_time

    global current_roll
    global current_pitch
    global current_heading

    global lat_series
    global lon_series
    global position_cnt
    global home_lat
    global home_lon
    global lat_current
    global lat_previous
    global lon_current
    global lon_previous

    global current_alt
    global previous_alt
    global Previous_distance
    global actual_throttle
    global vertical_speed
    global relative_alt

    global num_GPS
    global alt_GPS_series
    global gps_message_cnt
    global failsafe_error
    global RC_failsafe_error
    global takeoff

    global circle_radius_current
    global circle_radius_previous

    global current_rc_1
    global current_rc_2
    global current_rc_3
    global current_rc_4

    global rollspeed_current
    global rollspeed_previous
    global pitchspeed_current
    global pitchspeed_previous
    global yawspeed_current
    global yawspeed_previous

    global yaw_min
    global yaw_max
    global roll_min
    global roll_max
    global pitch_min
    global pitch_max

    global PRINT_DEBUG

    if stable_counter > 0:
        alt_avg = alt_series / stable_counter
        roll_avg = roll_series / stable_counter
        pitch_avg = pitch_series / stable_counter
        heading_avg = heading_series / stable_counter
    else:
        alt_avg = 0
        roll_avg = 0
        pitch_avg = 0
        heading_avg = 0

    if stable_counter_flip > 0:
        roll_avg_flip = roll_series_flip / stable_counter_flip
        pitch_avg_flip = pitch_series_flip / stable_counter_flip
        heading_avg_flip = heading_series_flip / stable_counter_flip
    else:
        roll_avg_flip = 0
        pitch_avg_flip = 0
        heading_avg_flip = 0

    if position_cnt > 0:
        lat_avg = lat_series / position_cnt
        lon_avg = lon_series / position_cnt

    else:
        lat_avg = 0
        lon_avg = 0

    lat_avg = lat_avg / 1000
    lat_avg = lat_avg * 1000
    lon_avg = lon_avg / 1000
    lon_avg = lon_avg * 1000

    previous_alt = current_alt
    current_alt = math.ceil(alt_avg)

    # Calculate a relative altitude from the home postion
    current_alt = current_alt - home_altitude

    if gps_message_cnt > 0:
        alt_GPS_avg = alt_GPS_series / gps_message_cnt
    else:
        alt_GPS_avg = alt_GPS_series

    if PRINT_DEBUG == 1:
        # print('[Debug] stable_counter:%d' %stable_counter)
        # print('[Debug] alt_avg:%f (previous_alt:%f, current_alt:%f), roll_avg:%f, pitch_avg:%f, heading_avg:%f' %(alt_avg, previous_alt, current_alt, roll_avg, pitch_avg, heading_avg))
        log(
            (
                "[Debug] lat_avg:%f, home_lat:%f, lon_avg:%f, home_lon:%f"
                % (lat_avg, home_lat, lon_avg, home_lon)
            )
        )

    position_cnt = 0
    stable_counter = 0
    stable_counter_flip = 0
    alt_series = 0
    roll_series = 0
    pitch_series = 0
    heading_series = 0
    lat_series = 0
    lon_series = 0
    alt_GPS_series = 0
    gps_message_cnt = 0

    global target_param
    global target_param_ready
    global target_param_value

    global Parachute_on
    global Armed
    global current_flight_mode
    global previous_flight_mode
    global previous_altitude
    global current_altitude
    global P
    global Global_distance
    global mission_cnt

    target_param_ready = 0

    current_alt_round = round(current_altitude, 1)
    previous_alt_round = round(previous_altitude, 1)

    # When the drone is in the air, we can assume that the drone already finished the takeoff stage.
    if relative_alt > 0:
        takeoff = 1

    # # ----------------------- (start) A.CHUTE1 policy -----------------------
    # # Propositional distances
    # # 0: turn off, 1: turn on
    #
    # # P1
    # if Parachute_on == 1:
    #     P[0] = 1
    # else:
    #     P[0] = -1
    # # P2
    # if Armed == 0:
    #     P[1] = 1
    # else:
    #     P[1] = -1
    # # P3
    # if current_flight_mode == "FLIP" or current_flight_mode == "ACRO":
    #     P[2] = 1
    # else:
    #     P[2] = -1
    #
    # # P4
    # if current_alt > 0:
    #     P[3] = (current_alt - previous_alt) / current_alt
    # else:
    #     P[3] = 0
    #
    # # P5
    # # Request parameter
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "CHUTE_ALT_MIN", -1
    # )
    #
    # target_param = "CHUTE_ALT_MIN"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # if target_param_value > 0:
    #     P[4] = (target_param_value - current_alt) / target_param_value
    # else:
    #     P[4] = 0
    #
    # Global_distance = -1 * (min(P[0], max(P[1], P[2], P[3], P[4])))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=5, policy="A.CHUTE", guid=guidance
    # )
    #
    # # ----------------------- (end) A.CHUTE1 policy -----------------------
    #
    # target_param_ready = 0
    # # ----------------------- (start) A.RTL1 policy -----------------------
    # # P1
    # if lat_avg == home_lat and lon_avg == home_lon:
    #     P[0] = -1
    # else:
    #     P[0] = 1
    #
    # # P2
    # # Request parameter
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "RTL_ALT", -1
    # )
    #
    # target_param = "RTL_ALT"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # # Convert centimeters to meters
    # target_param_value = target_param_value / 100
    #
    # if target_param_value > 0:
    #     P[1] = (target_param_value - current_alt) / target_param_value
    # else:
    #     P[1] = 0
    #
    # if PRINT_DEBUG == 1:
    #     log(("[Debug] RTL_ALT:%f, current_alt:%f" % (target_param_value, current_alt)))
    #
    # if current_flight_mode == "RTL":
    #     P[2] = 1
    # else:
    #     P[2] = -1
    #
    # if previous_alt != 0:
    #     P[3] = (previous_alt - current_alt) / previous_alt
    # else:
    #     P[3] = 0
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2], P[3]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=4, policy="A.RTL1", guid=guidance
    # )
    # # ----------------------- (end) A.RTL1 policy -----------------------
    #
    # # ----------------------- (start) A.RTL2 policy -----------------------
    # # P0: Mode_t = RTL
    # if current_flight_mode == "RTL":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: (ALT_t - RTL_ALT)/ALT_t
    # # Request parameter
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "RTL_ALT", -1
    # )
    #
    # target_param = "RTL_ALT"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # # Convert centimeters to meters
    # target_param_value = target_param_value / 100
    #
    # if target_param_value > 0:
    #     P[1] = (current_alt - target_param_value) / current_alt
    # else:
    #     P[1] = 0
    #
    # if PRINT_DEBUG == 1:
    #     log(("[Debug] RTL_ALT:%f, current_alt:%f" % (target_param_value, current_alt)))
    #
    # # P2: POS_t != Home_position
    # if lat_avg != home_lat and lon_avg != home_lon:
    #     P[2] = 1
    # else:
    #     P[2] = -1
    #
    # # P3: POS_(t-1) != POS_(t)
    # if lat_current != lat_previous or lon_current != lon_previous:
    #     P[3] = -1
    # else:
    #     P[3] = 1
    #
    # # P4: ALT_(t-1) = ALT_t
    # if previous_alt != 0 and previous_alt == current_alt:
    #     P[4] = -1
    # else:
    #     P[4] = 1
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2], max(P[3], P[4])))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=5, policy="A.RTL2", guid=guidance
    # )
    # # ----------------------- (end) A.RTL2 policy -----------------------
    #
    # # ----------------------- (start) A.RTL3 policy -----------------------
    # # P0: Mode_t = RTL
    # if current_flight_mode == "RTL":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: (ALT_t - RTL_ALT)/ALT_t
    # # Request parameter
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "RTL_ALT", -1
    # )
    #
    # target_param = "RTL_ALT"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # # Convert centimeters to meters
    # target_param_value = target_param_value / 100
    #
    # if target_param_value > 0:
    #     P[1] = (current_alt - target_param_value) / current_alt
    # else:
    #     P[1] = 0
    #
    # # P2: POS_t != Home_position
    # if lat_avg == home_lat and lon_avg == home_lon:
    #     P[2] = 1
    # else:
    #     P[2] = -1
    #
    # # P3: (ALT_t - ALT_(t-1)) / ALT_t
    # if previous_alt_round != 0 and current_alt_round != 0:
    #     P[3] = (current_alt_round - previous_alt_round) / current_alt_round
    # else:
    #     P[3] = 0
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         ("[Debug] ALT_t:%f, ALT_(t-1):%f" % (previous_alt_round, current_alt_round))
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2], P[3]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=4, policy="A.RTL3", guid=guidance
    # )
    # # ----------------------- (end) A.RTL3 policy -----------------------
    #
    # # ----------------------- (start) A.RTL4 policy -----------------------
    # # P0: Mode_(t-1) = RTL ^ Mode_t = LAND
    # if previous_flight_mode == "RTL" and current_flight_mode == "LAND":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: ALT_t = Ground_ALT
    # if round(current_altitude, 1) == round(home_altitude, 1):
    #     P[1] = 1
    # else:
    #     P[1] = 0
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] ALT_t:%f, Ground_ALT:%f"
    #             % (round(current_altitude, 1), round(home_altitude, 1))
    #         )
    #     )
    #
    # # P2: Disarm = on
    # if Armed == 0:
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=3, policy="A.RTL4", guid=guidance
    # )
    # # ----------------------- (end) A.RTL4 policy -----------------------
    #
    # # ----------------------- (start) A.FLIP1 policy -----------------------
    # # P0: Mode_t = FLIP
    # if current_flight_mode == "FLIP":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: Mode_(t-1) = ACRO/ALT_HOLD
    # if previous_flight_mode == "ACRO" or previous_flight_mode == "ALT_HOLD":
    #     P[1] = -1
    # else:
    #     P[1] = 1
    #
    # # P2: Roll_t <= 45
    # if abs(roll_avg) <= 45:
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # # P3: Throttle >= 1,500 (Yet, we can infer the actual throttle from current and previous altitudes)
    # if round(current_altitude, 0) >= round(previous_altitude, 0):
    #     P[3] = -1
    # else:
    #     P[3] = 1
    #
    # # P4: ALT_t >= 10
    # if current_alt >= 10:
    #     P[4] = -1
    # else:
    #     P[4] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] roll_avg:%f, ALT_t:%f, ALT_(t-1):%f, current_alt:%f"
    #             % (
    #                 roll_avg,
    #                 round(current_altitude, 0),
    #                 round(previous_altitude, 0),
    #                 current_alt,
    #             )
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], max(P[1], P[2], P[3], P[4])))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=5, policy="A.FLIP1", guid=guidance
    # )
    # # ----------------------- (end) A.FLIP1 policy -----------------------
    #
    # # ----------------------- (start) A.FLIP2 policy -----------------------
    # # P0: Mode_t = FLIP
    # if current_flight_mode == "FLIP":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: -90 <= Roll_t <= 45
    # if -90 <= abs(roll_avg_flip) <= 45:
    #     P[1] = 1
    # else:
    #     P[1] = -1
    #
    #     # P2: Roll_rate = 400
    #     P[
    #         2
    #     ] = (
    #         -1
    #     )  # We cannot monitor the roll rate; thus, we assume that it's always true.
    #
    # # P3: Roll_Direction = Right
    # if abs(roll_avg_flip) > 0:
    #     P[3] = -1
    # else:
    #     P[3] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(("[Debug] roll_avg_flip:%f" % roll_avg_flip))
    #
    # Global_distance = -1 * (min(P[0], P[1], max(P[2], P[3])))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=4, policy="A.FLIP2", guid=guidance
    # )
    # # ----------------------- (end) A.FLIP2 policy -----------------------
    #
    # # ----------------------- (start) A.FLIP3 policy -----------------------
    # # P0: Mode_t = FLIP
    # if previous_flight_mode == "FLIP":
    #     P[0] = 1
    #
    #     if round(roll_initial, 0) == round(current_roll, 0):
    #         P[1] = -1
    #     else:
    #         P[1] = 1
    #
    #     if round(pitch_initial, 0) == round(current_pitch, 0):
    #         P[2] = -1
    #     else:
    #         P[2] = 1
    #
    #     if (yaw_initial - current_heading) < 20:
    #         P[3] = -1
    #     else:
    #         P[3] = 1
    #
    # else:
    #     P[0] = -1
    #     P[1] = 1
    #     P[2] = 1
    #     P[3] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] roll_initial:%f, roll_current:%f"
    #             % (round(roll_initial, 0), round(current_roll, 0))
    #         )
    #     )
    #     log(
    #         (
    #             "[Debug] pitch_initial:%f, pitch_current:%f"
    #             % (round(pitch_initial, 0), round(current_pitch, 0))
    #         )
    #     )
    #     log(
    #         (
    #             "[Debug] yaw_initial:%f, yaw_current:%f"
    #             % (round(yaw_initial, 0), round(current_heading, 0))
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], max(P[1], P[2], P[3])))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=4, policy="A.FLIP3", guid=guidance
    # )
    #
    # roll_initial = 0.0
    # pitch_initial = 0.0
    # yaw_initial = 0.0
    #
    # # ----------------------- (end) A.FLIP3 policy -----------------------
    #
    # # ----------------------- (start) A.FLIP4 policy -----------------------
    # elapsed = 0.0
    # # P0: Mode_t = FLIP
    # if previous_flight_mode == "FLIP":
    #     P[0] = 1
    #
    #     elapsed = timeit.default_timer() - flip_start_time
    #
    #     # P1: Mode_t != FLIP (within 2.5 seconds)
    #     if elapsed <= 2.5:
    #         P[1] = -1
    #     else:
    #         P[1] = 1
    #
    # else:
    #     P[0] = -1
    #     P[1] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] flip_start_time:%f, flip_end_time:%f, elapsed:%f"
    #             % (flip_start_time, timeit.default_timer(), elapsed)
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=2, policy="A.FLIP4", guid=guidance
    # )
    #
    # flip_start_time = 0.0
    #
    # # ----------------------- (end) A.FLIP4 policy -----------------------
    #
    # # ----------------------- (start) A.ALT_HOLD1 policy -----------------------
    # # P0: ALT_src = Baro
    # ## EK2_ALT_SOURCE - 0: Baro, 1: Range finder, 2: GPS, 3: Range Beacon
    #
    # # Request parameter
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "EK2_ALT_SOURCE", -1
    # )
    #
    # target_param = "EK2_ALT_SOURCE"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # # Convert centimeters to meters
    # alt_source = int(target_param_value)
    #
    # if alt_source == 0:
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: ALT_t = ALT_baro
    # # P2: ALT_t != ALT_GPS
    # if round(alt_avg, 1) != round(alt_GPS_avg, 1):
    #     P[1] = -1
    #     P[2] = -1
    # else:
    #     P[1] = 1
    #     P[2] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] alt_source:%d, ALT_t:%f, ALT_GPS:%f"
    #             % (alt_source, round(alt_avg, 1), round(alt_GPS_avg, 1))
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], max(P[1], P[2])))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=3, policy="A.ALT_HOLD1", guid=guidance
    # )
    # # ----------------------- (end) A.ALT_HOLD1 policy -----------------------
    #
    # # ----------------------- (start) A.ALT_HOLD2 policy -----------------------
    # # P0: Mode_t = ALT_HOLD
    # if current_flight_mode == "ALT_HOLD":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: Throttle_t = 1,500
    # if actual_throttle == 34:
    #     P[1] = 1
    # else:
    #     P[1] = -1
    #
    # # P2: ALT_t = ALT_(t-1)
    # if round(current_altitude, 0) == round(previous_altitude, 0):
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] actual_throttle:%d, ALT_t:%f, ALT_(t-1):%f"
    #             % (
    #                 actual_throttle,
    #                 round(current_altitude, 0),
    #                 round(previous_altitude, 0),
    #             )
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=3, policy="A.ALT_HOLD2", guid=guidance
    # )
    # # ----------------------- (end) A.ALT_HOLD2 policy -----------------------
    #
    # # ----------------------- (start) A.CIRCLE1 policy -----------------------
    # # P0: Mode_t = CIRCLE
    # if current_flight_mode == "CIRCLE":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: RC_pitch < 1,500
    # ## (1,500 - RC_pitch) / 1,500
    # ## current_rc_2: RC_pitch
    #
    # if current_rc_2 == 1500:
    #     P[1] = 0
    # else:
    #     P[1] = (1500 - current_rc_2) / 1500
    #
    # # P2: Circle_radius_t > 0
    # if circle_radius_current > 0:
    #     P[2] = 1
    # else:
    #     P[2] = -1
    #
    # # P3: Circle_radius_t < Circle_radius_(t-1)
    # ## ( Circle_radius_(t-1) - Circle_radius_t ) / Circle_radius_(t-1)
    #
    # if (
    #     circle_radius_previous == 0
    #     or (circle_radius_previous - circle_radius_current) == 0
    # ):
    #     P[3] = 0
    # else:
    #     P[3] = (circle_radius_previous - circle_radius_current) / circle_radius_previous
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] RC_pitch:%d, circle_radius_t:%f, circle_radius_(t-1):%f"
    #             % (current_rc_2, circle_radius_current, circle_radius_previous)
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2], P[3]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=4, policy="A.CIRCLE1", guid=guidance
    # )
    # # ----------------------- (end) A.CIRCLE1 policy -----------------------
    #
    # # ----------------------- (start) A.CIRCLE2 policy -----------------------
    # # P0: Mode_t = CIRCLE
    # if current_flight_mode == "CIRCLE":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: RC_pitch > 1,500
    # ## (RC_pitch - 1,500) / RC_pitch
    # ## current_rc_2: RC_pitch
    #
    # if current_rc_2 == 0 or current_rc_2 == 1500:
    #     P[1] = 0
    # else:
    #     P[1] = (current_rc_2 - 1500) / current_rc_2
    #
    # # P2: Circle_radius_t > Circle_radius_(t-1)
    # ## ( Circle_radius_(t-1) - Circle_radius_t ) / Circle_radius_(t-1)
    #
    # if (
    #     circle_radius_previous == 0
    #     or (circle_radius_previous - circle_radius_current) == 0
    # ):
    #     P[2] = 0
    # else:
    #     P[2] = (circle_radius_previous - circle_radius_current) / circle_radius_previous
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] RC_pitch:%d, circle_radius_t:%f, circle_radius_(t-1):%f"
    #             % (current_rc_2, circle_radius_current, circle_radius_previous)
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=3, policy="A.CIRCLE2", guid=guidance
    # )
    # # ----------------------- (end) A.CIRCLE2 policy -----------------------
    #
    # # ----------------------- (start) A.CIRCLE3 policy -----------------------
    # # P0: Mode_t = CIRCLE
    # if current_flight_mode == "CIRCLE":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: RC_roll > 1,500
    # ## (RC_roll - 1,500) / RC_proll
    # ## current_rc_1: RC_roll
    #
    # if current_rc_1 == 0 or current_rc_1 == 1500:
    #     P[1] = 0
    # else:
    #     P[1] = (current_rc_1 - 1500) / current_rc_1
    #
    # # P2: circle_detection_t = clockwise
    # ## circle_radius_current > 0: clockwise, circle_radius_current < 0: counter-clockwise
    # if circle_radius_current > 0:
    #     P[2] = 1
    # else:
    #     P[2] = -1
    #
    # # P3: Circle_speed_t > Circle_speed_(t-1)
    # ## There is no direct way to measure the circle speed; thus, let's assume that it's aways hold.
    # P[3] = -1
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2], P[3]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=4, policy="A.CIRCLE3", guid=guidance
    # )
    # # ----------------------- (end) A.CIRCLE3 policy -----------------------
    #
    # # ----------------------- (start) A.CIRCLE4-6 policy -----------------------
    # # P0: Mode_t = CIRCLE
    # if current_flight_mode == "CIRCLE":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: RC_roll < 1,500
    # ## (1,500 - RC_roll) / 1,500
    # ## current_rc_1: RC_roll
    #
    # if (1500 - current_rc_1) == 0:
    #     P[1] = 0
    # else:
    #     P[1] = (1500 - current_rc_1) / 1500
    #
    # # P2: circle_detection_t = counter-clockwise
    # ## circle_radius_current > 0: clockwise, circle_radius_current < 0: counter-clockwise
    # if circle_radius_current < 0:
    #     P[2] = 1
    # else:
    #     P[2] = -1
    #
    # # P3: Circle_speed_t < Circle_speed_(t-1)
    # ## There is no direct way to measure the circle speed; thus, let's assume that it's aways hold.
    # P[3] = -1
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2], P[3]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=4, policy="A.CIRCLE4-6", guid=guidance
    # )
    # # ----------------------- (end) A.CIRCLE4-6 policy -----------------------
    #
    # rollspeed_current_raw = rollspeed_current
    # rollspeed_previous_raw = rollspeed_previous
    # pitchspeed_current_raw = pitchspeed_current
    # pitchspeed_previous_raw = pitchspeed_previous
    # yawspeed_current_raw = yawspeed_current
    # yawspeed_previous_raw = yawspeed_previous
    #
    # rollspeed_current = round(rollspeed_current, 0)
    # rollspeed_previous = round(rollspeed_previous, 0)
    # pitchspeed_current = round(pitchspeed_current, 0)
    # pitchspeed_previous = round(pitchspeed_previous, 0)
    # yawspeed_current = round(yawspeed_current, 0)
    # yawspeed_previous = round(yawspeed_previous, 0)
    # # ----------------------- (start) A.CIRCLE7 policy -----------------------
    # # P0: Mode_t = CIRCLE
    # if current_flight_mode == "CIRCLE":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: RC_roll_speed_t = RC_roll_speed_t-1
    # ## RC_roll_speed_t: rollspeed_current, RC_roll_speed_t-1: rollspeed_previous
    #
    # if rollspeed_current == rollspeed_previous:
    #     P[1] = -1
    # else:
    #     P[1] = 1
    #
    # # P2: RC_pitch_speed_t = RC_pitch_speed_t-1
    # ## RC_pitch_speed_t: pitchspeed_current, RC_pitch_speed_t-1: pitchspeed_previous
    #
    # if pitchspeed_current == pitchspeed_previous:
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # # P3: RC_yaw_speed_t = RC_yaw_speed_t-1
    # ## RC_yaw_speed_t: yawspeed_current, RC_yaw_speed_t-1: yawspeed_previous
    #
    # if yawspeed_current == yawspeed_previous:
    #     P[3] = -1
    # else:
    #     P[3] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] Roll_speed_t:%f, Roll_speed_t-1:%f"
    #             % (rollspeed_current, rollspeed_previous)
    #         )
    #     )
    #     log(
    #         (
    #             "[Debug] Pitch_speed_t:%f, Pitch_speed_t-1:%f"
    #             % (pitchspeed_current, pitchspeed_previous)
    #         )
    #     )
    #     log(
    #         (
    #             "[Debug] Yaw_speed_t:%f, Yaw_speed_t-1:%f"
    #             % (yawspeed_current, yawspeed_previous)
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], max(P[1], P[2], P[3])))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=4, policy="A.CIRCLE7", guid=guidance
    # )
    # # ----------------------- (end) A.CIRCLE7 policy -----------------------
    #
    # # ----------------------- (start) A.LAND1 policy -----------------------
    # # P0: Mode_t = LAND
    # if current_flight_mode == "LAND":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: ALT_t >= 10
    # ## (ALT_t - 10) / ALT_t
    # ## Use 'relative_alt': relative altitude from the ground
    # if relative_alt == 0:
    #     P[1] = -1  # Preventing false alarms when the drone lands on the ground
    # else:
    #     P[1] = (relative_alt - 10) / relative_alt
    #
    # # P2: Speed_vertical_t = LAND_SPEED_HIGH
    # expected_landing_speed = 0
    # ## Request parameter
    # target_param_ready = 0
    # target_param_value = 0
    #
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "LAND_SPEED_HIGH", -1
    # )
    #
    # target_param = "LAND_SPEED_HIGH"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # # If 'LAND_SPEED_HIGH' configuration parameter is zero then WPNAV_SPEED_DN is used.
    # if target_param_value == 0:
    #     ## Request parameter
    #     target_param_ready = 0
    #
    #     master.mav.param_request_read_send(
    #         master.target_system, master.target_component, "WPNAV_SPEED_DN", -1
    #     )
    #
    #     target_param = "WPNAV_SPEED_DN"
    #     count = 0
    #     while target_param_ready == 0 and count < 5:
    #         time.sleep(1)
    #         count += 1
    #
    # expected_landing_speed = target_param_value
    #
    # # The drone's actual landing speed cannot be exactly matched with the parameter value; thus, we leverage the following predicate.
    # if abs(vertical_speed - expected_landing_speed) <= 10:
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] ALT_t:%f, vertical_speed:%d, expected_vertical_speed:%d"
    #             % (relative_alt, vertical_speed, expected_landing_speed)
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=3, policy="A.LAND1", guid=guidance
    # )
    # # ----------------------- (end) A.LAND1 policy -----------------------
    #
    # # ----------------------- (start) A.LAND2 policy -----------------------
    # # P0: Mode_t = LAND
    # if current_flight_mode == "LAND":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: ALT_t < 10
    # ## (10 - ALT_t) / 10
    # ## Use 'relative_alt': relative altitude from the ground
    # if relative_alt == 10:
    #     P[1] = 0
    # elif relative_alt == 0:
    #     P[1] = -1  # Preventing false alarms when the drone lands on the ground
    # else:
    #     P[1] = (10 - relative_alt) / 10
    #
    # # P2: Speed_vertical_t = LAND_SPEED
    # expected_landing_speed = 0
    # ## Request parameter
    # target_param_ready = 0
    # target_param_value = 0
    #
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "LAND_SPEED", -1
    # )
    #
    # target_param = "LAND_SPEED"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # expected_landing_speed = target_param_value
    #
    # # The drone's actual landing speed cannot be exactly matched with the parameter value; thus, we leverage the following predicate.
    # if abs(vertical_speed - expected_landing_speed) <= 10:
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] ALT_t:%f, vertical_speed:%d, expected_vertical_speed:%d"
    #             % (relative_alt, vertical_speed, expected_landing_speed)
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=3, policy="A.LAND2", guid=guidance
    # )
    # # ----------------------- (end) A.LAND2 policy -----------------------
    #
    # # ----------------------- (start) A.AUTO1 policy -----------------------
    # # P0: Mode_t = AUTO
    # if current_flight_mode == "AUTO":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: RC_yaw_t != 1500
    # if current_rc_4 != 1500:
    #     P[1] = 1
    # else:
    #     P[1] = -1
    #
    # # P2: Yaw_t != Yaw_(t-1)
    # if round(yawspeed_current_raw, 2) != round(yawspeed_previous_raw, 2):
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] RC_yaw_t:%f, Yaw_t:%f, Yaw_(t-1):%f"
    #             % (
    #                 current_rc_4,
    #                 round(yawspeed_current_raw, 2),
    #                 round(yawspeed_previous_raw, 2),
    #             )
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=3, policy="A.AUTO1", guid=guidance
    # )
    # # ----------------------- (end) A.AUTO1 policy -----------------------
    #
    # global gps_failsafe_cnt
    # global brake_cnt
    # # ----------------------- (start) A.GPS.FS1 policy -----------------------
    # # P0: Mode_t = AUTO, AUTOTUNE, BRAKE, CIRCLE, DRIFT, FOLLOW, GUIDED, LOITER, POSHOLD, RTL, Simple, SMART_RTL, THROW, ZIGZAG
    # ## When the drone's flight mode leverages a GPS receiver
    # if (
    #     (current_flight_mode == "AUTO")
    #     or (current_flight_mode == "AUTOTUNE")
    #     or (current_flight_mode == "BRAKE")
    #     or (current_flight_mode == "CIRCLE")
    #     or (current_flight_mode == "DRIFT")
    #     or (current_flight_mode == "FOLLOW")
    #     or (current_flight_mode == "GUIDED")
    #     or (current_flight_mode == "LOITER")
    #     or (current_flight_mode == "POSHOLD")
    #     or (current_flight_mode == "RTL")
    #     or (current_flight_mode == "SIMPLE")
    #     or (current_flight_mode == "SMART_RTL")
    #     or (current_flight_mode == "THROW")
    #     or (current_flight_mode == "ZIGZAG")
    # ):
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: GPS_count < 4
    # ## (4 - GPS_count) / 4
    # if (num_GPS - 4) == 0:
    #     P[1] = 0
    # else:
    #     P[1] = float((4 - num_GPS) / 4.0)
    #
    # # P2: GPS_failsafe = on
    # if failsafe_error == 1:
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(("[Debug] GPS_count:%d, GPS_failsafe:%d" % (num_GPS, failsafe_error)))
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # # Triggering failsafe requires specific time;
    # # thus, we give two more chance before the control software trigger the failsafe
    # if (Global_distance < 0) and (gps_failsafe_cnt < 2):
    #     gps_failsafe_cnt = gps_failsafe_cnt + 1
    #     Global_distance = -1 * Global_distance
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=3, policy="A.GPS.FS1", guid=guidance
    # )
    # # ----------------------- (end) A.GPS.FS1 policy -----------------------
    #
    # # ----------------------- (start) A.GPS.FS2 policy -----------------------
    # # P0: GPS_failsafe = on
    # if failsafe_error == 1:
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: Baro = on
    # target_param_ready = 0
    # target_param_value = 0
    #
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "SIM_BARO_DISABLE", -1
    # )
    #
    # target_param = "SIM_BARO_DISABLE"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # # If 'SIM_BARO_DISABLE' configuration parameter is zero then a barometer sensor is activated and used.
    # if target_param_value == 0:
    #     P[1] = 1
    # else:
    #     P[1] = -1
    #
    # # P2: ALT_src = Baro
    # if round(alt_avg, 1) != round(alt_GPS_avg, 1):
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] GPS_failsafe:%d, SIM_BARO_DISABLE:%d, ALT_baro:%f, ALT_GPS:%f"
    #             % (
    #                 failsafe_error,
    #                 target_param_value,
    #                 round(alt_avg, 1),
    #                 round(alt_GPS_avg, 1),
    #             )
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=3, policy="A.GPS.FS2", guid=guidance
    # )
    # # ----------------------- (end) A.GPS.FS2 policy -----------------------
    #
    # # ----------------------- (start) A.RC.FS1 policy -----------------------
    # # P0: Takeoff != on
    # if takeoff != 1:
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # target_param_ready = 0
    # target_param_value = 0
    #
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "FS_THR_VALUE", -1
    # )
    #
    # target_param = "FS_THR_VALUE"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # fs_thr_val = target_param_value
    # # P1: Throttle_t < FS_THR_VALUE
    # ## (FS_THR_VALUE - Throttle_t) / FS_THR_VALUE
    # if (fs_thr_val == 0) or (fs_thr_val - current_rc_3) == 0:
    #     P[1] = 0
    # else:
    #     P[1] = float((fs_thr_val - current_rc_3) / fs_thr_val)
    #
    # # P2: Disarm = on
    # if Armed == 0:
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(("[Debug] Takeoff:%d, Armed:%d" % (takeoff, Armed)))
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=2, policy="A.RC.FS1", guid=guidance
    # )
    # # ----------------------- (end) A.RC.FS1 policy -----------------------
    #
    # # ----------------------- (start) A.RC.FS2 policy -----------------------
    # target_param_ready = 0
    # target_param_value = 0
    #
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "FS_THR_VALUE", -1
    # )
    #
    # target_param = "FS_THR_VALUE"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # fs_thr_val = target_param_value
    # # P0: Throttle_t < FS_THR_VALUE
    # ## (FS_THR_VALUE - Throttle_t) / FS_THR_VALUE
    # if (fs_thr_val == 0) or (fs_thr_val - current_rc_3) == 0:
    #     P[0] = 0
    # else:
    #     P[0] = float((fs_thr_val - current_rc_3) / fs_thr_val)
    #
    # # P1: RC_fail = on
    # if RC_failsafe_error == 1:
    #     P[1] = -1
    # else:
    #     P[1] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] Throttle_t:%d, FS_THR_VALUE:%d, RC_failsafe:%d"
    #             % (current_rc_3, fs_thr_val, RC_failsafe_error)
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=2, policy="A.RC.FS2", guid=guidance
    # )
    # # ----------------------- (end) A.RC.FS2 policy -----------------------
    #
    # # ----------------------- (start) A.SPORT1 policy -----------------------
    # # P0: Mode_t = SPORT
    # if current_flight_mode == "SPORT":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # target_param_ready = 0
    # target_param_value = 0
    #
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "PILOT_SPEED_UP", -1
    # )
    #
    # target_param = "PILOT_SPEED_UP"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # pilot_speed_vertical = target_param_value
    #
    # # P1: Vertical_speed_t = PILOT_SPEED_UP
    # if abs(vertical_speed - pilot_speed_vertical) <= 10:
    #     P[1] = -1
    # else:
    #     P[1] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] Mode_t:%s, vertical_speed:%f, PILOT_SPEED_UP:%d"
    #             % (current_flight_mode, vertical_speed, pilot_speed_vertical)
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=2, policy="A.SPORT1", guid=guidance
    # )
    # # ----------------------- (end) A.SPORT1 policy -----------------------
    #
    # # ----------------------- (start) A.GUIDED1 policy -----------------------
    # # P0: Mode_t = GUIDED
    # if current_flight_mode == "GUIDED":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: waypoint_count = 0
    # if mission_cnt == 0:
    #     P[1] = 1
    # else:
    #     P[1] = -1
    #
    # # P3: Yaw_t = Yaw_(t-1)
    # if round(yawspeed_current, 1) == round(yawspeed_previous, 1):
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # # P3: POS_t = POS_(t-1)
    # if round(ground_speed, 0) == 0:
    #     P[3] = -1
    # else:
    #     P[3] = 1
    #
    # # P4: ALT_t = ALT_(t-1)
    # if round(current_altitude, 0) == round(previous_altitude, 0):
    #     P[4] = -1
    # else:
    #     P[4] = 1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] Mode_t:%s, waypoint_count:%f, Yaw_t:%f, Yaw_(t-1):%f, ground speed:%f, ALT_t:%f, ALT_(t-1):%f"
    #             % (
    #                 current_flight_mode,
    #                 mission_cnt,
    #                 round(yawspeed_current, 1),
    #                 round(yawspeed_previous, 1),
    #                 round(ground_speed, 0),
    #                 round(current_altitude, 0),
    #                 round(previous_altitude, 0),
    #             )
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], max(P[2], P[3], P[4])))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=5, policy="A.GUIDED1", guid=guidance
    # )
    # # ----------------------- (end) A.GUIDED1 policy -----------------------
    #
    # # ----------------------- (start) A.LOITER1 policy -----------------------
    # # P0: Mode_t = LOITER
    # # log("Yaw (C):{0} (P):{1}".format(yawspeed_current, yawspeed_previous))
    # # log("Attitude (C):{0} (P):{1}".format(current_altitude, previous_altitude))
    # if current_flight_mode == "LOITER":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: Yaw_t = Yaw_(t-1)
    # if round(yawspeed_current, 1) == round(yawspeed_previous, 1):
    #     P[1] = -1
    # else:
    #     P[1] = 1
    #
    # # P2: POS_t = POS_(t-1)
    # # XXX: 2024-05-29T18:49:45+0000: silipwn: Is this a good assumption?
    # # Would have to def change this based on what Hyungsub mentioned in slack
    # if round(ground_speed, 0) == 0:
    #     P[2] = -1
    # else:
    #     P[2] = 1
    #
    # # P3: ALT_t = ALT_(t-1)
    # if round(current_altitude, 0) == round(previous_altitude, 0):
    #     P[3] = -1
    # else:
    #     P[3] = 1
    #
    # # if PRINT_DEBUG == 1:
    # log(
    #     (
    #         "[Debug] Mode_t:%s, Yaw_t:%f, Yaw_(t-1):%f, ground speed:%f, ALT_t:%f, ALT_(t-1):%f"
    #         % (
    #             current_flight_mode,
    #             yawspeed_current,
    #             yawspeed_previous,
    #             ground_speed,
    #             current_altitude,
    #             previous_altitude,
    #         )
    #     )
    # )
    #
    # Global_distance = -1 * (min(P[0], max(P[1], P[2], P[3])))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=4, policy="A.LOITER1", guid=guidance
    # )
    # ----------------------- (end) A.LOITER1 policy -----------------------

    # ----------------------- (start) A.RANGEFINDER policy -----------------------
    # Adjust Yaw
    if yawspeed_current > yaw_max:
        yaw_max = yawspeed_current
    if yawspeed_current < yaw_min:
        yaw_min = yawspeed_current
    # Adjust Roll
    if rollspeed_current > roll_max:
        roll_max = rollspeed_current
    if rollspeed_current < roll_min:
        roll_min = rollspeed_current
    # Adjust Pitch
    if pitchspeed_current > pitch_max:
        pitch_max = pitchspeed_current
    if pitchspeed_current < pitch_min:
        pitch_min = pitchspeed_current

    if current_flight_mode == "LOITER":  # Cause the bug is found in this mode
        P[0] = 1
    else:
        P[0] = -1
    # $Max(Roll)-Min(Roll) > 1  \lor Max(Yaw)-Min(Yaw) > 1 \lor Max(Pitch)-Min(Pitch) > 1$
    if (roll_max - roll_min) > 2:
        P[1] = 1
    else:
        P[1] = -1
    if (yaw_max - yaw_min) > 2:
        P[2] = 1
    else:
        P[2] = -1
    if (pitch_max - pitch_min) > 2:
        P[3] = 1
    else:
        P[3] = -1

    Global_distance = -1 * (min(P[0], max(P[1], P[3])))

    print_distance(
        G_dist=Global_distance,
        P_dist=P,
        length=3,
        policy="A.RANGEFINDER",
        guid=guidance,
    )

    log(
        (
            "[RANGEFINDER] Mode_t:%s, yaw_max:%f, yaw_min:%f, roll_max: %f, roll_min:%f, pitch_max:%f, pitch_min: %f"
            % (
                current_flight_mode,
                yaw_max,
                yaw_min,
                roll_max,
                roll_min,
                pitch_max,
                pitch_min,
            )
        )
    )
    log("P values %s" % P)
    # ----------------------- (end) A.RANGEFINDER policy -----------------------

    # ----------------------- (start) A.DRIFT1 policy -----------------------
    # P0: GPS_failsafe = on
    # if failsafe_error == 1:
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: Mode_t = DRIFT
    # if previous_flight_mode == "DRIFT":
    #     P[1] = 1
    # else:
    #     P[1] = -1
    #
    # # P2: Mode_t = FS_EKF_ACTION
    # target_param_ready = 0
    # target_param_value = 0
    #
    # master.mav.param_request_read_send(
    #     master.target_system, master.target_component, "FS_EKF_ACTION", -1
    # )
    #
    # target_param = "FS_EKF_ACTION"
    # count = 0
    # while target_param_ready == 0 and count < 5:
    #     time.sleep(1)
    #     count += 1
    #
    # expected_flight_mode_from_FS = target_param_value
    # # 1 or 3: LAND, 2: ALT_HOLD
    # if (expected_flight_mode_from_FS == 1) or (expected_flight_mode_from_FS == 3):
    #     if current_flight_mode == "LAND":
    #         P[2] = -1
    #     else:
    #         P[2] = 1
    # elif expected_flight_mode_from_FS == 2:
    #     if current_flight_mode == "ALT_HOLD":
    #         P[2] = -1
    #     else:
    #         P[2] = 1
    # # When FS_EKF_ACTION parameter's value is incorrectly assigned.
    # else:
    #     P[2] = -1
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] GPS_failsafe:%d, Mode_(t-1):%s, Mode_t:%s, FS_EKF_ACTION:%d"
    #             % (
    #                 failsafe_error,
    #                 previous_flight_mode,
    #                 current_flight_mode,
    #                 expected_flight_mode_from_FS,
    #             )
    #         )
    #     )
    #
    # Global_distance = -1 * (min(P[0], P[1], P[2]))
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=3, policy="A.DRIFT1", guid=guidance
    # )
    # # ----------------------- (end) A.DRIFT1 policy -----------------------
    #
    # # ----------------------- (start) A.BRAKE1 policy -----------------------
    # # P0: Mode_t = BRAKE
    # if previous_flight_mode == "BRAKE":
    #     P[0] = 1
    # else:
    #     P[0] = -1
    #
    # # P1: [0, k] POS_t = POS_(t-1)
    # # Stopping the drone's acceleration requires specific time according to the drone's current speed.
    # # thus, we give three more chances before checking this predicate.
    #
    # if round(ground_speed, 0) == 0:
    #     P[1] = -1
    # else:
    #     P[1] = 1
    #
    # Global_distance = -1 * (min(P[0], P[1]))
    #
    # if (
    #     (previous_flight_mode == "BRAKE")
    #     and (round(ground_speed, 0) != 0)
    #     and (brake_cnt < 2)
    # ):
    #     brake_cnt = brake_cnt + 1
    #     Global_distance = -1 * Global_distance
    #
    # if PRINT_DEBUG == 1:
    #     log(
    #         (
    #             "[Debug] Mode_t:%s, ground_speed:%f"
    #             % (current_flight_mode, round(ground_speed, 0))
    #         )
    #     )
    #
    # print_distance(
    #     G_dist=Global_distance, P_dist=P, length=2, policy="A.BRAKE1", guid=guidance
    # )
    # ----------------------- (end) A.BRAKE1 policy -----------------------

    target_param_ready = 0
    target_param = ""
    target_param_value = 0


# ------------------------------------------------------------------------------------
# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(id, mav_conn, pwm=1500):
    """Set RC channel pwm value
    Args:
        id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if id < 1:
        log("Channel does not exist.")
        return

    # We only have 8 channels
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm

        # global master

        mav_conn.mav.rc_channels_override_send(
            mav_conn.target_system,  # target_system
            mav_conn.target_component,  # target_component
            *rc_channel_values,
        )  # RC channel list, in microseconds.


# ------------------------------------------------------------------------------------
def throttle_th():
    global goal_throttle

    mav_conn = mavutil.mavlink_connection("127.0.0.1:14551")
    mav_conn.recv_match(type="HEARTBEAT", blocking=True)
    while True:
        set_rc_channel_pwm(3, mav_conn, goal_throttle)
        time.sleep(0.2)


# ------------------------------------------------------------------------------------
def match_cmd(cmd):
    cmds = []
    f = open("guidance_log.txt", "r")
    for line in f.readlines():
        if cmd in line:
            cmds.append(line)

    f.close()

    if len(cmds) >= 2:
        index = random.randint(0, len(cmds) - 1)
        row = cmds[index].rstrip().split(" ")
        # print("*****")
        # log(("[Matched input] {} {} {} {}".format(row[0], row[1], row[2], row[3])))
        # print("*****")

        return row[1]

    elif len(cmds) == 1:
        row = cmds[0].rstrip().split(" ")

        # print("*****")
        # log(("[Matched input] {} {} {} {}".format(row[0], row[1], row[2], row[3])))
        # print("*****")

        return row[1]

    return "null"


# ------------------------------------------------------------------------------------
def execute_cmd(num):
    global Current_input
    global Current_input_val
    global Guidance_decision
    rand = []

    mav_conn = mavutil.mavlink_connection("127.0.0.1:14551")
    # Each user command contains 7 parameters. We assign random values to these parameters.
    for i in range(7):
        rand.append(random.randint(1, 100))

    # To do: Implement all if statements for all user commands

    Current_input = read_inputs.cmd_name[num]

    if Guidance_decision == True:
        Current_input_val = match_cmd(cmd=Current_input)

    if Current_input_val != "null":
        log(
            (
                "@@@[Reuse stored input pair] (%s, %s)@@@"
                % (Current_input, Current_input_val)
            )
        )

    # ------------------------(start) execute a selected command-------------------------
    if (
        read_inputs.cmd_name[num] == "RC1"
        or read_inputs.cmd_name[num] == "RC2"
        or read_inputs.cmd_name[num] == "RC4"
    ):
        target_RC = 0
        if Current_input_val == "null":
            mutated_value = random.randint(1200, 1900)
            Current_input_val = str(mutated_value)
        else:
            mutated_value = int(Current_input_val)

        if read_inputs.cmd_name[num] == "RC1":
            target_RC = 1
        elif read_inputs.cmd_name[num] == "RC2":
            target_RC = 2
        elif read_inputs.cmd_name[num] == "RC4":
            target_RC = 4

        set_rc_channel_pwm(target_RC, mav_conn, mutated_value)

    elif read_inputs.cmd_name[num] == "RC3":
        global goal_throttle

        if Current_input_val == "null":
            goal_throttle = random.randint(1000, 2000)
            Current_input_val = str(goal_throttle)
        else:
            goal_throttle = int(Current_input_val)

    elif read_inputs.cmd_name[num] == "Flight_Mode":
        # Triggering a proper flight mode makes PGFUZZ quickly testing each policy
        Current_input_val = read_inputs.cmd_number[num]

        if Current_input_val == "null":
            rand_flight_mode = random.choice(mav_conn.mode_mapping().items())
            log("Rand flight mode " + rand_flight_mode)
            Current_input_val = str(rand_flight_mode[1])
            rand_fligh_mode = rand_flight_mode[1]
        else:
            rand_fligh_mode = int(Current_input_val)

        # master.mav.set_mode_send(
        #     master.target_system,
        #     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        #     rand_fligh_mode,
        # )
        #
        # mode_id = master.mode_mapping()[
        #     rand_fligh_mode
        # ]  # FIXME: Double check if this works
        mav_conn.set_mode(rand_fligh_mode)

    elif read_inputs.cmd_name[num] == "MAV_CMD_DO_PARACHUTE":
        Current_input_val = "2"

        mav_conn.mav.command_long_send(
            mav_conn.target_system,  # target_system
            mav_conn.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            0,
            2,
            0,
            0,
            0,
            0,
            0,
            0,
        )
    else:
        if Current_input_val != "null" and "," in Current_input_val:
            row = Current_input_val.rstrip().split(",")
            for i in range(7):
                rand[i] = int(row[i])

        Current_input_val = str(rand[0])
        Current_input_val += ","
        Current_input_val += str(rand[1])
        Current_input_val += ","
        Current_input_val += str(rand[2])
        Current_input_val += ","
        Current_input_val += str(rand[3])
        Current_input_val += ","
        Current_input_val += str(rand[4])
        Current_input_val += ","
        Current_input_val += str(rand[5])
        Current_input_val += ","
        Current_input_val += str(rand[6])

        mav_conn.mav.command_long_send(
            mav_conn.target_system,  # target_system
            mav_conn.target_component,  # target_component
            int(read_inputs.cmd_number[num]),
            0,
            rand[0],
            rand[1],
            rand[2],
            rand[3],
            rand[4],
            rand[5],
            rand[6],
        )
    # ------------------------(end) execute a selected command-------------------------

    log(("[Execute_cmd] (%s, %s)" % (Current_input, Current_input_val)))

    # Log executed the user command
    print_cmd = ""
    print_cmd += "C "
    print_cmd += Current_input
    print_cmd += " "
    print_cmd += Current_input_val
    print_cmd += "\n"
    write_log(print_cmd)

    mav_conn.close()


# ------------------------------------------------------------------------------------
def execute_env(num):
    global Current_input
    global Current_input_val
    global Guidance_decision

    # To do: implement all if statements for all user commands

    Current_input = read_inputs.env_name[num]

    mav_conn = mavutil.mavlink_connection("127.0.0.1:14551")
    if Guidance_decision == True:
        Current_input_val = match_cmd(cmd=Current_input)

    if Current_input_val == "null":
        rand = random.uniform(0, 100)
        Current_input_val = str(rand)
    else:
        log(
            (
                "@@@[Reuse stored input pair] (%s, %s)@@@"
                % (Current_input, Current_input_val)
            )
        )

    mav_conn.mav.param_set_send(
        mav_conn.target_system,
        mav_conn.target_component,
        Current_input,
        float(Current_input_val),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    log(("[Execute_env] (%s, %s)" % (Current_input, Current_input_val)))

    # Log executed the environmental factor
    print_env = ""
    print_env += "E "
    print_env += Current_input
    print_env += " "
    print_env += Current_input_val
    print_env += "\n"
    write_log(print_env)


# ------------------------------------------------------------------------------------
def pick_up_cmd():
    global Current_input
    global Current_input_val
    global Guidance_decision
    global RV_alive

    RV_alive = 1

    Current_input = ""
    Current_input_val = "null"
    Guidance_decision = None

    # a) Randomly select a type of inputs ( 1)user command, 2)parameter, 3)environmental factor)
    # input_type = random.randint(1, 4)
    input_type = random.choice(
        [1, 4]
    )  # XXX: Remove always set to 4, also enable the Flight_Mode to ensure LOITER

    # Hyungsub - to test user commands! I need to remove the below code after finishing to implement all user commands
    # input_type = 1

    # True: input mutated from guidance, False: randomly mutate an input
    Guidance_decision = random.choice([True, False])

    # b) Randomly select an input from the selected type of inputs

    # 1) User commands
    if input_type == 1:
        execute_cmd(num=random.randint(0, len(read_inputs.cmd_name) - 1))

    # 2) Parameters
    elif input_type == 2:
        change_parameter(
            selected_param=random.randint(0, len(read_inputs.param_name) - 1)
        )

    # 3) Environmental factors
    elif input_type == 3:
        execute_env(num=random.randint(0, len(read_inputs.env_name) - 1))

    # 4) Add RangeFinder
    elif input_type == 4:
        randomize_msg_rangefinder()


# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
def main(argv):
    global Precondition_path
    global home_altitude
    global drone_status
    global executing_commands
    global current_altitude
    global Armed
    global Parachute_on
    global count_main_loop
    global goal_throttle
    global failsafe_error
    global RV_alive
    global hit_ground

    ardupilot_dir = os.getenv("ARDUPILOT_HOME")

    # Get git commit in ardupilot_dir
    # Very bad programming practice, but it is a quick solution
    current_commit = subprocess.check_output(
        "git rev-parse HEAD", shell=True, cwd=ardupilot_dir
    ).strip()
    if current_commit == "":
        log("No commit found in the Ardupilot directory")
    else:
        log("The commit being tested is: %s" % current_commit)

    log('Pymavlink version %s'% pymavlink.__version__)

    # ------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------
    # Parsing parameters
    # Update the below path according to a changed target policy
    f_path_def = ""
    f_path_def += "./policies/"
    f_path_def += Current_policy

    # print(
    #     "#-----------------------------------------------------------------------------"
    # )
    params_path = ""
    params_path += f_path_def
    params_path += "/parameters.txt"
    read_inputs.parsing_parameter(params_path)
    log(
        (
            "# Check whether parsing parameters well done or not, received # of params: %d"
            % len(read_inputs.param_name)
        )
    )
    # log((read_inputs.param_name)) XXX: Enable the parameter printing via debug flag

    cmd_path = ""
    cmd_path += f_path_def
    cmd_path += "/cmds.txt"

    read_inputs.parsing_command(cmd_path)
    log(
        (
            "# Check whether parsing user commands well done or not, received # of params: %d"
            % len(read_inputs.cmd_name)
        )
    )
    # log((read_inputs.cmd_name)) XXX: Enable the parameter printing via debug flag

    env_path = ""
    env_path += f_path_def
    env_path += "/envs.txt"

    read_inputs.parsing_env(env_path)
    log(
        (
            "# Check whether parsing environmental factors well done or not, received # of params: %d"
            % len(read_inputs.env_name)
        )
    )
    # log((read_inputs.env_name)) XXX: Enable the parameter printing via debug flag
    # log(
    #     "#-----------------------------------------------------------------------------"
    # )
    mav_conn = mavutil.mavlink_connection("127.0.0.1:14551")

    mav_conn.wait_heartbeat()

    # request data to be sent at the given rate
    for i in range(0, 3):
        mav_conn.mav.request_data_stream_send(
            mav_conn.target_system,
            mav_conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            6,
            1,
        )

    message = mav_conn.recv_match(type="VFR_HUD", blocking=True)
    home_altitude = message.alt
    log(("home_altitude: %f" % home_altitude))

    message = mav_conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    home_lat = message.lat
    home_lat = home_lat / 1000
    home_lat = home_lat * 1000
    home_lon = message.lon
    home_lon = home_lon / 1000
    home_lon = home_lon * 1000
    log(("home_lat: %f, home_lon: %f" % (home_lat, home_lon)))

    # TODO: 2024-05-30T12:15:41-0400: silipwn: See if the approach is scalable
    # for every scenario To ensure that we have full setup finished wait till
    # we get a LOCAL_POSITION_NED
    # _ = mav_conn.recv_match(type="LOCAL_POSITION_NED", blocking=True)
    # log("Got the local position ned")

    # Start the Rangefinder thread
    # Set some preconditions to test a policy
    # When I switch to another target policy, I need to update the 'Precondition_path'.
    Precondition_path += "./policies/"
    Precondition_path += Current_policy
    Precondition_path += "/preconditions.txt"
    # set_preconditions(Precondition_path)
    # reboot_vehicle()

    while True:
        msg = mav_conn.recv_match(type="STATUSTEXT", blocking=True)
        if "is using GPS" in msg.text:
            log("Got GPS usage message")
            break

    t4 = multiprocessing.Process(target=send_msg_rangefinder)
    t4.daemon = True
    t4.start()

    time.sleep(10)  # TODO: Figure out the ideal time to wait

    # Testing
    # ------------------------------------------------------------------------
    for i in range(30):
        P.append(0)
        Previous_distance.append(0)

    # Choose a mode
    mode = "GUIDED"

    # Check if mode is available
    if mode not in mav_conn.mode_mapping():
        log(("Unknown mode : {}".format(mode)))
        log(("Try:", list(mav_conn.mode_mapping().keys())))
        exit(1)

    # Get mode ID
    mode_id = mav_conn.mode_mapping()[mode]

    # master.mav.set_mode_send( master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
    # )
    mav_conn.mav.command_long_send(
        mav_conn.target_system,
        mav_conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
        0,
        0,
        0,
        0,
        0,
    )
    # mav_conn.set_mode(mode_id)

    # Wait for ACK command
    # TODO: Figure out why we keep missing command_acks randomly
    ack_msg = mav_conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=10)
    # Check if command in the same in `set_mode`
    if ack_msg is None:
        # Let's check if the mode is set via Heartbeat
        hb_msg = mav_conn.recv_match(
            type="HEARTBEAT", blocking=True
        )  # XXX: Hoping this doesn't get stuck
        hb_msg = hb_msg.to_dict()
        if hb_msg["custom_mode"] != mode_id:
            log("Failed set to guided mode")
            log("Exiting")
            exit(0)
    else:
        ack_msg = ack_msg.to_dict()
        if ack_msg["command"] == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            # Print the ACK result !
            log((mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description))
        else:
            log("Failed set to guided mode")
            log(ack_msg)

    mav_conn.mav.command_long_send(
        mav_conn.target_system,
        mav_conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    while True:
        # Wait for ACK command
        ack_msg = mav_conn.recv_match(type="COMMAND_ACK", blocking=True)
        if ack_msg is None:
            log("Arming failed, exiting")
            exit(0)
        ack_msg = ack_msg.to_dict()

        log((mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description))
        break

    time.sleep(1)

    mav_conn.mav.command_long_send(
        mav_conn.target_system,  # target_system
        mav_conn.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
        0,  # confirmation
        0,  # param1
        0,  # param2
        0,  # param3
        0,  # param4
        0,  # param5
        0,  # param6
        MISSION_ATTITUDE,  # param7- altitude
    )

    ack = False
    while not ack:
        # Wait for ACK command
        ack_msg = mav_conn.recv_match(type="COMMAND_ACK", blocking=True)
        if ack_msg is None:
            log("Takeoff failed, exiting")
            exit(0)
        ack_msg = ack_msg.to_dict()

        log((mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description))
        break

    # Wait till we reach that height
    while True:
        msg = mav_conn.recv_match(type=["GLOBAL_POSITION_INT"], blocking=True)
        if msg is not None:
            altitude = msg.relative_alt / 1000.0  # Altitude in meters
            if altitude >= MISSION_ATTITUDE:
                log("Reached approximate height")
                break

    # 2024-07-04T11:15:22-0400: silipwn:  TODO: Fix ALT_HOLD mechanism later?
    #     mode_id = mav_conn.mode_mapping()["ALT_HOLD"]
    # master.mav.set_mode_send(
    #     master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
    # )
    # mav_conn.set_mode(mode_id)
    #
    # while True:
    #     # Wait for ACK command
    #     ack_msg = mav_conn.recv_match(type="COMMAND_ACK", blocking=True)
    #     ack_msg = ack_msg.to_dict()
    #
    #     # Check if command in the same in `set_mode`
    #     if ack_msg["command"] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
    #         continue
    #     log((mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description))
    #     break
    # # Set default throttle
    # set_rc_channel_pwm(3, mav_conn, 1500)
    # log("Setting default throttle")
    #
    # time.sleep(3)
    #
    # # Maintain mid-position of stick on RC controller
    # goal_throttle = 1500
    # t1 = threading.Thread(target=throttle_th, args=())
    # t1.daemon = True
    # t1.start()

    t2 = threading.Thread(target=read_loop, args=())
    t2.daemon = True
    t2.start()

    mutated_log = open("mutated_log.txt", "w")
    mutated_log.close()

    guidance_log = open("guidance_log.txt", "w")
    guidance_log.close()

    # Check liveness of the RV software

    t3 = threading.Thread(target=check_liveness, args=())
    t3.daemon = True
    t3.start()

    time.sleep(30)  # Just waiting to see if we can handle the 0 case
    # Setup a counter to catch stalls
    status_ctr = 0
    prev_status_ctr = 0
    # Main loop
    while True:
        log(
            "[Debug] drone_status:%d prev_status_ctr %d"
            % (drone_status, prev_status_ctr)
        )
        # Store previous status_ctr
        if prev_status_ctr == drone_status:
            status_ctr += 1
            log("Incrementing the status_ctr")
            if status_ctr >= 100 and (
                drone_status != mavutil.mavlink.MAV_STATE_ACTIVE
                and prev_status_ctr != mavutil.mavlink.MAV_STATE_ACTIVE
            ):
                log(
                    "Counter reached 100, check if the status has been stuck there, the value for status is {0}".format(
                        drone_status
                    )
                )
                send_telegram_message(
                    "Got stuck for a long time, check logs for more info"
                )
                raise Exception("Got stuck for a long time, check logs for more info")

        # if RV is still active state
        if drone_status == 4:
            if drone_status != prev_status_ctr:
                status_ctr = 0
                log("Reset status_ctr")
            prev_status_ctr = drone_status
            Armed = 1
            executing_commands = 1
            log(("### Next round (%d) for fuzzing commands. ###" % count_main_loop))
            count_main_loop += 1

            # Calculate propositional and global distances
            calculate_distance(guidance="false")

            pick_up_cmd()

            # Calculate distances to evaluate effect of the executed input
            time.sleep(4)
            calculate_distance(guidance="true")
            goal_throttle = 1500

            for i in range(4):
                set_rc_channel_pwm(i + 1, mav_conn, 1500)

            if Parachute_on == 1:
                Armed = 0
                re_launch()
                count_main_loop = 0

        # The vehicle is grounded
        elif (drone_status == 3 and RV_alive == 1) or (hit_ground == 1):
            log(("[Debug] drone_status:%d" % drone_status))
            if drone_status != prev_status_ctr:
                status_ctr = 0
                log("Reset status_ctr")
            prev_status_ctr = drone_status

            if hit_ground == 1:
                log("[Debug] *the drone hits ground*")

            log(
                (
                    "### Vehicle is grounded, Home alt:%f, Current alt:%f"
                    % (home_altitude, current_altitude)
                )
            )
            Armed = 0
            hit_ground = 0
            re_launch()
            count_main_loop = 0

        elif drone_status == 3:
            # 2024-05-28T16:17:21-0400: silipwn: Basically check if we are
            # critcal error raise ValueError("Unhandled drone status Status: %d
            # Hit_ground: %d PreArm: %d" %
            # (drone_status,hit_ground,PreArm_error))
            log("It is in standby mode")
            log(
                "Drone status Status: %d Hit_ground: %d PreArm: %d"
                % (drone_status, hit_ground, PreArm_error)
            )

            Armed = 0
            re_launch()
            count_main_loop = 0

        # It is in mayday and going down
        elif drone_status == 6:
            Armed = 0
            for i in range(1, 5):
                log(
                    "@@@@@@@@@@ Drone lost control. It is in mayday and going down @@@@@@@@@@"
                )
        elif drone_status == 5:  # Don't care about other variables
            if drone_status != prev_status_ctr:
                status_ctr = 0
            prev_status_ctr = drone_status
            # 2024-05-28T16:17:21-0400: silipwn: Basically check if we are
            # critcal error raise ValueError("Unhandled drone status Status: %d
            # Hit_ground: %d PreArm: %d" %
            # (drone_status,hit_ground,PreArm_error))
            log(
                "Drone status Status: %d Hit_ground: %d PreArm: %d"
                % (drone_status, hit_ground, PreArm_error)
            )
            log("Restarting the vehicle")
            failsafe_error = hit_ground = 0

            re_launch()
            count_main_loop = 0

        elif drone_status == 0:
            log("DANGER! It seems that the drone status is 0")
            log("I'm going to sleep, fix this man")
            send_telegram_message("Now is drone_status 0")
            time.sleep(100)
        else:
            log("Unhandled MAV_STATE, check what is wrong")
            send_telegram_message("Unhandled MAV_STATE, check what is wrong")
            raise Exception("Unhandled MAV_STATE please check what's wrong")

    log("-------------------- Fuzzing End --------------------")


if __name__ == "__main__":
    main(sys.argv[1:])
