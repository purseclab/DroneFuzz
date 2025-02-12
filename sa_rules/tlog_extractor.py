#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File            : tlog_extractor.py
# Description     : This file is responsible for <insert task>
# Date            : 07.02.2025
# Last Modified   : 07.02.2025
import argparse
import os
import pathlib
from pymavlink import mavutil
import csv


def log_parser(file_path: str):
    """
    Accepts a Mavlink Tlog file and then parses and extracts
    SIMSTATE and SERVO_OUTPUT_RAW messages
    """
    tlog_parsed = mavutil.mavlink_connection(file_path)
    sim_state = []
    servo_output = []
    in_air_flag = False
    while True:
        msg = tlog_parsed.recv_match()
        if msg is None:
            break
        msg = msg.to_dict()
        if msg["mavpackettype"] == "STATUSTEXT":
            if "Arming" in msg["text"]:
                print("Motors are armed")
                in_air_flag = True
            if "Disarming" in msg["text"]:
                print("Motors are disarmed")
                in_air_flag = False
        elif msg["mavpackettype"] == "SIMSTATE":
            msg.pop("mavpackettype")
            sim_state.append(msg) if in_air_flag else None
        elif msg["mavpackettype"] == "SERVO_OUTPUT_RAW":
            msg.pop("mavpackettype")
            servo_output.append(msg) if in_air_flag else None
    return (sim_state, servo_output)


def generate_csv(values, output, input_dir):
    """
    Convert the values into a CSV format"
    """
    data_file = os.path.join(input_dir, output)
    data_file = open(data_file, "w")
    csv_writer = csv.writer(data_file)
    init = True
    for msg in values:
        if init:
            # Writing headers of CSV file
            header = msg.keys()
            csv_writer.writerow(header)
            init = False
        # Writing data of CSV file
        csv_writer.writerow(msg.values())
    data_file.close()


if __name__ == "__main__":
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument(
        "-l", "--log", type=str, required=True, help="Path to the log file"
    )
    args = argument_parser.parse_args()
    # Check if the file is valid
    try:
        pathlib.Path(args.log).resolve(strict=True)
    except FileNotFoundError:
        print("File not found")
    sim_state, servo_output = log_parser(args.log)
    # Get the tlog directory
    tlog_directory = os.path.dirname(args.log)
    generate_csv(sim_state, "sim_state.csv", tlog_directory)
    generate_csv(servo_output, "servo_output.csv", tlog_directory)
