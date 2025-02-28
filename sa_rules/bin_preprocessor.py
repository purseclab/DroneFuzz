#!/usr/bin/env python3
# -*- coding:utf-8 -*-
###
# Finis coronat opus; Run this at your own peril ~ silipwn
# File: bin_preprocessor.py
# SPDX-License-Identifier: BSD-3-Clause or GPL-3.0-or-later
# Author: silipwn (contact@as-hw.in)
# Description: For {insert  task}
# Date: 2025-02-27T15:37:13-0500
# Last-Modified: 2025-02-28T13:37:13-0500
###
import os, csv, sys
import argparse
from pymavlink import mavutil

def main(args):
    sim_msgs = []
    status_msgs = []
    rc_msgs = []
    baro_msgs = []
    if not os.path.exists(args.input_file):
        print("Error: Input file does not exist.")
        sys.exit(1)
    logfile = mavutil.mavlink_connection(args.input_file)
    while True:
        msg = logfile.recv_match()
        if msg is None:
            break
        if msg.get_type() == "SIM":
            sim_msgs.append(msg.to_dict())
        elif msg.get_type() == "MSG":
            status_msgs.append(msg.to_dict())
        elif msg.get_type() == "RCOU":
            rc_msgs.append(msg.to_dict())
        elif msg.get_type() == "BARO":
            baro_msgs.append(msg.to_dict())
    in_air_time = None
    down_in_time = None
    for msg in baro_msgs:
        if msg["Alt"] > 45 and in_air_time is None:
            in_air_time = msg['TimeUS']
            print("Found in-air message at time:", in_air_time)
        if msg["Alt"] < 45 and in_air_time is not None:
            down_in_time = msg['TimeUS']
            print("Found down message at time:", down_in_time)
            break
    if in_air_time is None or down_in_time is None:
        print("Error: Could not find arming or disarming messages.")
        sys.exit(1)
    # Now we can filter the messages
    filtered_msgs = []
    for msg in rc_msgs:
        if in_air_time <= msg['TimeUS'] <= down_in_time:
            filtered_msgs.append(msg)
    # Dump the filtered messages to a CSV file
    if args.output_file is None:
        args.output_file = args.input_file.replace(".BIN", ".csv")
    print(f"Writing filtered messages to {args.output_file}")
    with open(args.output_file, 'w', newline='') as csvfile:
        fieldnames = ['time_usec', 'chan1_raw', 'chan2_raw', 'chan3_raw', 'chan4_raw']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for msg in filtered_msgs:
            writer.writerow({
                'time_usec': msg['TimeUS'],
                'chan1_raw': msg['C1'],
                'chan2_raw': msg['C2'],
                'chan3_raw': msg['C3'],
                'chan4_raw': msg['C4']
            })
    print(f"Filtered messages written to {args.output_file}")
    print(f"Total messages processed: {len(rc_msgs)}")
    print(f"Total messages filtered: {len(filtered_msgs)}")
    
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Preprocess binary log files for analysis.")
    parser.add_argument("-i","--input_file", type=str, help="Path to the input binary log file.",required=False)
    parser.add_argument("-I", "--input_dir", type=str, help="Path to the input directory containing binary log files.",required=False)
    parser.add_argument("-o","--output_file", type=str, help="Path to the output CSV file.",required=False)
    # TODO parser.add_argument('-f','--filter', type=str, help="Filter criteria for the log messages.",required=False)
    args = parser.parse_args()
    # Check if input_dir and input_file are both provided
    if args.input_dir and args.input_file:
        print("Error: Please provide either an input file or an input directory, not both.")
        sys.exit(1)
    if not args.input_file and not args.input_dir:
        print("Error: Please provide either an input file or an input directory.")
        sys.exit(1)
    # If input_dir is provided, process all .BIN files in the directory
    if args.input_dir:
        for filename in os.listdir(args.input_dir):
            if filename.endswith(".BIN"):
                args.input_file = os.path.join(args.input_dir, filename)
                args.output_file = os.path.join(args.input_dir, filename.replace(".BIN", "_RCOU.csv"))
                main(args)
    elif args.input_file:
        main(args)