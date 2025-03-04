#!/usr/bin/env python3
# -*- coding:utf-8 -*-
###
# Finis coronat opus; Run this at your own peril ~ silipwn
# File: bin_preprocessor.py
# SPDX-License-Identifier: BSD-3-Clause or GPL-3.0-or-later
# Author: silipwn (contact@as-hw.in)
# Description: For preprocessing binary log files for analysis.
# Date: 2025-02-27T15:37:13-0500
# Last-Modified: 2025-03-03T10:57:41-0500
###
import os, csv, sys
import argparse
from pymavlink import mavutil

def main(args):
    sim_msgs = []
    rc_msgs = []
    filtered_msgs = []
    if not os.path.exists(args.input_file):
        print("Error: Input file does not exist.")
        sys.exit(1)
    logfile = mavutil.mavlink_connection(args.input_file)
    start_time = None
    end_time = None
    while True:
        msg = logfile.recv_match()
        if msg is None:
            break
        if msg.get_type() == "SIM":
            sim_msgs.append(msg.to_dict())
        elif msg.get_type() == "RCOU":
            rc_msgs.append(msg.to_dict())
        elif msg.get_type() == "EV":
            msg = msg.to_dict()
            if msg['Id'] == 15: # Auto armed
                start_time = msg['TimeUS']
                print("Auto armed at:", start_time)
            elif msg['Id'] == 11: # Disarmed 
                end_time = msg['TimeUS'] 
                print("Disarmed at:", end_time) 
    if start_time is None or end_time is None:
        print("Warning : Could not find start and end times for the flight.")
        print(f"Skipping this file {args.input_file}")
        return 
    # Now we can filter the messages
    # filtered_msgs = rc_msgs
    for msg in rc_msgs:
        if start_time <= msg['TimeUS'] <= end_time:
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