#!/usr/bin/env python3
# -*- coding:utf-8 -*-
###
# Finis coronat opus; Run this at your own peril ~ silipwn
# File: invariant_finder.py
# SPDX-License-Identifier: BSD-3-Clause or GPL-3.0-or-later
# Author: silipwn (contact@as-hw.in)
# Description: For finding invariants in a TLog file
# Currently runs by diffing 2 consecutive messages 
# Usage: python invariant_finder.py --tlog <TLog file>
# Date: 2024-11-25T06:42:59-0500
# Last-Modified: 2024-12-03T16:35:02-0500
###
from pymavlink import mavutil
import argparse
import os
import numpy as np
import json
import logging

logger = logging.getLogger('invariant_finder')
logger.setLevel(logging.INFO)
# Create a custom formatter
format="%(asctime)s | %(levelname)s | %(filename)s:%(lineno)d  | %(message)s"
formatter = logging.Formatter(format)
ch = logging.StreamHandler() 
ch.setFormatter(formatter)
logger.addHandler(ch)

def find_consec_invariants(msg_dict: dict,threshold: float):
    """
    Find invariants in a dictionary of messages, based on where consec values 
    don't change more than the threshold
    """
    for msg in msg_dict:
        data = msg_dict[msg]
        for key in data[0]:
            invariant = True
            for i in range(1, len(data)):
                # Try to convert the value to a float/int if possible, else generate a warning
                try:
                    data[i][key] = float(data[i][key])
                    data[i-1][key] = float(data[i-1][key])
                except: 
                    logger.warning(f": {key} is not a float/int in {msg}")
                    invariant = False
                    break
                if abs(data[i][key] - data[i-1][key]) > threshold:
                    invariant = False
                    break
            if invariant:
                if msg in invariants:
                    invariants[msg].append(key)
                else:
                    invariants[msg] = [key]
    print("Invariants found:")
    value = json.dumps(invariants, indent=4)
    print(value)
    exit(0)

def find_std_dev(msg_dict: dict,threshold: float):
    """
    Find invariants in a dictionary of messages, based on the standard deviation of the values
    """
    for msg in msg_dict:
        data = msg_dict[msg]
        for key in data[0]:
            invariant = True
            values = []
            for i in range(1, len(data)):
                # Try to convert the value to a float/int if possible, else generate a warning
                try:
                    data[i][key] = float(data[i][key])
                    data[i-1][key] = float(data[i-1][key])
                except: 
                    print(f"Warning: {key} is not a float/int in {msg}")
                    invariant = False
                    break
                values.append(data[i][key] - data[i-1][key])
            if invariant:
                std_dev = np.std(values)
                if std_dev < threshold:
                    if msg in invariants:
                        invariants[msg].append(key)
                    else:
                        invariants[msg] = [key]
    print("Invariants found:")
    value = json.dumps(invariants, indent=4)
    print(value)
    exit(0)


parser = argparse.ArgumentParser(description='Find invariants in a TLog file')
parser.add_argument('--tlog', type=str, help='TLog file to analyze')
parser.add_argument('--json', type=str, help='JSON file to load')
parser.add_argument('--threshold', type=float, help='Threshold for invariants (Default: 0.9)', default=0.9)
parser.add_argument('--mode',choices=['consec','std_dev'],help='Mode to run in (Default: std_dev)',default='std_dev')
args = parser.parse_args()

print("Checking for invariants with threshold ", args.threshold)



# Check if tlog xor json is provided
if args.json is None and args.tlog is None:
    print("Please provide a JSON/Tlog file to load")
    exit(1)

msg_dict = {}
if args.tlog:
    # Connect to the TLog file
    mavutil_log = mavutil.mavlink_connection(args.tlog)
    # Extract all the messages into a internal dictionary
    while True:
        msg = mavutil_log.recv_msg()
        if msg is None:
            break
        msg = msg.to_dict()
        if msg['mavpackettype'] == 'BAD_DATA':
            print("Warning: Bad data found in Tlog")
            exit(1)
        # Group messages by their type
        if msg['mavpackettype'] in msg_dict:
            msg_dict[msg['mavpackettype']].append(msg)
        else:
            msg_dict[msg['mavpackettype']] = [msg]
    # Now track the invariants
        

if args.json:
    # Load the JSON file
    with open(args.json, 'r') as f:
        for line in f:
            # Data looks like this 
            # {'meta': {'type': 'HEARTBEAT', 'timestamp': 1730766586.291339}, 'data': {'type': 6, 'autopilot': 8, 'base_mode': 0, 'custom_mode': 0, 'system_status': 0, 'mavlink_version': 3}}
            # We need to extract the 'meta' and 'data' fields
            line = json.loads(line)
            msg = line['meta']['type']
            if msg in msg_dict:
                msg_dict[msg].append(line['data'])
            else:
                msg_dict[msg] = [line['data']]
    

invariants = {}
if args.mode == 'std_dev':
    find_std_dev(msg_dict,args.threshold)
else:
    find_consec_invariants(msg_dict,args.threshold)