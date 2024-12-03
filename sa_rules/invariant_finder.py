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

def find_consec_invariants(msg_dict: dict,threshold: float) -> dict:
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
    return invariants
    
def parse_tlog(tlog: str) -> dict:
    # Connect to the TLog file
    mavutil_log = mavutil.mavlink_connection(args.tlog)
    # Extract all the messages into a internal dictionary
    while True:
        msg = mavutil_log.recv_msg()
        if msg is None:
            break
        msg = msg.to_dict()
        if msg['mavpackettype'] == 'BAD_DATA':
            logger.warning("Bad data found in Tlog")
            continue
        # Group messages by their type
        if msg['mavpackettype'] in msg_dict:
            msg_dict[msg['mavpackettype']].append(msg)
        else:
            msg_dict[msg['mavpackettype']] = [msg]
    return msg_dict

def find_std_dev(msg_dict: dict,threshold: float) -> dict:
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
                except: 
                    logger.warning(f"{key} is not a float/int in {msg}")
                    invariant = False
                    break
                values.append(data[i][key])
            if invariant:
                std_dev = np.std(values)
                logger.debug("std_dev {} for {}".format(std_dev,key))
                if std_dev == 0.0:
                    if msg in invariants:
                        invariants[msg].append(key)
                    else:
                        invariants[msg] = [key]
    return invariants
    


parser = argparse.ArgumentParser(description='Find invariants in a TLog file')
parser.add_argument('--tlog', type=str, help='TLog file to analyze')
parser.add_argument('--folder', type=str, help='Folder to load Tlogs from to merge')
parser.add_argument('--threshold', type=float, help='Threshold for invariants (Default: 0.9)', default=0.9)
parser.add_argument('--mode',choices=['consec','std_dev'],help='Mode to run in (Default: std_dev)',default='std_dev')
parser.add_argument('--verbose', help='Enable verbose logging', action='store_true')
args = parser.parse_args()

logger.info(f"Checking for invariants with threshold {args.threshold}")

if args.verbose:
    logger.setLevel(logging.DEBUG)
    logger.debug("Verbose logging enabled")


# Check if tlog xor json is provided
if args.tlog is None and args.folder is None:
    logger.critical("Please provide a JSON/Tlog file to load")
    exit(1)

msg_dict = {}

if args.tlog:
    msg_dict = parse_tlog(args.tlog)

elif args.folder:
    # Load all the TLog files from the folder
    merge_dict = {}
    for file in os.listdir(args.folder):
        if file.endswith(".tlog"):
            logger.info(f"Loading {file}")
            msg_dict = parse_tlog(args.folder + "/" + file)
            logger.critical("Merging not implemented yet")
            exit(1)
            # Now track the invariants
            # if args.mode == 'std_dev':
            #     invariants = find_std_dev(msg_dict,args.threshold)
            # else:
            #     invariants = find_consec_invariants(msg_dict,args.threshold)
            # # Merge the invariants
            # for key in invariants:
            #     if key in merge_dict:
            #         merge_dict[key] += invariants[key]
            #     else:
            #         merge_dict[key] = invariants[key]

invariants = {}
if args.mode == 'std_dev':
    invariants = find_std_dev(msg_dict,args.threshold)
else:
    invariants = find_consec_invariants(msg_dict,args.threshold)

# Save the invariants into a JSON file
with open('invariants.json', 'w') as f:
    json.dump(invariants, f, indent=4)
    logger.info("Invariants saved to invariants.json")    