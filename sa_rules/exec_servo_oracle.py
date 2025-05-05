#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File            : exec_servo_oracle.py
# Description     : This file takes in 2 sets of 2 files, a BIN file and a callgrind.out file
# Date            : 19.04.2025
# Last Modified   : 2025-05-04T14:54:08-0400
#
import argparse
import os
import re
from scipy.spatial.distance import euclidean
import numpy as np
from fastdtw import fastdtw
from pymavlink import mavutil


def preprocess_mav_data(raw_data, channels=["C1", "C2", "C3", "C4"]):
    """Convert MAVLink packet list to DTW-ready format"""
    return {ch: [pkt[ch] for pkt in raw_data] for ch in channels}


def multivariate_dtw(series1, series2):
    # Convert to numpy arrays for efficient computation
    s1 = np.array([series1[ch] for ch in ["C1", "C2", "C3", "C4"]]).T
    s2 = np.array([series2[ch] for ch in ["C1", "C2", "C3", "C4"]]).T

    # Compute DTW with Euclidean distance
    

def compare_bins(args):
    vehicle_conn_1 = mavutil.mavlink_connection(args.bin_file)
    vehicle_conn_2 = mavutil.mavlink_connection(args.bin_file2)

    # Extract the RCOU values from the BIN files
    rcout_1 = []
    rcout_2 = []
    while True:
        msg = vehicle_conn_1.recv_match(type="RCOU", blocking=True)
        if msg is None:
            break
        else:
            rcout_1.append(msg.to_dict())
    while True:
        msg = vehicle_conn_2.recv_match(type="RCOU", blocking=True)
        if msg is None:
            break
        else:
            rcout_2.append(msg.to_dict())

    # Remove the first 50 samples (takeoff/landing)
    rcout_1 = rcout_1[50:-50]
    rcout_2 = rcout_2[50:-50]

    series_a = preprocess_mav_data(rcout_1)
    series_b = preprocess_mav_data(rcout_2)
    print(len(series_a))
    print(len(series_b))

    distance, path = multivariate_dtw(series_a, series_b)
    # Compare the RCOU values from both array with the given threshold
    # if len(rcout_1) != len(rcout_2):
    #     print("Warn: RCOU arrays are of different lengths.")
    #     print(f"Length of rcout_1: {len(rcout_1)} rcout_2: {len(rcout_2)}")
    #     return deviations

    # Compare the 4 motors with the diff threshold
    # for values in range(0, len(rcout_1)):
    #     for i in range(0, 4):
    #         diff = abs(rcout_1[values][f"C{i+1}"] - rcout_2[values][f"C{i+1}"])
    #         if diff > threshold:
    #             deviations += 1
    #             print(
    #                 f"Deviation found at index {values}: "
    #                 f"C{i+1} {rcout_1[values][f'C{i+1}']} vs "
    #                 f"{rcout_2[values][f'C{i+1}']} (diff: {diff})"
    #             )

    return distance


def compare_callgrind(args):
    file_1 = args.callgrind_file
    file_2 = args.callgrind_file2

    # Open the files and check for the line with "RCOutput::write"
    # And check for the next line which contains the "calls"
    search_term = "RCOutput::write"
    buffer_1 = []
    with open(file_1, "r") as f:
        after = 0
        for line in f:
            if search_term in line:
                buffer_1.append(line.strip())
                after = 2  # Set counter to print next 2 lines
            elif after > 0:
                buffer_1.append(line.strip())
                after -= 1
    match_1 = re.findall(r"calls=(\d+)", buffer_1)
    buffer_2 = []
    with open(file_2, "r") as f:
        after = 0
        for line in f:
            if search_term in line:
                buffer_2.append(line.strip())
                after = 2  # Set counter to print next 2 lines
            elif after > 0:
                buffer_2.append(line.strip())
                after -= 1
    match_2 = re.findall(r"calls=(\d+)", buffer_2)

    print(match_1)
    print(match_2)

    return 100


if __name__ == "__main__":

    args = argparse.ArgumentParser(
        description="Compare two types of files: a BIN file and a callgrind.out file."
    )
    args.add_argument("bin_file", type=str, help="Path to the 1st BIN file")
    # args.add_argument(
    #     "callgrind_file", type=str, help="Path to the 1st callgrind.out file"
    # )
    args.add_argument("bin_file2", type=str, help="Path to the 2nd BIN file")
    # args.add_argument(
    #     "callgrind_file2", type=str, help="Path to the 2nd callgrind.out file"
    # )
    args.add_argument(
        "threshold",
        type=float,
        default=30,
        nargs="?",
        help="Threshold for deviation comparison (default: 30)",
    )

    args = args.parse_args()

    # Sanity check if all files exist
    if (args.bin_file and not os.path.isfile(args.bin_file)) or (
        args.bin_file2 and not os.path.isfile(args.bin_file2)
    ):
        print("One or more files do not exist.")
        exit(1)

    deviations = compare_bins(args)
    print(f"Total deviations found: {deviations}")
    # callgrind = compare_callgrind(args)
