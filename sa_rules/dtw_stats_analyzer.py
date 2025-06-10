#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File            : dtw_stats_analyzer.py
# Description     : Analyzes multiple bin files using DTW and calculates statistics
# Date            : 2025-05-18
#
import argparse
import os
import numpy as np
from dtw import dtw
from pymavlink import mavutil
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
from datetime import datetime
import statistics


def get_newest_files(directory, count=11):
    """Get the newest 'count' files from the directory"""
    files = [
        os.path.join(directory, f)
        for f in os.listdir(directory)
        if os.path.isfile(os.path.join(directory, f)) and f.endswith(".BIN")
    ]

    # Sort files by creation time (newest first)
    files.sort(key=lambda x: os.path.getctime(x))

    # Return the newest 'count' files
    return files[:count]


def extract_sim_data(bin_file):
    """Extract SIM data from a bin file"""
    vehicle_conn = mavutil.mavlink_connection(bin_file)
    sim_data = []
    rcou_data = []
    status_data = []

    # First pass: collect all messages
    while True:
        msg = vehicle_conn.recv_match(type=["SIM", "RCOU", "STATUSTEXT"], blocking=True)
        if msg is None:
            break
        else:
            if msg.get_type() == "SIM":
                sim_data.append(msg.to_dict())
            elif msg.get_type() == "RCOU":
                rcou_data.append(msg.to_dict())
            elif msg.get_type() == "STATUSTEXT":
                status_data.append(msg.to_dict())

    # Find takeoff and disarm indices
    takeoff_index = 0
    disarm_index = len(sim_data) - 1

    if status_data:
        # Look for takeoff and disarm messages
        for i, status in enumerate(status_data):
            text = status.get("text", "")
            if "Takeoff" in text or "takeoff" in text:
                # Find the closest SIM message after this status
                status_time = status.get("time_boot_ms", 0)
                for j, sim in enumerate(sim_data):
                    if sim.get("time_boot_ms", 0) >= status_time:
                        takeoff_index = j
                        print(f"Found takeoff at index {takeoff_index}")
                        break

            if "Disarmed" in text or "disarmed" in text or "DISARMED" in text:
                # Find the closest SIM message before this status
                status_time = status.get("time_boot_ms", 0)
                for j in range(len(sim_data) - 1, -1, -1):
                    if sim_data[j].get("time_boot_ms", 0) <= status_time:
                        disarm_index = j
                        print(f"Found disarm at index {disarm_index}")
                        break

    # Trim the data based on takeoff and disarm indices
    if takeoff_index < disarm_index:
        print(f"Trimming data from {takeoff_index} to {disarm_index}")
        sim_data = sim_data[takeoff_index : disarm_index + 1]

        # Also trim RCOU data to match the same time range
        if sim_data and rcou_data:
            start_time = sim_data[0].get("time_boot_ms", 0)
            end_time = sim_data[-1].get("time_boot_ms", 0)

            rcou_data = [
                rcou
                for rcou in rcou_data
                if start_time <= rcou.get("time_boot_ms", 0) <= end_time
            ]

    return sim_data, rcou_data


def preprocess_sim_data(
    raw_data, fields=["Roll", "Pitch", "Yaw", "Alt", "Q1", "Q2", "Q3", "Q4"]
):
    """Convert MAVLink SIM packet list to DTW-ready format"""
    return {field: [pkt[field] for pkt in raw_data] for field in fields}


def preprocess_rcou_data(raw_data, channels=range(1, 9)):
    """Convert MAVLink RCOU packet list to DTW-ready format"""
    result = {}
    for chan in channels:
        # Try both possible key formats
        chan_key = f"C{chan}"
        chan_key_alt = f"chan{chan}"

        if raw_data and chan_key in raw_data[0]:
            result[chan_key] = [pkt[chan_key] for pkt in raw_data]
        elif raw_data and chan_key_alt in raw_data[0]:
            result[chan_key_alt] = [pkt[chan_key_alt] for pkt in raw_data]
    return result


def calculate_dtw_distance(series1, series2, fields=["Roll", "Pitch", "Yaw", "Alt"]):
    """Calculate DTW distance between two series"""
    # Convert to numpy arrays for efficient computation
    s1 = np.array([series1[field] for field in fields if field in series1]).T
    s2 = np.array([series2[field] for field in fields if field in series2]).T

    # Check if we have data to compare
    if s1.size == 0 or s2.size == 0:
        return None

    # Standardize the data
    data_standardized_1 = StandardScaler().fit_transform(s1)
    data_standardized_2 = StandardScaler().fit_transform(s2)

    # Compute DTW with Euclidean distance
    alignment = dtw(
        data_standardized_1,
        data_standardized_2,
        dist_method="euclidean",
        distance_only=True,
    )

    return alignment.normalizedDistance


def analyze_files(
    files, sim_fields=["Q1", "Q2", "Q3", "Q4"], rcou_channels=range(1, 9)
):
    """Analyze multiple files and calculate DTW distances"""
    # Extract and preprocess data from all files
    processed_sim_data = []
    processed_rcou_data = []

    for file in files:
        print(f"Processing {os.path.basename(file)}...")
        sim_data, rcou_data = extract_sim_data(file)

        if rcou_data:
            print(f"Found {len(rcou_data)} RCOU messages after trimming")
            # Print a sample of the first RCOU message to see its structure
            if rcou_data:
                print(f"Sample RCOU keys: {list(rcou_data[0].keys())}")
            processed = preprocess_rcou_data(rcou_data)
            if processed:
                processed_rcou_data.append(processed)
                print(f"Processed RCOU data with keys: {list(processed.keys())}")
            else:
                print(f"Warning: Could not process RCOU data from {file}")
        else:
            print(f"Warning: No RCOU data found in {file}")

        if sim_data:
            print(f"Found {len(sim_data)} SIM messages after trimming")
            processed_sim_data.append(preprocess_sim_data(sim_data))
        else:
            print(f"Warning: No SIM data found in {file}")

    # Calculate DTW distances between all pairs for SIM data
    sim_distances = []
    for i, data1 in enumerate(processed_sim_data):
        for j, data2 in enumerate(processed_sim_data):
            if i == j:
                continue
            distance = calculate_dtw_distance(
                data1, data2, sim_fields
            )
            if distance is not None:
                sim_distances.append(distance)
                print(
                    f"SIM DTW distance between file {i+1} and file {j+1}: {distance:.4f}"
                )

    # Calculate DTW distances between all pairs for RCOU data
    rcou_distances = []

    # Create a list of all possible channel field names
    rcou_fields = []
    for chan in rcou_channels:
        rcou_fields.append(f"C{chan}")
        rcou_fields.append(f"chan{chan}")

    # Only proceed if we have data to compare
    if processed_rcou_data:
        # Print the available keys for debugging
        # if processed_rcou_data[0]:
        #     print(f"Available RCOU keys: {list(processed_rcou_data[0].keys())}")

        for i,data1 in enumerate(processed_rcou_data):
            for j,data2 in enumerate(processed_rcou_data):
                if i == j:
                    continue
                distance = calculate_dtw_distance(
                    data1,data2, rcou_fields
                )
                if distance is not None:
                    rcou_distances.append(distance)
                    print(
                        f"RCOU DTW distance between file {i+1} and file {j+1}: {distance:.4f}"
                    )

    return sim_distances, rcou_distances


def calculate_statistics(distances):
    """Calculate mean and standard deviation of distances"""
    if not distances:
        return None, None, None

    mean = statistics.mean(distances)
    stdev = statistics.stdev(distances) if len(distances) > 1 else 0
    two_sigma = 2 * stdev

    return mean, stdev, two_sigma


def plot_histogram(distances, mean, stdev, title, output_file=None):
    """Plot histogram of distances with mean and sigma lines"""
    plt.figure(figsize=(10, 6))
    plt.hist(distances, bins=10, alpha=0.7, color="skyblue", edgecolor="black")

    # Plot mean line
    plt.axvline(
        mean, color="red", linestyle="dashed", linewidth=2, label=f"Mean: {mean:.4f}"
    )

    # Plot mean ± sigma lines
    plt.axvline(
        mean + stdev,
        color="green",
        linestyle="dashed",
        linewidth=1,
        label=f"Mean + Sigma: {mean + stdev:.4f}",
    )
    plt.axvline(
        mean - stdev,
        color="green",
        linestyle="dashed",
        linewidth=1,
        label=f"Mean - Sigma: {mean - stdev:.4f}",
    )

    # Plot 3-sigma lines
    plt.axvline(
        mean + 3 * stdev,
        color="purple",
        linestyle="dashed",
        linewidth=1,
        label=f"Mean + 3Sigma: {mean + 3*stdev:.4f}",
    )
    plt.axvline(
        mean - 3 * stdev,
        color="purple",
        linestyle="dashed",
        linewidth=1,
        label=f"Mean - 3Sigma: {mean - 3*stdev:.4f}",
    )

    plt.title(f"Distribution of {title} DTW Distances")
    plt.xlabel("Normalized DTW Distance")
    plt.ylabel("Frequency")
    plt.legend()
    plt.grid(True, alpha=0.3)

    if output_file:
        plt.savefig(output_file)
        print(f"Plot saved to {output_file}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Analyze multiple bin files using DTW and calculate statistics."
    )
    parser.add_argument("directory", type=str, help="Directory containing bin files")
    parser.add_argument(
        "--sim-fields",
        type=str,
        nargs="+",
        default=["Q1", "Q2", "Q3", "Q4"],
        help="SIM fields to analyze (default: Roll Pitch Yaw Alt)",
    )
    parser.add_argument(
        "--rcou-channels",
        type=int,
        nargs="+",
        default=list(range(1, 9)),
        help="RCOU channels to analyze (default: 1-8)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=11,
        help="Number of newest files to analyze (default: 11)",
    )
    parser.add_argument("--plot", action="store_true", help="Generate histogram plots")
    parser.add_argument(
        "--output-prefix",
        type=str,
        default="dtw_histogram",
        help="Output file prefix for the plots (if --plot is specified)",
    )

    args = parser.parse_args()

    # Check if directory exists
    if not os.path.isdir(args.directory):
        print(f"Error: Directory '{args.directory}' does not exist.")
        return 1

    # Get the newest files
    files = get_newest_files(args.directory, args.count)
    if len(files) < 2:
        print(f"Error: Need at least 2 bin files for analysis, found {len(files)}.")
        return 1

    print(f"Analyzing {len(files)} files from {args.directory}...")
    for i, file in enumerate(files):
        print(f"{i+1}. {os.path.basename(file)}")

    # Analyze files
    sim_distances, rcou_distances = analyze_files(
        files, args.sim_fields, args.rcou_channels
    )

    # Calculate statistics for SIM data
    if sim_distances:
        sim_mean, sim_stdev, sim_two_sigma = calculate_statistics(sim_distances)

        print("\nSIM Statistics:")
        print(f"Number of comparisons: {len(sim_distances)}")
        print(f"Mean DTW distance: {sim_mean:.4f}")
        print(f"Standard deviation: {sim_stdev:.4f}")
        print(f"Mean - Sigma: {sim_mean - sim_stdev:.4f}")
        print(f"Mean + Sigma: {sim_mean + sim_stdev:.4f}")
        print(f"Mean - 2Sigma: {sim_mean - sim_two_sigma:.4f}")
        print(f"Mean + 2Sigma: {sim_mean + sim_two_sigma:.4f}")
    else:
        print("\nNo SIM data available for analysis")

    # Calculate statistics for RCOU data
    if rcou_distances:
        rcou_mean, rcou_stdev, rcou_two_sigma = calculate_statistics(rcou_distances)

        print("\nRCOU Statistics:")
        print(f"Number of comparisons: {len(rcou_distances)}")
        print(f"Mean DTW distance: {rcou_mean:.4f}")
        print(f"Standard deviation: {rcou_stdev:.4f}")
        print(f"Mean - Sigma: {rcou_mean - rcou_stdev:.4f}")
        print(f"Mean + Sigma: {rcou_mean + rcou_stdev:.4f}")
        print(f"Mean - 2Sigma: {rcou_mean - rcou_two_sigma:.4f}")
        print(f"Mean + 2Sigma: {rcou_mean + rcou_two_sigma:.4f}")
    else:
        print("\nNo RCOU data available for analysis")

    # Generate plots if requested
    if args.plot:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        if sim_distances:
            sim_output = f"{args.output_prefix}_sim_{timestamp}.png"
            plot_histogram(sim_distances, sim_mean, sim_stdev, "SIM", sim_output)

        if rcou_distances:
            rcou_output = f"{args.output_prefix}_rcou_{timestamp}.png"
            plot_histogram(rcou_distances, rcou_mean, rcou_stdev, "RCOU", rcou_output)

    return 0


if __name__ == "__main__":
    exit(main())
