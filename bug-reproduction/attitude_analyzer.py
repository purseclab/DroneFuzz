from pymavlink import mavutil
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt
import argparse
import sys
import re


def extract_attitude_data(tlog_path):
    """
    Extract attitude data from tlog file
    Returns: timestamps, roll, pitch, yaw arrays
    """
    mlog = mavutil.mavlink_connection(tlog_path)

    timestamps = []
    rolls = []
    pitches = []
    yaws = []

    while True:
        msg = mlog.recv_match(type="ATTITUDE")
        if msg is None:
            break

        timestamps.append(msg._timestamp)  # Convert to seconds
        rolls.append(np.degrees(msg.roll))
        pitches.append(np.degrees(msg.pitch))
        yaws.append(np.degrees(msg.yaw))

    return np.array(timestamps), np.array(rolls), np.array(pitches), np.array(yaws)


def calculate_attitude_differentials(timestamps, attitudes):
    """
    Calculate rate of change of attitude values
    Returns: Array of differentials (degrees/second)
    """
    time_diffs = np.diff(timestamps)
    attitude_diffs = np.diff(attitudes)

    # Calculate rate of change (degrees per second)
    differentials = attitude_diffs / time_diffs

    return differentials


def extract_policy_violations(tlog_path):
    """
    Extract timestamps of policy violations from tlog file
    Args:
        tlog_path: Path to the tlog file
    Returns: Tuple of (violations list, mission_start_time)
    """
    mlog = mavutil.mavlink_connection(tlog_path)
    violations = []

    # Get the first message to establish mission start time
    first_msg = mlog.recv_match(type="ATTITUDE")
    if first_msg is None:
        return [], None
    mission_start_time = first_msg._timestamp
    print("Mission start time:", datetime.fromtimestamp(mission_start_time)
    )

    # Rewind to start of log
    mlog = mavutil.mavlink_connection(tlog_path)

    while True:
        msg = mlog.recv_match()
        if msg is None:
            break

        # Check for policy violation messages in STATUSTEXT
        if msg.get_type() == "STATUSTEXT":
            if "Policy violation" in msg.text:
                violation_time = msg._timestamp
                # time_since_start = violation_time - mission_start_time
                # print("time_since_start:", time_since_start)
                violations.append(violation_time)
                # print(
                #     f"Policy violation detected at {time_since_start:.2f} seconds from mission start"
                # )
                print("Policy violation datetime:",datetime.fromtimestamp(violation_time))

    return violations, mission_start_time


def plot_attitude_data(
    timestamps,
    rolls,
    pitches,
    yaws,
    roll_diffs,
    pitch_diffs,
    yaw_diffs,
    violation_times=None,
):
    """Plot attitude values and their differentials with optional violation markers"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # Plot attitude values
    ax1.plot(timestamps, rolls, label="Roll")
    ax1.plot(timestamps, pitches, label="Pitch")
    ax1.plot(timestamps, yaws, label="Yaw")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Degrees")
    ax1.set_title("Attitude Values")
    ax1.legend()
    ax1.grid(True)

    # Add violation markers if provided
    if violation_times:
        for vt in violation_times:
            if min(timestamps) <= vt <= max(timestamps):
                ax1.axvline(
                    x=vt, color="r", linestyle="--", alpha=0.5, label="Policy Violation"
                )
                ax2.axvline(x=vt, color="r", linestyle="--", alpha=0.5)
                # Add text annotation in the first subplot
                ax1.text(
                    vt,
                    ax1.get_ylim()[1],
                    "Violation",
                    rotation=90,
                    verticalalignment="top",
                    color="r",
                )

    # Plot differentials
    ax2.plot(timestamps[1:], roll_diffs, label="Roll Rate")
    ax2.plot(timestamps[1:], pitch_diffs, label="Pitch Rate")
    ax2.plot(timestamps[1:], yaw_diffs, label="Yaw Rate")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Degrees/second")
    ax2.set_title("Attitude Rates of Change")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Analyze attitude data from tlog files"
    )
    parser.add_argument("tlog_path", help="Path to the tlog file")

    args = parser.parse_args()

    if not args.tlog_path:
        print("Error: Please provide a tlog file path")
        sys.exit(1)

    try:
        # Extract attitude data
        timestamps, rolls, pitches, yaws = extract_attitude_data(args.tlog_path)
        if len(timestamps) == 0:
            print("No attitude data found in the log file")
            sys.exit(1)

        # Calculate differentials
        roll_diffs = calculate_attitude_differentials(timestamps, rolls)
        pitch_diffs = calculate_attitude_differentials(timestamps, pitches)
        yaw_diffs = calculate_attitude_differentials(timestamps, yaws)

        # Extract violation timestamps and mission start time from tlog
        violation_times, mission_start_time = extract_policy_violations(args.tlog_path)

        if violation_times:
            print(f"Found {len(violation_times)} policy violations")
            print(f"Mission start time: {datetime.fromtimestamp(mission_start_time)}")


        # Plot the results
        plot_attitude_data(
            timestamps,
            rolls,
            pitches,
            yaws,
            roll_diffs,
            pitch_diffs,
            yaw_diffs,
            violation_times,
        )

    except Exception as e:
        print(f"Error processing log file: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()
