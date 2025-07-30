#!/usr/bin/env python3
"""
Script to parse Ardupilot BIN files and plot servo (RC) values.
Extracts C1/C2/C3/C4 channels from two BIN files and plots comparisons between them.
"""

import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
from typing import Dict, List, Tuple, Optional, Any


def parse_ardupilot_bin(
    bin_file: str, rc_log_filter: bool, channels: List[str] = None
) -> Dict[str, Any]:
    """
    Parse an Ardupilot BIN file and extract servo (RC) values.

    Args:
        bin_file: Path to the BIN file
        channels: List of channels to extract (default: C1-C16)

    Returns:
        Dictionary containing servo data with timestamps
    """
    if not os.path.exists(bin_file):
        raise FileNotFoundError(f"BIN file not found: {bin_file}")

    # Open the binary log file
    mlog = mavutil.mavlink_connection(bin_file)

    # Define all possible channels
    all_channels = [f"C{i}" for i in range(1, 17)]  # C1 to C16

    # Use specified channels or default to C1-C4
    if channels is None:
        channels = ["C1", "C2", "C3", "C4"]

    # Validate channels
    for channel in channels:
        if channel not in all_channels:
            raise ValueError(
                f"Invalid channel: {channel}. Must be one of {all_channels}"
            )

    # Initialize data structures
    servo_data = {"timestamp": []}
    for channel in all_channels:
        servo_data[channel] = []

    # If we have the filter triggered, we need to set the flag
    if rc_log_filter:
        rc_log_flag = False
    # Else, use the default value
    else:
        rc_log_flag = True
    # Extract RCOU messages (servo outputs)
    while True:
        msg = mlog.recv_match()
        if msg is None:
            break
        if msg.get_type() == "MSG" and rc_log_filter:
            if "LOG RC" in msg.Message:
                print("RC log filter triggered")
                rc_log_flag = True
            if "STOP RC" in msg.Message:
                print("RC log filter removed")
                rc_log_flag = False
        if msg.get_type() == "RCOU":
            # Process RCOU message here
            # Store timestamp
            if rc_log_flag:
                servo_data["timestamp"].append(
                    msg.TimeUS / 1000000.0
                )  # Convert to seconds
                # Store channel values
                for i in range(1, 17):
                    channel = f"C{i}"
                    if hasattr(msg, channel):
                        servo_data[channel].append(getattr(msg, channel))
                    else:
                        servo_data[channel].append(
                            0
                        )  # Default value if channel doesn't exist

    # Convert to numpy arrays for easier manipulation
    for key in servo_data:
        servo_data[key] = np.array(servo_data[key])

    # Check if we got any data
    if len(servo_data["timestamp"]) == 0:
        raise ValueError(f"No RCOU messages found in {bin_file}")

    return servo_data


def plot_servo_values(
    servo_data1: Dict[str, np.ndarray],
    servo_data2: Dict[str, np.ndarray],
    channels: List[str] = ["C1", "C2", "C3", "C4"],
    show_diff: bool = True,
    diff_channel: str = "C1",
    colors: Dict[str, str] = None,
    labels: Tuple[str, str] = ("BIN 1", "BIN 2"),
    title: str = "Servo Values Comparison",
    output_file: Optional[str] = None,
    figsize: Tuple[int, int] = (12, 10),
) -> None:
    """
    Plot servo values from two parsed BIN files and compare them.

    Args:
        servo_data1: Dictionary containing servo data from first BIN file
        servo_data2: Dictionary containing servo data from second BIN file
        channels: List of channels to plot
        show_diff: Whether to show differences between channels
        diff_channel: The reference channel for difference calculation
        colors: Dictionary mapping channels to colors
        labels: Labels for the two BIN files
        title: Plot title
        output_file: Path to save the plot (if None, display only)
        figsize: Figure size as (width, height) in inches
    """
    # Default colors with meaning:
    # C1 (Roll): Blue - Sky/horizon reference
    # C2 (Pitch): Green - Earth/ground reference
    # C3 (Throttle): Red - Power/energy
    # C4 (Yaw): Purple - Rotation/direction
    default_colors = {"C1": "blue", "C2": "green", "C3": "red", "C4": "purple"}

    if colors is None:
        colors = default_colors

    # Create figure with 3 subplots if showing diff, otherwise 2
    fig, axes = plt.subplots(3 if show_diff else 2, 1, figsize=figsize, sharex=True)

    # Plot raw servo values from first BIN file
    for channel in channels:
        if channel in servo_data1:
            axes[0].plot(
                servo_data1["timestamp"],
                servo_data1[channel],
                label=f"{channel} ({labels[0]})",
                color=colors.get(channel, "black"),
                linestyle="-",
            )

    axes[0].set_ylabel("Servo Value")
    axes[0].set_title(f"{title} - {labels[0]}")
    axes[0].legend()
    axes[0].grid(True)

    # Plot raw servo values from second BIN file
    for channel in channels:
        if channel in servo_data2:
            axes[1].plot(
                servo_data2["timestamp"],
                servo_data2[channel],
                label=f"{channel} ({labels[1]})",
                color=colors.get(channel, "black"),
                linestyle="-",
            )

    axes[1].set_ylabel("Servo Value")
    axes[1].set_title(f"{title} - {labels[1]}")
    axes[1].legend()
    axes[1].grid(True)

    # Plot differences between corresponding channels if requested
    if show_diff:
        # Create a common time scale for both datasets
        # Find the overlapping time range
        start_time = max(servo_data1["timestamp"][0], servo_data2["timestamp"][0])
        end_time = min(servo_data1["timestamp"][-1], servo_data2["timestamp"][-1])

        # Create a common time grid with reasonable number of points
        num_points = 1000  # Adjust as needed for resolution vs performance
        common_time = np.linspace(start_time, end_time, num_points)

        # Dictionary to store interpolated values
        interp_data1 = {}
        interp_data2 = {}

        # Interpolate both datasets to the common time grid
        for channel in channels:
            if channel in servo_data1 and channel in servo_data2:
                # Interpolate data from both files to common time grid
                interp_data1[channel] = np.interp(
                    common_time, servo_data1["timestamp"], servo_data1[channel]
                )

                interp_data2[channel] = np.interp(
                    common_time, servo_data2["timestamp"], servo_data2[channel]
                )

                # Calculate and plot difference
                diff = interp_data2[channel] - interp_data1[channel]
                label = f"{channel} ({labels[1]} - {labels[0]})"

                axes[2].plot(
                    common_time, diff, label=label, color=colors.get(channel, "black")
                )

        axes[2].set_ylabel("Difference")
        axes[2].set_xlabel("Time (seconds)")
        axes[2].set_title(f"Channel Differences Between {labels[0]} and {labels[1]}")
        axes[2].legend()
        axes[2].grid(True)
    else:
        axes[1].set_xlabel("Time (seconds)")

    plt.tight_layout()

    # Save or display the plot
    if output_file:
        plt.savefig(output_file)
        print(f"Plot saved to {output_file}")
    else:
        plt.show()


def main():
    """Main function to parse arguments and run the script."""
    parser = argparse.ArgumentParser(
        description="Parse two Ardupilot BIN files and plot servo (RC) value comparisons."
    )

    parser.add_argument("bin_file1", help="Path to the first Ardupilot BIN file")

    parser.add_argument("bin_file2", help="Path to the second Ardupilot BIN file")

    parser.add_argument(
        "--channels",
        nargs="+",
        default=["C1", "C2", "C3", "C4"],
        help="Channels to plot (default: C1 C2 C3 C4)",
    )

    parser.add_argument(
        "--no-diff",
        action="store_true",
        help="Do not show channel differences between BIN files",
    )

    parser.add_argument(
        "--labels",
        nargs=2,
        default=["BIN 1", "BIN 2"],
        help='Labels for the two BIN files (default: "BIN 1" "BIN 2")',
    )

    parser.add_argument(
        "--colors",
        nargs="+",
        help="Custom colors for channels (format: C1=blue C2=green ...)",
    )

    parser.add_argument("--title", default="Servo Values Comparison", help="Plot title")

    parser.add_argument(
        "--output", help="Path to save the plot (if not specified, display only)"
    )

    parser.add_argument(
        "--figsize",
        nargs=2,
        type=int,
        default=[12, 10],
        help="Figure size as width height (default: 12 10)",
    )

    parser.add_argument(
        "--rc-log-filter", action="store_true", help="Enable RC log filter"
    )

    args = parser.parse_args()

    # Parse custom colors if provided
    colors = None
    if args.colors:
        colors = {}
        for color_arg in args.colors:
            if "=" in color_arg:
                channel, color = color_arg.split("=")
                colors[channel] = color

    try:
        # Parse BIN files with specified channels
        print(f"Parsing first BIN file: {args.bin_file1}")
        servo_data1 = parse_ardupilot_bin(
            args.bin_file1, args.rc_log_filter, args.channels
        )

        print(f"Parsing second BIN file: {args.bin_file2}")
        servo_data2 = parse_ardupilot_bin(
            args.bin_file2, args.rc_log_filter, args.channels
        )

        # Plot servo values comparison
        plot_servo_values(
            servo_data1=servo_data1,
            servo_data2=servo_data2,
            channels=args.channels,
            show_diff=not args.no_diff,
            colors=colors,
            labels=tuple(args.labels),
            title=args.title,
            output_file=args.output,
            figsize=tuple(args.figsize),
        )

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
