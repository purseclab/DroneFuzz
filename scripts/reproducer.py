import argparse
import yaml
import os
from pymavlink import mavutil

"""
For now assume that we can run simulations on our own and then we just take the file from the fuzzer and rerun it
"""


def replay_inputs():
    pass


if __name__ == "__main__":

    argument_parser = argparse.ArgumentParser(description="Mini reproducer")
    argument_parser.add_argument(
        "--inputs", type=str, help="Path to the inputs file", required=True
    )
    argument_parser.add_argument(
        "--connection_string",
        type=str,
        help="Connection string for the vehicle",
        default="udp:localhost:14555",
    )
    argument_parser.add_argument(
        "--map_yaml", type=str, help="Path to the map yaml file", required=True
    )

    args = argument_parser.parse_args()

    # Figure out the sending frequencies from the mapping
    with open(args.map_yaml, "r") as f:
        pgfuzz_config = yaml.safe_load(f)

    # Open a connection with the string
    vehicle_conn = mavutil.mavlink_connection(args.connection_string)

    if os.path.exists(args.inputs):
        # Inputs are a set of dicts so load them in the same way
        with open(args.inputs, "r") as f:
            for line in f:
                # Each line is a
                input_list = eval(line.strip())
                # Check if the input dict is valid
                if isinstance(input_list, list):
                    # Send the input to the vehicle
                    print(f"Sent input: {input_list}")
                else:
                    print(f"Invalid input: {input_list}")
    else:
        print("Can't access input file.")
