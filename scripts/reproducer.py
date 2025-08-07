import argparse
import time
import threading
import sys
import os
import ast
from typing import List
import tempfile
from types import SimpleNamespace

# Add parent directory to path to import minimal_poc
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from minimal_poc import FuzzConfig, setup_logging, file_exists

def parse_file(mutated_file: str) -> List:
    # Contains the actual messages
    # Check if the file exists
    file_exists(mutated_file)
    parsed_data = []
    with open(mutated_file) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                parsed = ast.literal_eval(line)
                parsed_data.append(parsed)
            except Exception as e:
                print(f"Failed with {e}")
                return []
    return parsed_data

def main():
    """
    Main function to drive the replay script.
    """
    parser = argparse.ArgumentParser(
        description="Reproducer script for peripheral messages."
    )
    parser.add_argument(
        "-c",
        "--config",
        required=True,
        help="Path to the configuration YAML file.",
    )
    parser.add_argument(
        "-i",
        "--inputs",
        required=True,
        help="Path to input files for the peripheral. If not provided, will use config defaults.",
    )

    args = parser.parse_args()
    fuzzer = None
    temp_dir = tempfile.mkdtemp("run_data","reproducer", os.getcwd())
    logger = setup_logging("reproducer", file_dir=temp_dir)

    try:
        # Create a namespace object for FuzzConfig arguments
        fuzz_args = SimpleNamespace(
            config=args.config,
            fuzzer_temp_dir=temp_dir,
            # Set other potential args to None to use config file values
            bin=None,
            peripheral=None,
            ap_dir=None,
            xml=None,
            peripheral_file=None,
            vehicle=None,
            calibration_rounds=None,
            dtw_threshold=None,
        )

        # Initialize FuzzConfig
        fuzzer = FuzzConfig(fuzz_args, logger)

        # Check the file we have
        mutated_data = parse_file(args.inputs)
        # Run a single simulation
        logger.info("Starting simulation run.")
        fuzzer.run_sim()

        logger.info("Waiting for drone to be ready...")
        # Wait for the drone to be ready (GPS lock, etc.)

        ready = fuzzer.wait_for_condition(lambda: fuzzer.tcp_conn.drone_ready, timeout=120)

        if not ready:
            raise Exception("Drone did not become ready in time.")

        logger.info("Drone is ready. Sending mission.")
        # Start the replayer thread
        replayer_thread = threading.Thread(target=fuzzer.replay_messages,args=(mutated_data,))
        replayer_thread.start()
        fuzzer.send_mission(fuzzing=False)
        logger.info("Simulation run finished.")
        if fuzzer:
            logger.info("Cleaning up simulation.")
            fuzzer.cleanup_sim(oracle=False)

    except Exception as e:
        logger.error(f"An error occurred: {e}", exc_info=True)
        sys.exit(1)
    finally:
        logger.info("Reproducer script finished.")

if __name__ == "__main__":
    main()
