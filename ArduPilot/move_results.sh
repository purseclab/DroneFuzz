#!/bin/bash
set -e
#  move_results.sh Description "This file is responsible for moving the results" Date 18.06.2024

# Now ask user for what unique identifier they want to use for the results folder
REASON=$1
if [ -z "$REASON" ]; then
	echo "Please provide a unique identifier for the results folder"
	exit 1
fi
# Check if PGFUZZ_HOME is set
if [ -z "$PGFUZZ_HOME" ]; then
	echo "Please set the PGFUZZ_HOME environment variable"
	exit 1
fi

# First get the commit under test from the fuzzing.log file
# Then move the results to the results folder, get only 8 bytes of the commit hash
COMMIT_HASH=$(grep "commit" fuzzing.log | cut -d ":" -f 2)
COMMIT=${COMMIT_HASH:1:8} # Cause there's a space in front of the commit hash

# Get today's date in YYYYMMDD format
DATENOW=$(date +"%Y%m%d")

# Concat the date, commit and reason to create a folder
mkdir -p $PGFUZZ_HOME/results-devel/ArduPilot/${DATENOW}_${COMMIT}_${REASON}

mv fuzzing.log $PGFUZZ_HOME/results-devel/ArduPilot/${DATENOW}_${COMMIT}_${REASON}/fuzzing.log
mv policy_violations/ $PGFUZZ_HOME/results-devel/ArduPilot/${DATENOW}_${COMMIT}_${REASON}/policy_violations
