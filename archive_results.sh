#!/bin/bash

# Get current date in YYYY/MM/DD format
current_date=$(date +"%Y%m%d")

# Move to the ardupilot_pgfuzz directory
pushd ardupilot_pgfuzz/
# Get the short Git hash (7 digits)
git_hash=$(git rev-parse --short=7 HEAD)
popd

# Prompt for experiment reason
read -p "Enter a 2-3 word summary for the experiment: " experiment_reason

# Replace spaces with underscores in the experiment reason
experiment_reason=${experiment_reason// /_}

# Create the archive name
archive_name="${current_date}_${git_hash}_${experiment_reason}"

# Create the archive directory
mkdir -p "results-devel/ArduPilot/$archive_name"
# Get the realpath of the archive directory
archive_dir=$(realpath "results-devel/ArduPilot/$archive_name")

# Copy the folders and file to the archive directory
pushd ArduPilot/$archive_name
cp -r policy_violations $archive_dir
cp -r logs $archive_dir
cp fuzzing.log $archive_dir
popd

echo "Archiving complete. Results stored in results-devel/ArduPilot/$archive_name"
