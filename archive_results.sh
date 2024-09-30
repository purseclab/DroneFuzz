#!/bin/bash

# Get current date in YYYY/MM/DD format
current_date=$(date +"%Y/%m/%d")

# Get the short Git hash (7 digits)
git_hash=$(git rev-parse --short=7 HEAD)

# Prompt for experiment reason
read -p "Enter a 2-3 word summary for the experiment: " experiment_reason

# Replace spaces with underscores in the experiment reason
experiment_reason=${experiment_reason// /_}

# Create the archive name
archive_name="${current_date}_${git_hash}_${experiment_reason}"

# Create the archive directory
mkdir -p "results-devel/$archive_name"

# Copy the folders and file to the archive directory
cp -r policy_violations "results-devel/$archive_name/"
cp -r logs "results-devel/$archive_name/"
cp fuzzing.log "results-devel/$archive_name/"

echo "Archiving complete. Results stored in results-devel/$archive_name"
