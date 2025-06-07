#!/usr/bin/env python3
# -*- coding:utf-8 -*-
###
# Finis coronat opus; Run this at your own peril ~ silipwn
# File: oracle_lstm.py
# SPDX-License-Identifier: BSD-3-Clause or GPL-3.0-or-later
# Author: silipwn (contact@as-hw.in)
# Description: For training and testing an LSTM autoencoder for anomaly detection.
# Date: 2025-02-23T07:51:33-0500
# Last-Modified: 2025-06-07T16:10:53-0400
###

import argparse
import glob
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import tensorflow as tf
from tensorflow.keras.models import Model, load_model
from tensorflow.keras.layers import (
    LSTM,
    Dense,
    Dropout,
    Input,
    RepeatVector,
    TimeDistributed,
)
from pymavlink import mavutil

# --- Global Constants ---
SELECTED_COLS_RC = ["C1", "C2", "C3", "C4"]
# SELECTED_COLS_TELEMETRY = ["chan1_raw", "chan2_raw", "chan3_raw", "chan4_raw"] # This seems unused, consider removing if not planned for future use

# INITIAL_CUTOFF and FINAL_CUTOFF are no longer used for data loading based on MSG packets.
WINDOW_SIZE = 25     # Number of timesteps per sequence
DEFAULT_MODEL_FILENAME = "autoencoder_model.keras"
DEFAULT_THRESHOLD_FILENAME = "reconstruction_threshold.npy"
COMBINED_MAPPING_LIST = ["RCOU_C1", "RCOU_C2", "RCOU_C3", "RCOU_C4"]
COMBINED_MAPPING_DICT = {i: v for i, v in enumerate(COMBINED_MAPPING_LIST)}
RANDOM_SEED = 1337

# --- Function Definitions ---

def load_rcou_from_bin(file_path, output_channel_names):
    """
    Loads RCOU data from a MAVLink .BIN file, between "LOG RC" and "STOP RC" MSG/STATUSTEXT markers.
    output_channel_names: list of strings, e.g., ["C1", "C2"]
    These names will be used as column headers in the output DataFrame.
    It assumes output_channel_names[i] corresponds to RCOU.C(i+1).
    """
    try:
        # Using mavutil.mavlogfile for robust parsing, similar to how it handles .tlog
        # It falls back to mavlink_connection for .bin if needed.
        mlog = mavutil.mavlink_connection(file_path, robust_parsing=True)
    except Exception as e:
        print(f"Error opening or parsing BIN file {file_path}: {e}")
        return pd.DataFrame(columns=output_channel_names)

    rcou_entries = []
    num_channels_to_extract = len(output_channel_names)
    logging_active = False
    log_segment_started = False # To indicate if "LOG RC" was ever found

    while True:
        try:
            # Listen for RCOU, MSG, and STATUSTEXT messages
            msg = mlog.recv_match()
        except Exception as e:
            print(f"Error reading message from BIN file {file_path}: {e}")
            break # Stop processing this file on read error
        
        if msg is None:
            break # End of log file
        
        msg_type = msg.get_type()
        
        current_message_str = "" # Default to empty string

        if msg_type == 'MSG':
            if hasattr(msg, 'Message'):
                # msg.Message is usually str. Ensure it's a string for 'in' check.
                current_message_str = str(msg.Message) if msg.Message is not None else ""
        elif msg_type == 'STATUSTEXT':
            if hasattr(msg, 'text'):
                # msg.text is usually str. Ensure it's a string.
                current_message_str = str(msg.text) if msg.text is not None else ""
            
        if current_message_str: # Only proceed if we have some message text
            if "LOG RC" in current_message_str:
                if not logging_active:
                    print(f"Found 'LOG RC' in {file_path} (type: {msg_type}). Starting RCOU data logging.")
                    logging_active = True
                    log_segment_started = True
            elif "STOP RC" in current_message_str:
                if logging_active:
                    print(f"Found 'STOP RC' in {file_path} (type: {msg_type}). Stopping RCOU data logging.")
                    logging_active = False
        
        elif msg_type == 'RCOU':
            if logging_active:
                entry = {}
                for i in range(num_channels_to_extract):
                    r_field_name = f"C{i+1}"
                    if hasattr(msg, r_field_name):
                        entry[output_channel_names[i]] = getattr(msg, r_field_name)
                    else:
                        # This case should ideally not happen if RCOU message is well-formed
                        # and selected_cols align with available channels.
                        entry[output_channel_names[i]] = np.nan 
                
                if entry: # Ensure entry is not empty if all channels were NaN (though unlikely for RCOU)
                    rcou_entries.append(entry)
            # else: RCOU message received while not logging_active, so ignore it.
            
    if not log_segment_started:
        print(f"Warning: 'LOG RC' message not found in {file_path}. No RCOU data logged from this file based on MSG triggers.")
    elif logging_active: # Implies LOG RC was found but STOP RC was not found before EOF
        print(f"Warning: 'LOG RC' found in {file_path}, but 'STOP RC' not found before end of file. Logged RCOU data until EOF.")

    df = pd.DataFrame(rcou_entries, columns=output_channel_names)
    if not df.empty:
        # Drop rows where all specified RCOU channels are NaN (e.g., if some RCOU messages had fewer channels than expected)
        df.dropna(subset=output_channel_names, how='all', inplace=True)
    return df

def parse_cli_args():
    """Parses command line arguments."""
    parser = argparse.ArgumentParser(description="LSTM autoencoder for anomaly detection")
    parser.add_argument(
        "--input_folder",
        type=str,
        required=True,
        help="Path to the folder containing .BIN files for training/testing.",
    )
    parser.add_argument(
        "--compare_file",
        type=str,
        default=None,
        help="Path to a single .BIN file for comparison analysis against the trained/loaded model.",
    )
    parser.add_argument(
        "--epochs", type=int, default=25, help="Number of epochs for training"
    )
    parser.add_argument(
        "--batch_size", type=int, default=32, help="Batch size for training"
    )
    return parser.parse_args()

def load_dataset_from_bin_files(folder_path, file_pattern, selected_cols, dataset_name):
    """Loads and concatenates RCOU data from .BIN files in a given folder."""
    files = glob.glob(os.path.join(folder_path, file_pattern))
    files.sort(key=os.path.getctime)
    # Select the first N files in the folder  (ideally make this configurable)
    if len(files) > 20:
        files = files[:20]  # Limit to first 20 files for performance
    if not files:
        print(f"Warning: No {file_pattern} files found in {dataset_name} folder: {folder_path}")
        return pd.DataFrame(columns=selected_cols)

    all_data = []
    for f_path in files:
        df = load_rcou_from_bin(f_path, selected_cols)
        if not df.empty:
            all_data.append(df)

    if not all_data:
        print(f"Warning: No RCOU data successfully loaded from {dataset_name} .BIN files in {folder_path}.")
        return pd.DataFrame(columns=selected_cols)
    
    concatenated_data = pd.concat(all_data, ignore_index=True)
    if concatenated_data.empty:
        print(f"Warning: Concatenated RCOU data for {dataset_name} is empty.")
    return concatenated_data

def prepare_main_dataset(args):
    """Loads the main dataset for training."""
    input_folder = args.input_folder
    
    data_rc = load_dataset_from_bin_files(input_folder, "*.BIN", SELECTED_COLS_RC, "main training")
    if data_rc.empty:
        print(f"No RCOU data successfully loaded from .BIN files in {input_folder} for training. Exiting.")
        exit(1)
    return data_rc

def create_sequences(input_data, window_size):
    """Creates time-series sequences from input data."""
    X = []
    if input_data.empty or len(input_data) <= window_size:
        # This warning is now more critical if it's for the main training data
        # For optional datasets (anomaly/fixed), it's less critical.
        # The calling function should handle the implications of an empty array.
        print(f"Warning: Data length ({len(input_data)}) is too short for window_size ({window_size}). Cannot create sequences.")
        return np.array([])

    input_data_np = input_data.values
    for i in range(len(input_data_np) - window_size):
        X.append(input_data_np[i : i + window_size])
    return np.array(X)

def scale_training_data(X_seq, num_features):
    """Scales the training dataset using StandardScaler and returns the scaled data and the scaler."""
    scaler = StandardScaler()
    X_seq_reshaped = X_seq.reshape(-1, num_features)
    X_seq_scaled = scaler.fit_transform(X_seq_reshaped)
    X_seq_scaled = X_seq_scaled.reshape(X_seq.shape)
    return X_seq_scaled, scaler

def scale_comparison_data(data_seq, scaler, num_features):
    """Scales comparison data using a pre-fitted scaler."""
    if data_seq.size == 0:
        print("Warning: Comparison sequence data is empty, skipping scaling.")
        return np.array([])
    data_reshaped = data_seq.reshape(-1, num_features)
    data_scaled = scaler.transform(data_reshaped)
    data_scaled = data_scaled.reshape(data_seq.shape)
    return data_scaled

def build_and_train_model(X_seq_scaled, window_size, num_features, epochs, batch_size):
    """Builds, trains, and evaluates the LSTM autoencoder model."""
    X_train, X_test_val = train_test_split(X_seq_scaled, test_size=0.2, random_state=RANDOM_SEED)

    inputs = Input(shape=(window_size, num_features))
    encoded = LSTM(128, activation="relu", return_sequences=True)(inputs)
    encoded = Dropout(0.2)(encoded)
    encoded = LSTM(64, activation="relu", return_sequences=False)(encoded)
    bottleneck = RepeatVector(window_size)(encoded)
    decoded = LSTM(64, activation="relu", return_sequences=True)(bottleneck)
    decoded = Dropout(0.2)(decoded)
    decoded = LSTM(128, activation="relu", return_sequences=True)(decoded)
    outputs = TimeDistributed(Dense(num_features))(decoded)

    autoencoder = Model(inputs, outputs)
    autoencoder.compile(optimizer="adam", loss="mae")
    autoencoder.summary()

    print("Training model...")
    history = autoencoder.fit(
        X_train,
        X_train,
        epochs=epochs,
        batch_size=batch_size,
        validation_data=(X_test_val, X_test_val),
        verbose=1 # Added verbose for training progress
    )

    X_test_val_pred = autoencoder.predict(X_test_val)
    reconstruction_errors = np.mean(np.power(X_test_val - X_test_val_pred, 2), axis=(1, 2))
    threshold = np.percentile(reconstruction_errors, 98)
    print(f"Reconstruction error threshold for anomaly detection: {threshold}")

    plt.figure(figsize=(8, 4))
    plt.hist(reconstruction_errors, bins=50)
    plt.xlabel("Reconstruction error")
    plt.ylabel("Frequency")
    plt.title("Histogram of Reconstruction Errors on Validation Set")
    plt.show()

    idx = np.random.randint(0, X_test_val.shape[0])
    original_seq = X_test_val[idx]
    reconstructed_seq = X_test_val_pred[idx]

    plt.figure(figsize=(10, 6))
    # Plotting each feature separately
    for i in range(num_features):
        plt.subplot(num_features, 1, i + 1)
        plt.plot(original_seq[:, i], label=f"Original {COMBINED_MAPPING_DICT.get(i, f'Feature {i+1}')}")
        plt.plot(reconstructed_seq[:, i], label=f"Reconstructed {COMBINED_MAPPING_DICT.get(i, f'Feature {i+1}')}")
        plt.ylabel(COMBINED_MAPPING_DICT.get(i, f'Feature {i+1}'))
        if i == 0:
            plt.title("Original vs. Reconstructed Sequence (Validation Sample)")
        if i < num_features -1:
            plt.xticks([]) # Remove x-axis ticks for upper plots
    plt.xlabel("Timestep")
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15*num_features), fancybox=True, shadow=True, ncol=2) # Adjust legend
    plt.tight_layout(rect=[0, 0.05, 1, 0.95]) # Adjust layout to make space for legend
    plt.show()
    
    autoencoder.save(DEFAULT_MODEL_FILENAME)
    np.save(DEFAULT_THRESHOLD_FILENAME, threshold)
    print(f"Model saved to {DEFAULT_MODEL_FILENAME}")
    print(f"Threshold saved to {DEFAULT_THRESHOLD_FILENAME}")
    
    return autoencoder, threshold

def load_existing_model_and_threshold(model_path, threshold_path):
    """Loads an existing autoencoder model and threshold."""
    print(f"Loading existing model from {model_path}...")
    autoencoder = load_model(model_path)
    print(f"Loading existing threshold from {threshold_path}...")
    threshold = np.load(threshold_path)
    print(f"Loaded model and threshold: {threshold}")
    return autoencoder, threshold

def plot_comparison_results(data_scaled, predictions, errors, threshold, data_type_name, num_features, sequence_initial_cutoff, sequence_final_cutoff):
    """Plots results for comparison data (anomaly or fixed)."""
    if data_scaled.size == 0:
        print(f"No scaled {data_type_name} data to plot.")
        return

    # Apply sequence-based cutoffs to errors for mean calculation, but plot all points
    errors_for_mean = errors
    if len(errors) > sequence_initial_cutoff + sequence_final_cutoff and (sequence_initial_cutoff > 0 or sequence_final_cutoff > 0):
        if sequence_final_cutoff == 0: # Handle case where only initial cutoff is applied
             errors_for_mean = errors[sequence_initial_cutoff:]
        else:
             errors_for_mean = errors[sequence_initial_cutoff : -sequence_final_cutoff]
    elif (sequence_initial_cutoff > 0 or sequence_final_cutoff > 0) and len(errors) > 0 : # only if cutoffs were intended
        print(f"Warning: {data_type_name} sequence data length ({len(errors)}) is too short for full sequence cutoff. Using available data for mean error.")
    
    mean_error = np.mean(errors_for_mean) if errors_for_mean.size > 0 else 'N/A'
    print(f"Mean of reconstruction errors for {data_type_name} data: {mean_error}")

    anomalies_detected_mask = errors > threshold

    plt.figure(figsize=(12, 8))
    
    # Plot 1: Reconstruction Errors vs. Threshold
    plt.subplot(2, 1, 1)
    plt.plot(errors, label="Reconstruction Error", color='blue', alpha=0.7)
    plt.axhline(threshold, color="r", linestyle="--", label="Anomaly Threshold")
    plt.scatter(np.where(anomalies_detected_mask)[0], errors[anomalies_detected_mask], color='red', label='Detected Anomaly Point', s=50)
    plt.title(f"Reconstruction Errors for {data_type_name} Data")
    plt.xlabel("Sequence Index")
    plt.ylabel("Mean Squared Error")
    plt.legend()

    # Plot 2: Example of Original vs. Reconstructed Sequence from this dataset
    # Choose a sequence, perhaps one with high error if anomalies are detected, or random
    plt.subplot(2, 1, 2)
    if anomalies_detected_mask.any():
        idx_example = np.where(anomalies_detected_mask)[0][0] # First detected anomaly
        title_suffix = f"(Sequence {idx_example} - Detected Anomaly)"
    elif data_scaled.shape[0] > 0:
        idx_example = np.random.randint(0, data_scaled.shape[0])
        title_suffix = f"(Sequence {idx_example} - Random Sample)"
    else: # Should not happen if data_scaled.size > 0 check passed
        idx_example = 0 
        title_suffix = "(No data for example)"

    if data_scaled.shape[0] > 0 :
        original_seq_example = data_scaled[idx_example]
        reconstructed_seq_example = predictions[idx_example]
        for i in range(num_features):
            plt.plot(original_seq_example[:, i], label=f"Original {COMBINED_MAPPING_DICT.get(i, f'F{i+1}')}", alpha=0.7, linestyle='--')
            plt.plot(reconstructed_seq_example[:, i], label=f"Reconstructed {COMBINED_MAPPING_DICT.get(i, f'F{i+1}')}", alpha=0.7)
    
    plt.title(f"Original vs. Reconstructed - {data_type_name} {title_suffix}")
    plt.xlabel("Timestep in Sequence")
    plt.ylabel("Scaled Value")
    plt.legend(fontsize='small')
    
    plt.tight_layout()
    plt.show()

    # Deviation Metric
    deviation_metric = 0.0
    num_samples_for_metric = len(errors_for_mean)
    if num_samples_for_metric > 0: # Check if errors_for_mean is not empty
        # Ensure errors_for_mean is a numpy array for boolean indexing
        errors_for_mean_np = np.array(errors_for_mean)
        above_threshold_errors = errors_for_mean_np[errors_for_mean_np > threshold]
        if above_threshold_errors.size > 0:
            deviation_metric = np.sum(above_threshold_errors) / num_samples_for_metric
        else: # No errors above threshold
            deviation_metric = 0.0 
    else: # No errors to calculate metric from
        deviation_metric = 0.0
    print(f"{data_type_name} Data Deviation Metric: {deviation_metric:.4f}")


def main():
    """Main function to orchestrate the LSTM autoencoder workflow."""
    args = parse_cli_args()

    # --- Early Exit if Model Exists and No Comparison is Requested ---
    if (
        os.path.isfile(DEFAULT_MODEL_FILENAME)
        and os.path.isfile(DEFAULT_THRESHOLD_FILENAME)
        and args.compare_file is None
    ):
        print(
            f"Model '{DEFAULT_MODEL_FILENAME}' and threshold '{DEFAULT_THRESHOLD_FILENAME}' already exist."
        )
        try:
            # Optionally load and print the threshold
            loaded_threshold = np.load(DEFAULT_THRESHOLD_FILENAME)
            print(f"Existing threshold: {loaded_threshold}")
        except Exception as e:
            print(f"Could not load existing threshold to display: {e}")
        print("No comparison file provided. Nothing further to do. Exiting.")
        exit(0)

    np.random.seed(RANDOM_SEED)
    tf.random.set_seed(RANDOM_SEED)

    autoencoder = None
    threshold = None
    scaler = None
    
    # Determine the number of features based on SELECTED_COLS_RC.
    # This will be validated against loaded model's expectations or actual training data.
    expected_num_features = len(SELECTED_COLS_RC)
    active_num_features = expected_num_features # Initialize, will be confirmed/updated

    # --- Load Model and Scaler if Existing, or Train New Model ---
    if os.path.isfile(DEFAULT_MODEL_FILENAME) and os.path.isfile(DEFAULT_THRESHOLD_FILENAME):
        print(f"Loading existing model ('{DEFAULT_MODEL_FILENAME}') and threshold ('{DEFAULT_THRESHOLD_FILENAME}')...")
        autoencoder, threshold = load_existing_model_and_threshold(
            DEFAULT_MODEL_FILENAME, DEFAULT_THRESHOLD_FILENAME
        )
        
        model_input_features = autoencoder.input_shape[-1]
        if model_input_features != active_num_features:
            print(f"Warning: Loaded model expects {model_input_features} features, but SELECTED_COLS_RC implies {active_num_features}. Using model's expected features: {model_input_features}.")
            active_num_features = model_input_features
        else:
            print(f"Model expects {active_num_features} features, consistent with SELECTED_COLS_RC.")

        if args.compare_file:
            print("Comparison requested. Preparing main dataset to derive the scaler...")
            # Load main dataset specifically for fitting the scaler
            data_main_for_scaler = prepare_main_dataset(args) # Uses SELECTED_COLS_RC
            
            if data_main_for_scaler.empty:
                print("Main dataset (for scaler derivation) is empty. Cannot proceed with comparison. Exiting.")
                exit(1)
            
            if data_main_for_scaler.shape[1] != active_num_features:
                print(f"Error: Data for scaler has {data_main_for_scaler.shape[1]} features, but loaded model expects {active_num_features}. Exiting.")
                exit(1)
                
            X_seq_for_scaler = create_sequences(data_main_for_scaler, WINDOW_SIZE)
            if X_seq_for_scaler.size == 0:
                print("Failed to create sequences from main data for scaler derivation. Exiting.")
                exit(1)
            _, scaler = scale_training_data(X_seq_for_scaler, active_num_features)
            print("Scaler derived from main dataset.")
        # If no comparison file, autoencoder and threshold are loaded. Scaler remains None as it's not needed.
    else:
        print("No pre-trained model found. Preparing data for training a new model.")
        # --- Step 1: Load Main Dataset for Training ---
        data_main_processed = prepare_main_dataset(args) # Uses SELECTED_COLS_RC
        
        if data_main_processed.empty:
            # prepare_main_dataset already prints and exits if underlying load fails.
            print("Main dataset is empty after loading. Cannot proceed with training. Exiting.")
            exit(1)
        
        if data_main_processed.shape[1] != active_num_features:
            print(f"Warning: Training data has {data_main_processed.shape[1]} features, while SELECTED_COLS_RC implies {active_num_features}. Using actual data features: {data_main_processed.shape[1]}.")
            active_num_features = data_main_processed.shape[1]
        else:
            print(f"Training data has {active_num_features} features, consistent with SELECTED_COLS_RC.")

        # --- Step 2: Create Time-Series Sequences for Main Data ---
        X_seq = create_sequences(data_main_processed, WINDOW_SIZE)
        if X_seq.size == 0:
            print("Failed to create training sequences from main data. Exiting.")
            exit(1)

        # --- Step 3: Scale the Training Data ---
        X_seq_scaled, scaler = scale_training_data(X_seq, active_num_features)

        # --- Step 4 & 5 (part 2): Train the model ---
        print(f"Building and training new model with {active_num_features} features...")
        autoencoder, threshold = build_and_train_model(
            X_seq_scaled, WINDOW_SIZE, active_num_features, args.epochs, args.batch_size
        )
    
    # --- Step 6: Perform Comparison Analysis (if requested) ---
    if args.compare_file:
        if autoencoder is None or threshold is None:
            print("Error: Model or threshold not available for comparison. Exiting.")
            exit(1)
        if scaler is None:
            print("Error: Scaler not available for comparison. This can happen if model files existed but scaler derivation failed. Exiting.")
            exit(1)
            
        print(f"\n--- Analyzing Comparison File: {args.compare_file} ---")
        if not os.path.isfile(args.compare_file):
            print(f"Error: Comparison file not found: {args.compare_file}. Skipping comparison.")
        else:
            data_compare_processed = load_rcou_from_bin(args.compare_file, SELECTED_COLS_RC) # Uses SELECTED_COLS_RC
            
            if data_compare_processed.empty:
                print(f"No data loaded from comparison file {args.compare_file} (MSG-based filtering). Skipping comparison.")
            elif data_compare_processed.shape[1] != active_num_features:
                print(f"Error: Comparison data from {args.compare_file} has {data_compare_processed.shape[1]} features, but model/scaler expects {active_num_features}. Skipping comparison.")
            elif len(data_compare_processed) <= WINDOW_SIZE:
                print(f"Comparison file {args.compare_file} has insufficient data (length {len(data_compare_processed)}) for window size ({WINDOW_SIZE}) after MSG-based filtering. Skipping comparison.")
            else:
                compare_seq = create_sequences(data_compare_processed, WINDOW_SIZE)
                if compare_seq.size > 0:
                    compare_data_scaled = scale_comparison_data(compare_seq, scaler, active_num_features)
                    if compare_data_scaled.size > 0:
                        predictions_compare = autoencoder.predict(compare_data_scaled)
                        errors_compare = np.mean(np.power(compare_data_scaled - predictions_compare, 2), axis=(1, 2))
                            
                        plot_title_name = os.path.basename(args.compare_file)
                        # Pass 0,0 for sequence_initial_cutoff and sequence_final_cutoff to use all sequences for mean error calculation
                        plot_comparison_results(compare_data_scaled, predictions_compare, errors_compare, threshold, plot_title_name, active_num_features, 0, 0)
                    else:
                        print(f"Failed to scale comparison data from {args.compare_file}. Skipping comparison plotting.")
                else:
                    print(f"No sequences created from {args.compare_file} after processing. Skipping comparison.")
    else:
        print("\nTraining/loading completed.")
        print(f"Model: {DEFAULT_MODEL_FILENAME}, Threshold for anomaly detection: {threshold}")
        print("Use --compare_file <path_to_bin_file> to analyze a specific BIN file with this model.")

if __name__ == "__main__":
    main()
