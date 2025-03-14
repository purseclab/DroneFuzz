#!/usr/bin/env python3
# -*- coding:utf-8 -*-
###
# Finis coronat opus; Run this at your own peril ~ silipwn
# File: oracle_lstm.py
# SPDX-License-Identifier: BSD-3-Clause or GPL-3.0-or-later
# Author: silipwn (contact@as-hw.in)
# Description: For training and testing an LSTM autoencoder for anomaly detection.
# Date: 2025-02-23T07:51:33-0500
# Last-Modified: 2025-03-11T10:26:17-0400
###
import os
import glob
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import tensorflow as tf
from tensorflow.keras.models import Model, Sequential
from tensorflow.keras.layers import (
    LSTM,
    Dense,
    Dropout,
    Input,
    RepeatVector,
    TimeDistributed,
)
import matplotlib.pyplot as plt

# --- Step 1: Load CSV files from a folder and select desired columns ---
# input_folder = '/mnt/oldhd/Data/PGFUZZ-data/ml_data/rfbug_stable' # Replace with your folder path
# input_folder = '/mnt/oldhd/Data/PGFUZZ-data/ml_data/gimbalyaw_stable' # Replace with your folder path
# input_folder = "mount_csvs/"  # Mount Bug
# input_folder = "/home/silipwn/Documents/Drone/PGFUZZplusplus/ardupilot_rf_bug/logs/"
input_folder = "/home/silipwn/Documents/Drone/PGFUZZplusplus/ardupilot_gimbal_bug/logs/"
rc_csv_files = glob.glob(os.path.join(input_folder, "*_RCOU.csv"))
sim_csv_files = glob.glob(os.path.join(input_folder, "*_SIM.csv"))

anomaly_csv = input_folder + "/anomaly/"
rc_anomaly_files = glob.glob(os.path.join(anomaly_csv, "*_RCOU.csv"))
sim_anomaly_files = glob.glob(os.path.join(anomaly_csv, "*_SIM.csv"))

fixed_csv = input_folder + "/fixed/"
rc_fixed_files = glob.glob(os.path.join(fixed_csv, "*_RCOU.csv"))
sim_fixed_files = glob.glob(os.path.join(fixed_csv, "*_SIM.csv"))

# List of columns to extract (for example, two features and one target)
selected_cols_rc = ["chan1_raw", "chan2_raw", "chan3_raw", "chan4_raw"]
selected_cols_sim = ["yaw", "roll", "pitch"]

# Ignore the first 50 samples and last 200 samples (for takeoff and landing)
INITIAL_CUTOFF = 50
FINAL_CUTOFF = 200

combined_mapping_list = [
    "C1",
    "C2",
    "C3",
    "C4",
    "Yaw",
    "Roll",
    "Pitch",
]

combined_mapping_dict = {i: v for i, v in enumerate(combined_mapping_list)}

# Ensure the seeds are set always the same
np.random.seed(1337)
tf.random.set_seed(1337)

# Read and concatenate CSV files
df_list = [pd.read_csv(f)[selected_cols_rc] for f in rc_csv_files]
data_rc = pd.concat(df_list, ignore_index=True)

df_list = [pd.read_csv(f)[selected_cols_sim] for f in sim_csv_files]
data_sim = pd.concat(df_list, ignore_index=True)

df_test = [pd.read_csv(f)[selected_cols_rc] for f in rc_anomaly_files]
data_anomaly_rc = pd.concat(df_test, ignore_index=True)

df_test = [pd.read_csv(f)[selected_cols_sim] for f in sim_anomaly_files]
data_anomaly_sim = pd.concat(df_test, ignore_index=True)
df_fixed = [pd.read_csv(f)[selected_cols_rc] for f in rc_fixed_files]
data_fixed_rc = pd.concat(df_fixed, ignore_index=True)

df_fixed = [pd.read_csv(f)[selected_cols_sim] for f in sim_fixed_files]
data_fixed_sim = pd.concat(df_fixed, ignore_index=True)


# Merge the two dataframes
data = pd.concat([data_rc, data_sim], axis=1)
data_test = pd.concat([data_anomaly_rc, data_anomaly_sim], axis=1)
data_fixed = pd.concat([data_fixed_rc, data_fixed_sim], axis=1)
# data = data_rc
# data_test = data_anomaly_rc
# data_fixed = data_fixed_rc

# --- Step 2: Prepare features and target ---
# Separate features and target column
features = data[selected_cols_rc].values
targets = data[selected_cols_sim].values

num_features = features.shape[1] + targets.shape[1]
# --- Step 2: Create Time-Series Sequences ---
# For anomaly detection, we want to reconstruct the same sequence.

window_size = 25  # Number of timesteps per sequence


def create_sequences(data, window_size):
    X = []
    for i in range(len(data) - window_size):
        X.append(data[i : i + window_size])
    return np.array(X)


X_seq = create_sequences(data, window_size)
test_data = create_sequences(data_test, window_size)
fixed_data = create_sequences(data_fixed, window_size)

# --- Step 3: Scale the Data ---
scaler = StandardScaler()
X_seq_reshaped = X_seq.reshape(-1, num_features)
X_seq_scaled = scaler.fit_transform(X_seq_reshaped)
X_seq_scaled = X_seq_scaled.reshape(X_seq.shape)

test_data_reshaped = test_data.reshape(-1, num_features)
test_data_scaled = scaler.transform(test_data_reshaped)
test_data_scaled = test_data_scaled.reshape(test_data.shape)

fixed_data_reshaped = fixed_data.reshape(-1, num_features)
fixed_data_scaled = scaler.transform(fixed_data_reshaped)
fixed_data_scaled = fixed_data_scaled.reshape(fixed_data.shape)

default_model = "autoencoder_model.keras"
default_threshold = "reconstruction_threshold.npy"
# Check if the model is already trained
if os.path.isfile(default_model):
    from tensorflow.keras.models import load_model

    autoencoder = load_model(default_model)
    print("Model loaded")
    threshold = np.load(default_threshold)
    print(f"Loaded threshold for anomaly detection: {threshold}")
else:
    # --- Step 4: Split into Training and Testing Sets ---
    # Ideally, the training data consists of only "normal" sequences.
    # Here we split the dataset (adjust if you have labeled normal vs. anomalous).
    X_train, X_test = train_test_split(X_seq_scaled, test_size=0.2, random_state=42)
    # --- Step 5: Build an LSTM Autoencoder Model ---
    # Encoder
    inputs = Input(shape=(window_size, num_features))
    encoded = LSTM(128, activation="relu", return_sequences=True)(inputs)
    encoded = Dropout(0.2)(encoded)
    encoded = LSTM(64, activation="relu", return_sequences=False)(encoded)
    # Bottleneck representation
    bottleneck = RepeatVector(window_size)(encoded)
    # Decoder
    decoded = LSTM(64, activation="relu", return_sequences=True)(bottleneck)
    decoded = Dropout(0.2)(decoded)
    decoded = LSTM(128, activation="relu", return_sequences=True)(decoded)
    # TimeDistributed output layer to reconstruct original features at each timestep
    outputs = TimeDistributed(Dense(num_features))(decoded)

    autoencoder = Model(inputs, outputs)
    # TODO Explore if higher learning rate would improve
    autoencoder.compile(optimizer="adam", loss="mae")
    autoencoder.summary()

    # --- Step 6: Train the Model ---
    history = autoencoder.fit(
        X_train, X_train, epochs=25, batch_size=32, validation_data=(X_test, X_test)
    )

    # --- Step 7: Compute Reconstruction Error ---
    # Predict on test set
    X_test_pred = autoencoder.predict(X_test)
    # Calculate MSE for each sequence (averaging over timesteps and features)
    reconstruction_errors = np.mean(np.power(X_test - X_test_pred, 2), axis=(1, 2))
    # Ignore the first 50 samples and last 200 samples
    print(f"Mean of reconstruction errors: {np.mean(reconstruction_errors)}")
    # Threshold for anomaly detection (e.g., 95th percentile)
    threshold = np.percentile(reconstruction_errors, 95)
    print(f"Reconstruction error threshold for anomaly detection: {threshold}")

    # Plot histogram of reconstruction errors
    plt.figure(figsize=(8, 4))
    plt.hist(reconstruction_errors, bins=50)
    plt.xlabel("Reconstruction error")
    plt.ylabel("Frequency")
    plt.title("Histogram of Reconstruction Errors")
    plt.show()

    # --- Step 8: Plot Example of Original vs. Reconstructed Sequence ---
    # Choose a random sample from the test set
    idx = np.random.randint(0, X_test.shape[0])
    original_seq = X_test[idx]
    reconstructed_seq = X_test_pred[idx]

    # Save the model
    autoencoder.save("autoencoder_model.keras")
    # Save the threshold
    np.save("reconstruction_threshold.npy", threshold)
    print("Model saved")

    # Plot each feature (or average over features)
    plt.figure(figsize=(10, 6))
    plt.plot(original_seq.mean(axis=1), label="Original (avg over features)")
    plt.plot(reconstructed_seq.mean(axis=1), label="Reconstructed (avg over features)")
    plt.xlabel("Timestep")
    plt.ylabel("Average Feature Value")
    plt.title("Original vs. Reconstructed Sequence")
    plt.legend()
    plt.show()

# Check with real anomaly data
X_test_pred = autoencoder.predict(test_data_scaled)
reconstruction_errors = np.mean(np.power(test_data_scaled - X_test_pred, 2), axis=(1))
reconstruction_errors = reconstruction_errors[50:-200]
print(f"Mean of reconstruction errors: {np.mean(reconstruction_errors)}")

# You can set a threshold (e.g., based on percentile or statistical properties)
# Calculate anomalies for each feature seperately, and store into an array for plotting
anomalies = np.zeros((test_data_scaled.shape[0], num_features), dtype=bool)
for index in range(num_features):
    feature_reconstruction_errors = np.mean(
        np.power(test_data_scaled[:, :, index] - X_test_pred[:, :, index], 2), axis=1
    )
    # threshold = np.percentile(feature_reconstruction_errors, 95)  # 95th percentile as threshold
    anomalies[:, index] = feature_reconstruction_errors > threshold

# Remove anomalies from the first 50 samples and last 200 samples -> change them to False
anomalies[:INITIAL_CUTOFF] = False  # In the mount scenario
anomalies[-FINAL_CUTOFF:] = False

deviation_metric = 0.00
for index in range(num_features):
    print(
        f"Number of anomalies detected in feature {combined_mapping_dict[index]}: {np.sum(anomalies[:, index])} out of {len(reconstruction_errors)} samples, The current threshold is {threshold}"
    )
    deviation_metric += np.sum(anomalies[:, index]) / len(reconstruction_errors)

print("Deviation Metric: ", deviation_metric)
# anomalies = reconstruction_errors > threshold
# print(f"Number of anomalies detected: {np.sum(anomalies)} out of {len(reconstruction_errors)} samples, The current threshold is {threshold}")
# # Create a variable to store the index of the anomalies relative to the test_data
# anomaly_indices = np.where(anomalies)[0]
# Find the indices only where values less than the features are changed
# Split the anomaly_indices into number of features

# Plot the original and reconstructed sequence only per feature
# Create a subplot for each feature
# Scatter plot the anomalies
fig, ax = plt.subplots(num_features, 1)
for index in range(num_features):
    original_feature = test_data_scaled[:, :, index]
    reconstructed_feature = X_test_pred[:, :, index]
    ax[index].plot(original_feature.mean(axis=1), label="Original (avg over timesteps)")
    ax[index].plot(
        reconstructed_feature.mean(axis=1), label="Reconstructed (avg over timesteps)"
    )
    ax[index].scatter(
        np.where(anomalies[:, index])[0],
        original_feature.mean(axis=1)[anomalies[:, index]],
        c="red",
        s=10,
    )
    ax[index].set_xlabel("Sample")
    ax[index].set_ylabel(f"Average Feature Value {combined_mapping_dict[index]}")
    ax[index].legend()
plt.show()

# --- Optional: Flag Anomalies ---

# Now test with fixed data
X_test_pred = autoencoder.predict(fixed_data_scaled)
reconstruction_errors = np.mean(np.power(fixed_data_scaled - X_test_pred, 2), axis=(1))
reconstruction_errors = reconstruction_errors[50:-200]
print(f"Mean of reconstruction errors: {np.mean(reconstruction_errors)}")


# --- Optional: Flag Anomalies ---
# You can set a threshold (e.g., based on percentile or statistical properties)
anomalies = np.zeros((fixed_data_scaled.shape[0], num_features), dtype=bool)
for index in range(num_features):
    feature_reconstruction_errors = np.mean(
        np.power(fixed_data_scaled[:, :, index] - X_test_pred[:, :, index], 2), axis=1
    )
    anomalies[:, index] = feature_reconstruction_errors > threshold


anomalies[:INITIAL_CUTOFF] = False
anomalies[-FINAL_CUTOFF:] = False


deviation_metric = 0.00
for index in range(num_features):
    print(
        f"Number of anomalies detected in feature {combined_mapping_dict[index]}: {np.sum(anomalies[:, index])} out of {len(reconstruction_errors)} samples, The current threshold is {threshold}"
    )
    deviation_metric += np.sum(anomalies[:, index]) / len(reconstruction_errors)

print("Deviation Metric: ", deviation_metric)

# Plot the original and reconstructed sequence only per feature
# Create a subplot for each feature
fig, ax = plt.subplots(num_features, 1)
for index in range(num_features):
    original_feature = fixed_data_scaled[:, :, index]
    reconstructed_feature = X_test_pred[:, :, index]
    ax[index].plot(original_feature.mean(axis=1), label="Original (avg over timesteps)")
    ax[index].plot(
        reconstructed_feature.mean(axis=1), label="Reconstructed (avg over timesteps)"
    )
    ax[index].scatter(
        np.where(anomalies[:, index])[0],
        original_feature.mean(axis=1)[anomalies[:, index]],
        c="red",
        s=10,
    )
    ax[index].set_xlabel("Sample")
    ax[index].set_ylabel(f"Average Feature Value {combined_mapping_dict[index]}")
    ax[index].legend()
plt.show()
