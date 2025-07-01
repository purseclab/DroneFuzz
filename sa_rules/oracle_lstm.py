#!/usr/bin/env python3
# -*- coding:utf-8 -*-
###
# Finis coronat opus; Run this at your own peril ~ silipwn
# File: oracle_lstm_pytorch.py
# SPDX-License-Identifier: BSD-3-Clause or GPL-3.0-or-later
# Author: silipwn (contact@as-hw.in)
# Description: For training and testing an LSTM autoencoder for anomaly detection in PyTorch.
# Date: 2025-02-23T07:51:33-0500
# Last-Modified: 2025-06-30T09:41:18-0400
###

# Some notes about the approach
# - The best reconstruction seems to be when we have over >100 samples of data

import argparse
import glob
import os
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import MinMaxScaler
import torch
import torch.nn as nn
import torch.optim as optim
from pymavlink import mavutil
import pickle  # For saving/loading the scaler

# --- Global Constants ---
SELECTED_COLS_RC = ["C1", "C2", "C3", "C4"]
WINDOW_SIZE = 50  # Number of timesteps per sequence
DEFAULT_MODEL_FILENAME = "autoencoder_model.pth"  # Changed extension for PyTorch
DEFAULT_THRESHOLD_FILENAME = "reconstruction_threshold.npy"
DEFAULT_SCALER_FILENAME = "scaler.pkl"
COMBINED_MAPPING_LIST = ["RCOU_C1", "RCOU_C2", "RCOU_C3", "RCOU_C4"]
COMBINED_MAPPING_DICT = {i: v for i, v in enumerate(COMBINED_MAPPING_LIST)}
RANDOM_SEED = 1337
TRAINING_BINS = 75

# Set device for PyTorch (CPU in this case)
DEVICE = torch.device("cpu")
print(f"Using device: {DEVICE}")

# --- Function Definitions ---


def load_rcou_from_bin(file_path, output_channel_names):
    """
    Loads RCOU data from a MAVLink .BIN file, between "LOG RC" and "STOP RC" MSG/STATUSTEXT markers.
    output_channel_names: list of strings, e.g., ["C1", "C2"]
    It assumes output_channel_names[i] corresponds to RCOU.C(i+1).
    Returns a numpy array with the extracted data.
    """
    try:
        mlog = mavutil.mavlink_connection(file_path, robust_parsing=True)
    except Exception as e:
        print(f"Error opening or parsing BIN file {file_path}: {e}")
        return np.empty((0, len(output_channel_names)))

    rcou_entries = []
    num_channels_to_extract = len(output_channel_names)
    logging_active = False
    log_segment_started = False

    while True:
        try:
            msg = mlog.recv_match()
        except Exception as e:
            print(f"Error reading message from BIN file {file_path}: {e}")
            break

        if msg is None:
            break

        msg_type = msg.get_type()
        current_message_str = ""

        if msg_type == "MSG":
            if hasattr(msg, "Message"):
                current_message_str = (
                    str(msg.Message) if msg.Message is not None else ""
                )
        elif msg_type == "STATUSTEXT":
            if hasattr(msg, "text"):
                current_message_str = str(msg.text) if msg.text is not None else ""

        if current_message_str:
            if "LOG RC" in current_message_str:
                if not logging_active:
                    print(
                        f"Found 'LOG RC' in {file_path} (type: {msg_type}). Starting RCOU data logging."
                    )
                    logging_active = True
                    log_segment_started = True
            elif "STOP RC" in current_message_str:
                if logging_active:
                    print(
                        f"Found 'STOP RC' in {file_path} (type: {msg_type}). Stopping RCOU data logging."
                    )
                    logging_active = False

        elif msg_type == "RCOU":
            if logging_active:
                entry = []
                for i in range(num_channels_to_extract):
                    r_field_name = f"C{i+1}"
                    if hasattr(msg, r_field_name):
                        entry.append(getattr(msg, r_field_name))
                    else:
                        entry.append(np.nan)

                if entry:
                    rcou_entries.append(entry)

    if not log_segment_started:
        print(
            f"Warning: 'LOG RC' message not found in {file_path}. No RCOU data logged from this file based on MSG triggers."
        )
    elif logging_active:
        print(
            f"Warning: 'LOG RC' found in {file_path}, but 'STOP RC' not found before end of file. Logged RCOU data until EOF."
        )

    if not rcou_entries:
        return np.empty((0, len(output_channel_names)))

    data_array = np.array(rcou_entries, dtype=np.float64)
    if data_array.size > 0:
        # Remove rows where all values are NaN
        data_array = data_array[~np.all(np.isnan(data_array), axis=1)]
    return data_array


def parse_cli_args():
    """Parses command line arguments."""
    parser = argparse.ArgumentParser(
        description="LSTM autoencoder for anomaly detection"
    )
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
    if len(files) > TRAINING_BINS:
        files = files[:TRAINING_BINS]
    if not files:
        print(
            f"Warning: No {file_pattern} files found in {dataset_name} folder: {folder_path}"
        )
        return np.empty((0, len(selected_cols)))

    all_data = []
    for f_path in files:
        data_array = load_rcou_from_bin(f_path, selected_cols)
        if data_array.size > 0:
            all_data.append(data_array)

    if not all_data:
        print(
            f"Warning: No RCOU data successfully loaded from {dataset_name} .BIN files in {folder_path}."
        )
        return np.empty((0, len(selected_cols)))

    concatenated_data = np.concatenate(all_data, axis=0)
    if concatenated_data.size == 0:
        print(f"Warning: Concatenated RCOU data for {dataset_name} is empty.")
    return concatenated_data


def prepare_main_dataset(args):
    """Loads the main dataset for training."""
    input_folder = args.input_folder

    data_rc = load_dataset_from_bin_files(
        input_folder, "*.BIN", SELECTED_COLS_RC, "main training"
    )
    if data_rc.size == 0:
        print(
            f"No RCOU data successfully loaded from .BIN files in {input_folder} for training. Exiting."
        )
        exit(1)
    return data_rc


def create_sequences(input_data, window_size):
    """Creates time-series sequences from input data."""
    X = []
    if input_data.size == 0 or len(input_data) <= window_size:
        print(
            f"Warning: Data length ({len(input_data)}) is too short for window_size ({window_size}). Cannot create sequences."
        )
        return np.array([])

    input_data_np = input_data
    for i in range(len(input_data_np) - window_size):
        X.append(input_data_np[i : i + window_size])
    return np.array(X)


def scale_training_data(X_seq, num_features):
    """Scales the training dataset using MinMaxScaler and returns the scaled data and the scaler."""
    scaler = MinMaxScaler()
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


# --- PyTorch Model Definition ---
class LSTMAE(nn.Module):
    def __init__(self, seq_len, n_features, hidden_size_enc=64, hidden_size_dec=64):
        super(LSTMAE, self).__init__()
        self.seq_len = seq_len
        self.n_features = n_features
        self.hidden_size_enc = hidden_size_enc
        self.hidden_size_dec = hidden_size_dec

        # Encoder
        self.lstm1_enc = nn.LSTM(
            input_size=n_features, hidden_size=128, batch_first=True
        )
        self.dropout1_enc = nn.Dropout(0.2)
        self.lstm2_enc = nn.LSTM(
            input_size=128, hidden_size=hidden_size_enc, batch_first=True
        )

        # Decoder
        # The decoder takes the last hidden state of the encoder (hidden_size_enc)
        # and "repeats" it for each timestep in the sequence.
        # This is implicitly handled by the LSTM's initial hidden state or by
        # feeding the last encoder output repeatedly. Here, we feed a Linear
        # layer that maps the bottleneck to the decoder's input size.
        self.linear_dec = nn.Linear(
            hidden_size_enc, hidden_size_dec
        )  # Mapping bottleneck to decoder hidden size
        self.lstm1_dec = nn.LSTM(
            input_size=hidden_size_dec, hidden_size=128, batch_first=True
        )
        self.dropout1_dec = nn.Dropout(0.2)
        self.lstm2_dec = nn.LSTM(
            input_size=128, hidden_size=n_features, batch_first=True
        )  # Output layer

    def forward(self, x):
        # Encoder
        # Input shape: (batch_size, seq_len, n_features)
        # hidden_state and cell_state are initialized to zeros by default if not provided
        x, (hidden_state, cell_state) = self.lstm1_enc(x)
        x = self.dropout1_enc(x)
        x, (hidden_state, cell_state) = self.lstm2_enc(
            x
        )  # hidden_state[-1] contains the last hidden state for the last layer.

        # Keras's RepeatVector takes the last output of the previous layer and
        # repeats it for `seq_len` times. In PyTorch, we can use `expand` or `repeat`
        # on the last hidden state/output to create the sequence for the decoder.
        # We'll use the last hidden state of the second encoder LSTM as the "bottleneck".
        # We need the hidden state for the last layer of the encoder.
        # hidden_state shape: (num_layers * num_directions, batch, hidden_size_enc)
        # We take the last layer's hidden state: hidden_state[-1, :, :]
        # Then unsqueeze it to (batch, 1, hidden_size_enc) and expand to (batch, seq_len, hidden_size_enc)

        # Take the hidden state from the last layer of the encoder LSTM
        # (num_layers, batch_size, hidden_size) -> (batch_size, hidden_size)
        bottleneck = hidden_state[-1, :, :]

        # "RepeatVector" equivalent:
        # Expand the bottleneck output to match the sequence length for the decoder
        # (batch_size, hidden_size_enc) -> (batch_size, 1, hidden_size_enc) -> (batch_size, seq_len, hidden_size_enc)
        bottleneck_repeated = bottleneck.unsqueeze(1).expand(-1, self.seq_len, -1)

        # Apply the linear transformation before feeding to decoder LSTM
        x = self.linear_dec(bottleneck_repeated)

        # Decoder
        # Input to decoder is (batch_size, seq_len, hidden_size_dec)
        x, _ = self.lstm1_dec(x)
        x = self.dropout1_dec(x)
        x, _ = self.lstm2_dec(
            x
        )  # Output is (batch_size, seq_len, n_features) for TimeDistributed Dense

        return x


def build_and_train_model_pytorch(
    X_seq_scaled, window_size, num_features, epochs, batch_size, scaler_to_save
):
    """Builds, trains, and evaluates the LSTM autoencoder model in PyTorch."""

    # Convert numpy arrays to PyTorch tensors
    X_seq_scaled_tensor = torch.from_numpy(X_seq_scaled).float().to(DEVICE)

    # Split data into training and validation sets
    X_train_tensor, X_test_val_tensor = train_test_split(
        X_seq_scaled_tensor, test_size=0.2, random_state=RANDOM_SEED
    )

    # Create DataLoader for batching
    train_dataset = torch.utils.data.TensorDataset(
        X_train_tensor, X_train_tensor
    )  # Input and target are the same
    train_loader = torch.utils.data.DataLoader(
        train_dataset, batch_size=batch_size, shuffle=True
    )

    val_dataset = torch.utils.data.TensorDataset(X_test_val_tensor, X_test_val_tensor)
    val_loader = torch.utils.data.DataLoader(
        val_dataset, batch_size=batch_size, shuffle=False
    )

    model = LSTMAE(seq_len=window_size, n_features=num_features).to(DEVICE)
    optimizer = optim.Adam(model.parameters())
    criterion = nn.MSELoss()  # Mean Squared Error Loss

    # Model summary (using torchinfo if installed, or a custom print)
    try:
        from torchinfo import summary

        summary(
            model,
            input_size=(batch_size, window_size, num_features),
            device=str(DEVICE),
        )
    except ImportError:
        print(
            "torchinfo not installed. Install with 'pip install torchinfo' for detailed model summary."
        )
        print(model)  # Fallback to basic model print

    print("\nTraining PyTorch model...")
    history = {"train_loss": [], "val_loss": []}

    for epoch in range(epochs):
        model.train()  # Set model to training mode
        running_loss = 0.0
        for batch_X, batch_y in train_loader:
            optimizer.zero_grad()  # Zero the gradients
            outputs = model(batch_X)
            loss = criterion(outputs, batch_y)
            loss.backward()  # Backpropagation
            optimizer.step()  # Update weights
            running_loss += loss.item() * batch_X.size(0)  # Accumulate batch loss

        epoch_train_loss = running_loss / len(train_loader.dataset)
        history["train_loss"].append(epoch_train_loss)

        # Validation phase
        model.eval()  # Set model to evaluation mode
        val_loss = 0.0
        with torch.no_grad():  # Disable gradient calculations
            for batch_X_val, batch_y_val in val_loader:
                outputs_val = model(batch_X_val)
                loss_val = criterion(outputs_val, batch_y_val)
                val_loss += loss_val.item() * batch_X_val.size(0)

        epoch_val_loss = val_loss / len(val_loader.dataset)
        history["val_loss"].append(epoch_val_loss)

        print(
            f"Epoch {epoch+1}/{epochs}, Train Loss: {epoch_train_loss:.6f}, Val Loss: {epoch_val_loss:.6f}"
        )

    # Calculate reconstruction errors on the validation set
    model.eval()
    with torch.no_grad():
        X_test_val_pred = model(X_test_val_tensor)

    # Calculate reconstruction errors as Mean Absolute Error (MAE)
    reconstruction_errors = torch.mean(
        torch.abs(X_test_val_tensor - X_test_val_pred), dim=(1, 2)
    )

    # Calculate threshold (3-sigma rule)
    mean_error = torch.mean(reconstruction_errors).item()
    std_error = torch.std(reconstruction_errors).item()
    threshold = mean_error + 3 * std_error
    print(f"Reconstruction error threshold for anomaly detection: {threshold}")

    # Plot histogram of reconstruction errors
    plt.figure(figsize=(8, 4))
    plt.hist(reconstruction_errors.cpu().numpy(), bins=50)  # Move to CPU for plotting
    plt.xlabel("Reconstruction error")
    plt.ylabel("Frequency")
    plt.title("Histogram of Reconstruction Errors on Validation Set")
    plt.show()

    # Plot original vs. reconstructed sequence (validation sample)
    idx = np.random.randint(0, X_test_val_tensor.shape[0])
    original_seq = X_test_val_tensor[idx].cpu().numpy()
    reconstructed_seq = X_test_val_pred[idx].cpu().numpy()

    plt.figure(figsize=(10, 6))
    for i in range(num_features):
        plt.subplot(num_features, 1, i + 1)
        plt.plot(
            original_seq[:, i],
            label=f"Original {COMBINED_MAPPING_DICT.get(i, f'Feature {i+1}')}",
        )
        plt.plot(
            reconstructed_seq[:, i],
            label=f"Reconstructed {COMBINED_MAPPING_DICT.get(i, f'Feature {i+1}')}",
        )
        plt.ylabel(COMBINED_MAPPING_DICT.get(i, f"Feature {i+1}"))
        if i == 0:
            plt.title("Original vs. Reconstructed Sequence (Validation Sample)")
        if i < num_features - 1:
            plt.xticks([])
    plt.xlabel("Timestep")
    plt.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.15 * num_features),
        fancybox=True,
        shadow=True,
        ncol=2,
    )
    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    plt.show()

    # Save the model state dictionary
    torch.save(model.state_dict(), DEFAULT_MODEL_FILENAME)
    np.save(DEFAULT_THRESHOLD_FILENAME, threshold)
    print(f"Model saved to {DEFAULT_MODEL_FILENAME}")
    print(f"Threshold saved to {DEFAULT_THRESHOLD_FILENAME}")

    # Save the scaler
    with open(DEFAULT_SCALER_FILENAME, "wb") as f:
        pickle.dump(scaler_to_save, f)
    print(f"Scaler saved to {DEFAULT_SCALER_FILENAME}")

    return model, threshold


def load_existing_model_threshold_and_scaler(
    model_path, threshold_path, scaler_path, window_size, num_features
):
    """Loads an existing autoencoder model, threshold, and scaler."""
    print(f"Loading existing model from {model_path}...")
    model = LSTMAE(seq_len=window_size, n_features=num_features).to(DEVICE)
    model.load_state_dict(torch.load(model_path, map_location=DEVICE))
    model.eval()  # Set to evaluation mode

    print(f"Loading existing threshold from {threshold_path}...")
    threshold = np.load(threshold_path)

    print(f"Loading existing scaler from {scaler_path}...")
    with open(scaler_path, "rb") as f:
        scaler = pickle.load(f)
    print(f"Loaded model, threshold: {threshold}, and scaler.")
    return model, threshold, scaler


def plot_comparison_results(
    data_scaled,
    predictions,
    errors,
    threshold,
    data_type_name,
    num_features,
):
    """Plots results for comparison data (anomaly or fixed)."""
    if data_scaled.size == 0:
        print(f"No scaled {data_type_name} data to plot.")
        return

    # Errors are already sequence-based, no initial/final cutoff needed for this plot
    # The original code's initial_cutoff and final_cutoff were for a very specific
    # filtering approach that isn't directly related to how `create_sequences` works.
    # We'll just use the `errors` as is.
    threshold_tensor = torch.tensor(threshold, dtype=errors.dtype, device=DEVICE)

    mean_error = (
        torch.mean(errors).item() if errors.numel() > 0 else "N/A"
    )  # .numel() for PyTorch tensor size
    print(f"Mean of reconstruction errors for {data_type_name} data: {mean_error}")

    anomalies_detected_mask = errors > threshold_tensor

    plt.figure(figsize=(12, 8))

    # Plot 1: Reconstruction Errors vs. Threshold
    plt.subplot(2, 1, 1)
    plt.plot(
        errors.cpu().numpy(), label="Reconstruction Error", color="blue", alpha=0.7
    )
    plt.axhline(threshold, color="r", linestyle="--", label="Anomaly Threshold")

    # Plot anomalies
    anomaly_indices = torch.where(anomalies_detected_mask)[0]
    plt.scatter(
        anomaly_indices.cpu().numpy(),
        errors[anomalies_detected_mask].cpu().numpy(),
        color="red",
        label="Detected Anomaly Point",
        s=50,
    )
    plt.title(f"Reconstruction Errors for {data_type_name} Data")
    plt.xlabel("Sequence Index")
    plt.ylabel("MAE Error")  # Changed from Squared Error to MAE
    plt.legend()

    # Plot 2: Example of Original vs. Reconstructed Sequence from this dataset
    plt.subplot(2, 1, 2)
    idx_example = 0
    title_suffix = ""
    if anomalies_detected_mask.any():
        idx_example = torch.where(anomalies_detected_mask)[0][
            0
        ].item()  # First detected anomaly
        title_suffix = f"(Sequence {idx_example} - Detected Anomaly)"
    elif data_scaled.shape[0] > 0:
        idx_example = torch.randint(
            0, data_scaled.shape[0], (1,)
        ).item()  # Random sample
        title_suffix = f"(Sequence {idx_example} - Random Sample)"
    else:
        print(f"No data for example plot in {data_type_name}.")
        plt.title(
            f"Original vs. Reconstructed - {data_type_name} (No data for example)"
        )
        plt.tight_layout()
        plt.show()
        return

    original_seq_example = data_scaled[idx_example].cpu().numpy()
    reconstructed_seq_example = predictions[idx_example].cpu().numpy()

    for i in range(num_features):
        plt.plot(
            original_seq_example[:, i],
            label=f"Original {COMBINED_MAPPING_DICT.get(i, f'F{i+1}')}",
            alpha=0.7,
            linestyle="--",
        )
        plt.plot(
            reconstructed_seq_example[:, i],
            label=f"Reconstructed {COMBINED_MAPPING_DICT.get(i, f'F{i+1}')}",
            alpha=0.7,
        )

    plt.title(f"Original vs. Reconstructed - {data_type_name} {title_suffix}")
    plt.xlabel("Timestep in Sequence")
    plt.ylabel("Scaled Value")
    plt.legend(fontsize="small")

    plt.tight_layout()
    plt.savefig("comparison_plot.png")  # Save the plot to a file

    # Deviation Metric
    deviation_metric = 0.0
    num_samples_for_metric = errors.numel()  # Use numel for PyTorch tensor
    if num_samples_for_metric > 0:
        above_threshold_errors = errors[errors > threshold_tensor]
        if above_threshold_errors.numel() > 0:
            deviation_metric = (
                torch.sum(above_threshold_errors).item() / num_samples_for_metric
            )
        else:
            deviation_metric = 0.0
    else:
        deviation_metric = 0.0
    print(f"{data_type_name} Data Deviation Metric: {deviation_metric:.4f}")


def main():
    """Main function to orchestrate the LSTM autoencoder workflow."""
    args = parse_cli_args()

    # --- Early Exit if Model Exists and No Comparison is Requested ---
    if (
        os.path.isfile(DEFAULT_MODEL_FILENAME)
        and os.path.isfile(DEFAULT_THRESHOLD_FILENAME)
        and os.path.isfile(DEFAULT_SCALER_FILENAME)
        and args.compare_file is None
    ):
        print(
            f"Model '{DEFAULT_MODEL_FILENAME}', threshold '{DEFAULT_THRESHOLD_FILENAME}', and scaler '{DEFAULT_SCALER_FILENAME}' already exist."
        )
        try:
            loaded_threshold = np.load(DEFAULT_THRESHOLD_FILENAME)
            print(f"Existing threshold: {loaded_threshold}")
        except Exception as e:
            print(f"Could not load existing threshold to display: {e}")
        print("No comparison file provided. Nothing further to do. Exiting.")
        exit(0)

    # Set seeds for reproducibility
    torch.manual_seed(RANDOM_SEED)
    np.random.seed(RANDOM_SEED)

    autoencoder = None
    threshold = None
    scaler = None

    expected_num_features = len(SELECTED_COLS_RC)
    active_num_features = expected_num_features

    # --- Load Model and Scaler if Existing, or Train New Model ---
    if (
        os.path.isfile(DEFAULT_MODEL_FILENAME)
        and os.path.isfile(DEFAULT_THRESHOLD_FILENAME)
        and os.path.isfile(DEFAULT_SCALER_FILENAME)
    ):
        print(
            f"Loading existing model ('{DEFAULT_MODEL_FILENAME}'), threshold ('{DEFAULT_THRESHOLD_FILENAME}'), and scaler ('{DEFAULT_SCALER_FILENAME}')..."
        )
        # We need the num_features and window_size to instantiate the model before loading state_dict
        # A common practice is to save these as part of the model metadata or in a config file
        # For simplicity here, we assume SELECTED_COLS_RC is consistent with the trained model's features.
        # If your model's features can change, you'd need to save/load 'num_features' too.
        autoencoder, threshold, scaler = load_existing_model_threshold_and_scaler(
            DEFAULT_MODEL_FILENAME,
            DEFAULT_THRESHOLD_FILENAME,
            DEFAULT_SCALER_FILENAME,
            WINDOW_SIZE,
            active_num_features,
        )

        # PyTorch model doesn't have a direct `input_shape` attribute like Keras.
        # We can infer it from the first layer or ensure consistency.
        # For an LSTM AE, the last dimension of the first LSTM's input corresponds to n_features.
        # This check is more heuristic for PyTorch, assuming the loaded model aligns.
        # If the number of features can change when loading, you'd need to pass it explicitly
        # when defining LSTMAE, or save it with the model.
        # For now, we trust active_num_features as derived from SELECTED_COLS_RC.
        print(
            f"Model expects {active_num_features} features, consistent with SELECTED_COLS_RC. Scaler is loaded."
        )

    else:
        print(
            "No pre-trained model/threshold/scaler found. Preparing data for training a new model."
        )
        # --- Step 1: Load Main Dataset for Training ---
        data_main_processed = prepare_main_dataset(args)

        if data_main_processed.size == 0:
            print(
                "Main dataset is empty after loading. Cannot proceed with training. Exiting."
            )
            exit(1)

        if data_main_processed.shape[1] != active_num_features:
            print(
                f"Warning: Training data has {data_main_processed.shape[1]} features, while SELECTED_COLS_RC implies {active_num_features}. Using actual data features: {data_main_processed.shape[1]}."
            )
            active_num_features = data_main_processed.shape[1]
        else:
            print(
                f"Training data has {active_num_features} features, consistent with SELECTED_COLS_RC."
            )

        # --- Step 2: Create Time-Series Sequences for Main Data ---
        X_seq = create_sequences(data_main_processed, WINDOW_SIZE)
        if X_seq.size == 0:
            print("Failed to create training sequences from main data. Exiting.")
            exit(1)

        # --- Step 3: Scale the Training Data ---
        X_seq_scaled, scaler = scale_training_data(X_seq, active_num_features)

        # --- Step 4 & 5 (part 2): Train the model ---
        print(f"Building and training new model with {active_num_features} features...")
        autoencoder, threshold = build_and_train_model_pytorch(
            X_seq_scaled,
            WINDOW_SIZE,
            active_num_features,
            args.epochs,
            args.batch_size,
            scaler,
        )

    # --- Step 6: Perform Comparison Analysis (if requested) ---
    if args.compare_file:
        if autoencoder is None or threshold is None:
            print("Error: Model or threshold not available for comparison. Exiting.")
            exit(1)
        if scaler is None:
            print(
                "Error: Scaler not available for comparison. This can happen if model files existed but scaler derivation failed or scaler file is missing. Exiting."
            )
            exit(1)

        print(f"\n--- Analyzing Comparison File: {args.compare_file} ---")
        if not os.path.isfile(args.compare_file):
            print(
                f"Error: Comparison file not found: {args.compare_file}. Skipping comparison."
            )
        else:
            data_compare_processed = load_rcou_from_bin(
                args.compare_file, SELECTED_COLS_RC
            )

            if data_compare_processed.size == 0:
                print(
                    f"No data loaded from comparison file {args.compare_file} (MSG-based filtering). Skipping comparison."
                )
            elif data_compare_processed.shape[1] != active_num_features:
                print(
                    f"Error: Comparison data from {args.compare_file} has {data_compare_processed.shape[1]} features, but model/scaler expects {active_num_features}. Skipping comparison."
                )
            elif len(data_compare_processed) <= WINDOW_SIZE:
                print(
                    f"Comparison file {args.compare_file} has insufficient data (length {len(data_compare_processed)}) for window size ({WINDOW_SIZE}) after MSG-based filtering. Skipping comparison."
                )
            else:
                compare_seq = create_sequences(data_compare_processed, WINDOW_SIZE)
                if compare_seq.size > 0:
                    compare_data_scaled_np = scale_comparison_data(
                        compare_seq, scaler, active_num_features
                    )
                    if compare_data_scaled_np.size > 0:
                        compare_data_scaled_tensor = (
                            torch.from_numpy(compare_data_scaled_np).float().to(DEVICE)
                        )

                        autoencoder.eval()  # Set model to evaluation mode
                        with torch.no_grad():
                            predictions_compare = autoencoder(
                                compare_data_scaled_tensor
                            )

                        errors_compare = torch.mean(
                            torch.abs(compare_data_scaled_tensor - predictions_compare),
                            dim=(1, 2),
                        )

                        plot_title_name = os.path.basename(args.compare_file)
                        plot_comparison_results(
                            compare_data_scaled_tensor,  # Pass the tensor
                            predictions_compare,  # Pass the tensor
                            errors_compare,  # Pass the tensor
                            threshold,
                            plot_title_name,
                            active_num_features,
                        )
                    else:
                        print(
                            f"Failed to scale comparison data from {args.compare_file}. Skipping comparison plotting."
                        )
                else:
                    print(
                        f"No sequences created from {args.compare_file} after processing. Skipping comparison."
                    )
    else:
        print("\nTraining/loading completed.")
        print(
            f"Model: {DEFAULT_MODEL_FILENAME}, Threshold for anomaly detection: {threshold}"
        )
        print(
            "Use --compare_file <path_to_bin_file> to analyze a specific BIN file with this model."
        )


if __name__ == "__main__":
    main()
