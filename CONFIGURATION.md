# DroneFuzz++ Configuration Guide

This guide provides detailed information on configuring DroneFuzz++ for various fuzzing scenarios and vehicle types.

## Configuration Overview

DroneFuzz++ supports two main configuration approaches:
1. **YAML Configuration Files** (Recommended)
2. **Command-line Arguments**

Configuration files provide better organization and reusability, while command-line arguments offer quick customization for testing.

## YAML Configuration Structure

### Core Configuration Sections

```yaml
# =============================================================================
# BASIC SIMULATION SETTINGS
# =============================================================================
sitl_bin: "/path/to/ardupilot/build/sitl/bin/arducopter"
ap_dir: "/path/to/ardupilot/"
xml_file: "/path/to/leanfuzzer/xmls/ardupilotmega.xml"
peripheral_file: "/path/to/leanfuzzer/configs/mapping.yaml"

# =============================================================================
# VEHICLE AND MISSION CONFIGURATION
# =============================================================================
vehicle: "copter"                    # Vehicle type: copter, plane, rover
peripheral: "CS5"                    # Peripheral identifier for fuzzing
supported_modes: ["AUTO", "GUIDED"]  # Flight modes to test
mission_file: "/path/to/leanfuzzer/missions/mission.txt"

# =============================================================================
# FUZZING PARAMETERS
# =============================================================================
fuzz_interval: 1.0                  # Message sending interval (seconds)
calibration_rounds: 20              # Baseline calibration runs
timeout: 1000                       # Overall timeout (seconds)

# =============================================================================
# ORACLE CONFIGURATION
# =============================================================================
oracle_model: "dtw"                 # Options: "dtw", "lstm"
dtw_threshold: 0.5                  # DTW anomaly detection threshold

# =============================================================================
# ADVANCED PARAMETER FUZZING
# =============================================================================
generic_params:
  - "PARAM_NAME MIN_VAL MAX_VAL [STEP]"
  - "ANOTHER_PARAM MIN_VAL MAX_VAL"
```

## Configuration File Examples

### Basic Copter Configuration

```yaml
# configs/basic_copter.yaml
sitl_bin: "/home/user/ardupilot/build/sitl/bin/arducopter"
ap_dir: "/home/user/ardupilot/"
xml_file: "./xmls/ardupilotmega.xml"
peripheral_file: "./configs/mapping.yaml"

vehicle: "copter"
peripheral: "CS5"
supported_modes: ["GUIDED"]
mission_file: "./missions/mission_copter.txt"

fuzz_interval: 2.0
calibration_rounds: 10
```

### Advanced Parameter Fuzzing Configuration

```yaml
# configs/advanced_param_fuzzing.yaml
sitl_bin: "/home/user/ardupilot/build/sitl/bin/arducopter"
ap_dir: "/home/user/ardupilot/"
xml_file: "./xmls/ardupilotmega.xml"
peripheral_file: "./configs/mapping.yaml"

vehicle: "copter"
peripheral: "CS5"

# Parameter fuzzing with constraints
generic_params:
  # Vibration frequency parameters (discrete steps)
  - "SIM_VIB_FREQ_X 10.0 100.0 5.0"
  - "SIM_VIB_FREQ_Y 10.0 100.0 5.0"
  - "SIM_VIB_FREQ_Z 10.0 100.0 5.0"
  
  # Motor vibration multiplier (continuous range)
  - "SIM_VIB_MOT_MULT 0.0 2.0"
  
  # AHRS trim parameters (small continuous range)
  - "AHRS_TRIM_X -0.5 0.5"
  - "AHRS_TRIM_Y -0.5 0.5"
  
  # GPS and compass parameters (heuristic-based)
  - "GPS_AUTO_SWITCH"
  - "COMPASS_EXTERNAL"
  - "LOG_BITMASK"

oracle_model: "dtw"
dtw_threshold: 0.3
calibration_rounds: 25
```

### LSTM Oracle Configuration

```yaml
# configs/lstm_oracle.yaml
sitl_bin: "/home/user/ardupilot/build/sitl/bin/arducopter"
ap_dir: "/home/user/ardupilot/"
xml_file: "./xmls/ardupilotmega.xml"
peripheral_file: "./configs/mapping.yaml"

vehicle: "copter"
peripheral: "CS5"

# LSTM-based anomaly detection
oracle_model: "lstm"
lstm_model_path: "./models/trained_autoencoder.pth"
lstm_threshold: 0.05
lstm_sequence_length: 50

calibration_rounds: 30  # More calibration needed for LSTM training
```

### Fixed-Wing Aircraft Configuration

```yaml
# configs/plane_config.yaml
sitl_bin: "/home/user/ardupilot/build/sitl/bin/arduplane"
ap_dir: "/home/user/ardupilot/"
xml_file: "./xmls/ardupilotmega.xml"
peripheral_file: "./configs/mapping.yaml"

vehicle: "plane"
peripheral: "ADSB"
supported_modes: ["AUTO", "FBWA", "GUIDED"]
mission_file: "./missions/mission_plane.txt"

# Plane-specific parameters
generic_params:
  - "ARSPD_FBW_MIN 10.0 30.0"
  - "ARSPD_FBW_MAX 30.0 60.0"
  - "TRIM_THROTTLE 0.3 0.7"
  - "L1_PERIOD 15.0 25.0"

fuzz_interval: 1.5
calibration_rounds: 15
```

## Peripheral Mapping Configuration

The peripheral mapping file defines which MAVLink messages to fuzz and their characteristics.

### Basic Mapping Structure

```yaml
# configs/mapping.yaml
CS5:  # Peripheral identifier
  messages:
    - name: "ATTITUDE"
      frequency: 10      # Hz
      enabled: true
      priority: "high"
    
    - name: "GPS_RAW_INT"
      frequency: 4
      enabled: true
      priority: "medium"
    
    - name: "RC_CHANNELS"
      frequency: 20
      enabled: false     # Disabled for this test
      priority: "low"

ADSB:  # Different peripheral
  messages:
    - name: "ADSB_VEHICLE"
      frequency: 2
      enabled: true
      priority: "high"
```

### Advanced Mapping with Message Filtering

```yaml
# configs/advanced_mapping.yaml
CS5:
  messages:
    - name: "ATTITUDE"
      frequency: 10
      enabled: true
      priority: "high"
      # Field-specific configuration
      fields:
        roll: { fuzz: true, range: [-3.14, 3.14] }
        pitch: { fuzz: true, range: [-1.57, 1.57] }
        yaw: { fuzz: false }  # Don't fuzz yaw
    
    - name: "GPS_RAW_INT"
      frequency: 4
      enabled: true
      priority: "medium"
      fields:
        lat: { fuzz: true, range: [-90000000, 90000000] }  # degE7
        lon: { fuzz: true, range: [-180000000, 180000000] } # degE7
        alt: { fuzz: true, range: [0, 100000] }  # mm
```

## Parameter Fuzzing Configuration

### Parameter Constraint Formats

DroneFuzz++ supports several parameter constraint formats:

1. **Range with Step**: `"PARAM_NAME MIN MAX STEP"`
   ```yaml
   - "SIM_VIB_FREQ_X 10.0 100.0 5.0"  # Values: 10.0, 15.0, 20.0, ..., 100.0
   ```

2. **Continuous Range**: `"PARAM_NAME MIN MAX"`
   ```yaml
   - "AHRS_TRIM_X -0.5 0.5"  # Any float between -0.5 and 0.5
   ```

### Common Parameter Categories

#### Sensor Parameters
```yaml
generic_params:
  # IMU/Gyro parameters
  - "INS_GYRO_FILTER 20.0 100.0"
  - "INS_ACCEL_FILTER 10.0 50.0"
  
  # Barometer parameters
  - "BARO_PRIMARY 0 2 1"
  - "BARO_ALT_OFFSET -100.0 100.0"
  
  # Compass parameters
  - "COMPASS_AUTO_ROT 0 4 1"
  - "COMPASS_DEC -180.0 180.0"
```

#### Control Parameters
```yaml
generic_params:
  # PID controller parameters
  - "ATC_RAT_RLL_P 0.05 0.3"
  - "ATC_RAT_PIT_P 0.05 0.3"
  - "ATC_RAT_YAW_P 0.1 0.5"
  
  # Motor parameters
  - "MOT_THST_EXPO 0.2 0.8"
  - "MOT_PWM_MIN 1000 1200 50"
  - "MOT_PWM_MAX 1800 2000 50"
```

#### Navigation Parameters
```yaml
generic_params:
  # GPS parameters
  - "GPS_TYPE 1 14 1"
  - "GPS_AUTO_SWITCH 0 1 1"
  
  # EKF parameters
  - "EK2_ALT_NOISE 0.1 2.0"
  - "EK2_GPS_DELAY 100 300 50"
```

## Mission Configuration

### Mission File Format

Mission files use the QGroundControl waypoint format:

```
QGC WPL 110
# seq curr frame cmd p1 p2 p3 p4 lat lon alt continue
0   1    0     16  0  0  0  0  -35.363261 149.165230 584.000000 1
1   0    3     22  0  0  0  0  0          0          20.000000  1
2   0    3     16  0  0  0  0  -35.363261 149.165230 20.000000  1
3   0    3     21  0  0  0  0  0          0          0          1
```

### Mission Types

#### Simple Takeoff and Land
```
QGC WPL 110
0	1	0	16	0	0	0	0	LAT	LON	ALT	1
1	0	3	22	0	0	0	0	0	0	20	1    # Takeoff to 20m
2	0	3	21	0	0	0	0	0	0	0	1     # Land
```

#### Waypoint Mission
```
QGC WPL 110
0	1	0	16	0	0	0	0	LAT	LON	ALT	1
1	0	3	22	0	0	0	0	0	0	20	1         # Takeoff
2	0	3	16	0	0	0	0	LAT1	LON1	20	1    # Waypoint 1
3	0	3	16	0	0	0	0	LAT2	LON2	25	1    # Waypoint 2
4	0	3	20	0	0	0	0	LAT	LON	0	1       # RTL
```

## Oracle Model Configuration

### DTW (Dynamic Time Warping) Oracle

```yaml
oracle_model: "dtw"
dtw_threshold: 0.5        # Similarity threshold (0-1, lower = more sensitive)
dtw_window_size: 100      # Number of samples to compare
dtw_features:             # Which signals to monitor
  - "servo_output"
  - "attitude"
  - "position"
```

### LSTM Autoencoder Oracle (TODO)

```yaml
oracle_model: "lstm"
lstm_model_path: "./models/autoencoder.pth"
lstm_threshold: 0.05      # Reconstruction error threshold
lstm_sequence_length: 50  # Input sequence length
lstm_features: 64         # Hidden layer size
lstm_learning_rate: 0.001 # Training learning rate
```

## Environment-Specific Configuration

### Development Environment
```yaml
# configs/dev_config.yaml
sitl_bin: "/home/dev/ardupilot/build/sitl/bin/arducopter"
ap_dir: "/home/dev/ardupilot/"
xml_file: "./xmls/ardupilotmega.xml"
peripheral_file: "./configs/mapping.yaml"

vehicle: "copter"
peripheral: "CS5"

# Fast development settings
calibration_rounds: 5
fuzz_interval: 0.5
timeout: 300

# Enable verbose logging
log_level: "DEBUG"
```

### Common Configuration Issues

1. **Path Issues**
   - Use absolute paths or paths relative to the script location
   - Verify all file paths exist and are accessible

2. **SITL Binary Issues**
   - Ensure the SITL binary is built and executable
   - Check that the vehicle type matches the binary (arducopter vs arduplane)

3. **Parameter Constraints**
   - Verify parameter names exist in the vehicle firmware
   - Check that min/max values are within acceptable ranges

4. **Mission File Format**
   - Ensure mission files follow QGC WPL format
   - Validate coordinates are in correct decimal degrees format

## Configuration Best Practices

1. **Use Version Control**: Keep configuration files in version control
2. **Environment-Specific Configs**: Maintain separate configs for dev/test/prod
3. **Document Changes**: Comment configuration changes and reasoning
4. **Validate Before Running**: Always test configurations before long runs
5. **Monitor Resource Usage**: Adjust parameters based on system capabilities
6. **Backup Calibration Data**: Save calibration results for reuse

## Troubleshooting

### Common Issues and Solutions

1. **Connection Timeouts**
   ```yaml
   # Increase timeout values
   timeout: 2000
   connection_timeout: 30
   ```

2. **High False Positive Rate**
   ```yaml
   # Increase detection thresholds
   dtw_threshold: 0.7
   calibration_rounds: 30
   ```

For additional help, check the logs and ensure all dependencies are properly installed.