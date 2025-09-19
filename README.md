# DroneFuzz++

DroneFuzz++ is an advanced fuzzing framework designed to test UAV (Unmanned Aerial Vehicle) systems, specifically targeting ArduPilot-based drones. It performs intelligent fuzzing of MAVLink messages to discover potential vulnerabilities and edge cases in drone flight control systems.

## Table of Contents

- [Quick Start](#quick-start)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Supported Vehicles](#supported-vehicles)
- [Oracle Models](#oracle-models)
- [Documentation](#documentation)
- [Contributing](#contributing)

## Quick Start

**New to DroneFuzz++?** Follow the [Quick Start Guide](QUICKSTART.md) to get up and running in minutes.

For detailed configuration options, see the [Configuration Guide](CONFIGURATION.md).

## Installation

### Prerequisites

- Python 3.8 or higher
- ArduPilot SITL simulator
- Git

### Dependencies Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd leanfuzzer
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

### ArduPilot SITL Setup

1. Clone and build ArduPilot:
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
./Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

2. Note the path to your SITL binary (typically `ardupilot/build/sitl/bin/arducopter`)

## Configuration

DroneFuzz++ can be configured using YAML configuration files or command-line arguments. The configuration system is flexible and supports various fuzzing scenarios.

### Basic Configuration

#### Using Configuration Files (Recommended)

Create a configuration file based on the example in `configs/config.yaml`:

```yaml
# Basic SITL configuration
sitl_bin: "/path/to/ardupilot/build/sitl/bin/arducopter"
ap_dir: "/path/to/ardupilot/"
xml_file: "/path/to/leanfuzzer/xmls/ardupilotmega.xml"
peripheral_file: "/path/to/leanfuzzer/configs/mapping.yaml"

# Vehicle and mission settings
vehicle: "copter"  # Options: copter, plane, rover
peripheral: "CS5"  # Peripheral identifier for fuzzing
supported_modes: ["AUTO", "GUIDED"]
mission_file: "/path/to/leanfuzzer/missions/mission.txt"

# Fuzzing parameters
fuzz_interval: 1.0  # Message sending interval in seconds
calibration_rounds: 20  # Number of calibration runs for baseline
```

#### Command Line Usage

```bash
python minimal_poc.py --config configs/config.yaml
```

Or specify parameters directly:

```bash
python minimal_poc.py \
    --bin /path/to/ardupilot/build/sitl/bin/arducopter \
    --ap_dir /path/to/ardupilot/ \
    --xml xmls/ardupilotmega.xml \
    --peripheral_file configs/mapping.yaml \
    --peripheral CS5 \
    --vehicle copter
```

### Advanced Configuration Options

#### Generic Parameters Fuzzing

Configure parameter fuzzing using the `generic_params` section:

```yaml
# Example: configs/example_generic_params.yaml
generic_params:
  # Parameter with MIN, MAX, and STEP constraints
  - "SIM_VIB_FREQ_X 10.0 100.0 5.0"
  - "SIM_VIB_FREQ_Y 10.0 100.0 5.0"
  
  # Parameter with MIN and MAX only (continuous range)
  - "SIM_VIB_MOT_MULT 0.0 2.0"
  - "AHRS_TRIM_X -0.5 0.5"
  
  # Parameter without constraints (uses heuristics)
  - "LOG_BITMASK"
  - "SERVO1_FUNCTION"
```

#### Oracle Model Configuration

Choose between different anomaly detection models:

```yaml
# DTW-based oracle (default)
oracle_model: "dtw"
dtw_threshold: 0.5

# LSTM autoencoder oracle
oracle_model: "lstm"
lstm_model_path: "/path/to/trained/model.pth"
```

#### Mission Configuration

Configure mission parameters:

```yaml
mission_file: "/path/to/mission.txt"
supported_modes: ["AUTO", "GUIDED"]  # Supported flight modes
takeoff_altitude: 20  # Meters
```

### Configuration File Examples

The `configs/` directory contains several example configurations:

- `config.yaml` - Basic configuration template
- `copter.yaml` - Copter-specific settings
- `plane.yaml` - Fixed-wing aircraft settings
- `rover.yaml` - Ground vehicle settings
- `example_generic_params.yaml` - Parameter fuzzing examples

### Environment Variables

Set the MAVLink protocol version:
```bash
export MAVLINK20=1
```

### Command Line Arguments Reference

| Argument | Description | Required |
|----------|-------------|----------|
| `--config` | Path to YAML configuration file | No* |
| `--bin` | Path to SITL binary | Yes* |
| `--ap_dir` | ArduPilot source directory | Yes* |
| `--xml` | MAVLink XML definition file | Yes* |
| `--peripheral_file` | Peripheral mapping YAML file | Yes* |
| `--peripheral` | Peripheral identifier to fuzz | Yes* |
| `--vehicle` | Vehicle type (copter/plane/rover) | No |
| `--auto_mission` | Auto mission file path | No |
| `--calibration_rounds` | Number of calibration rounds | No |
| `--dtw_threshold` | DTW anomaly threshold | No |

*Required when `--config` is not provided

### Peripheral Mapping

The peripheral mapping file (`configs/mapping.yaml`) defines which messages to fuzz and their frequencies:

```yaml
# Example peripheral mapping
CS5:
  messages:
    - name: "ATTITUDE"
      frequency: 10
      enabled: true
    - name: "GPS_RAW_INT"
      frequency: 5
      enabled: true
```

### Mission Files

Mission files define waypoints and commands for the drone:

```
QGC WPL 110
0	1	0	16	0	0	0	0	-35.363261	149.165230	584.000000	1
1	0	3	22	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	20.00000000	1
2	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.363261	149.165230	20.00000000	1
```

## Usage

### Basic Fuzzing Run

1. Start a basic fuzzing session:
```bash
python minimal_poc.py --config configs/config.yaml
```

2. The fuzzer will:
   - Start the SITL simulator
   - Establish MAVLink connection
   - Run calibration rounds to establish baseline
   - Begin fuzzing with intelligent mutations
   - Monitor for anomalies using the configured oracle

### Monitoring Progress

The fuzzer provides real-time feedback including:
- Calibration progress with tqdm progress bars
- Mission execution status
- Anomaly detection results
- Coverage statistics
- Error handling and recovery

### Output

DroneFuzz++ generates several types of output:
- **Log files**: Detailed execution logs with timestamps
- **Coverage data**: Code coverage information
- **Anomaly reports**: Detected anomalies with context
- **Statistics**: Fuzzing statistics and performance metrics

## Project Structure

```
leanfuzzer/
├── configs/          # Configuration files
├── missions/         # Mission definition files
├── xmls/            # MAVLink XML definitions
├── scripts/         # Utility scripts
├── sa_rules/        # Static analysis rules
├── notes/           # Documentation and notes
├── minimal_poc.py   # Main fuzzer implementation
├── requirements.txt # Python dependencies
└── README.md       # This file
```

## Supported Vehicles

- **Copter**: Multi-rotor aircraft (quadcopters, hexacopters, etc.)
- **Plane**: Fixed-wing aircraft
- **Rover**: Ground vehicles

## Oracle Models

### DTW (Dynamic Time Warping)
- Compares servo output patterns against baseline
- Good for detecting timing and sequence anomalies
- Fast and lightweight

### LSTM Autoencoder
- Deep learning-based anomaly detection
- Learns normal flight patterns during calibration
- More sensitive to subtle anomalies

## Documentation

- **[Quick Start Guide](QUICKSTART.md)** - Get up and running quickly
- **[Configuration Guide](CONFIGURATION.md)** - Comprehensive configuration documentation
- **[Example Configurations](configs/)** - Ready-to-use configuration files
- **[Mission Files](missions/)** - Example mission definitions

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Update documentation as needed
6. Submit a pull request

## License


## Support

For issues and questions:
- Start with the [Quick Start Guide](QUICKSTART.md)
- Check the [Configuration Guide](CONFIGURATION.md) for detailed setup
- Review configuration examples in `configs/`
- Examine the logs for detailed error information
- Check existing issues on GitHub
