# DroneFuzz++ Quick Start Guide

This guide will help you get DroneFuzz++ up and running quickly.

## Prerequisites Checklist

- [ ] Python 3.8+ installed
- [ ] Git installed
- [ ] Ubuntu/Linux environment (recommended)
- [ ] At least 4GB RAM available
- [ ] 10GB free disk space

## 1. Install ArduPilot SITL

```bash
# Clone ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Install dependencies
git submodule update --init --recursive
./Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile

# Build SITL for copter
./waf configure --board sitl
./waf copter

# Test SITL (optional)
./build/sitl/bin/arducopter --help
```

## 2. Setup DroneFuzz++

```bash
# Clone DroneFuzz++
git clone <repository-url>
cd leanfuzzer

# Install Python dependencies
pip install -r requirements.txt
```

## 3. Quick Configuration

Create a basic configuration file:

```bash
# Copy example configuration
cp configs/config.yaml configs/my_config.yaml
```

Edit `configs/my_config.yaml` with your paths:

```yaml
# Update these paths to match your setup
sitl_bin: "/home/user/ardupilot/build/sitl/bin/arducopter"
ap_dir: "/home/user/ardupilot/"
xml_file: "/home/user/leanfuzzer/xmls/ardupilotmega.xml"
peripheral_file: "/home/user/leanfuzzer/configs/mapping.yaml"

# Basic settings
vehicle: "copter"
peripheral: "CS5"
supported_modes: ["GUIDED"]
mission_file: "/home/user/leanfuzzer/missions/mission_copter.txt"

# Quick test settings
fuzz_interval: 2.0
calibration_rounds: 5
```

## 4. First Run

```bash
# Test the configuration
python minimal_poc.py --config configs/my_config.yaml
```

You should see output like:
```
Logging initialized. Log file: dronefuzz_20241218_143022.log
Starting SITL simulation...
Waiting for heartbeat...
Connected to vehicle
Beginning calibration
Calibration Progress: 0%|          | 0/5 [00:00<?, ?it/s]
```

## 5. Understanding the Output

### Calibration Phase
- DroneFuzz++ first runs baseline missions to establish normal behavior
- Progress is shown with a progress bar
- Each calibration round tests a complete mission

### Fuzzing Phase
- After calibration, intelligent fuzzing begins
- Messages are mutated and sent to the drone
- The oracle monitors for anomalous behavior

### Results
- Log files contain detailed execution information
- Anomalies are reported with context
- Coverage data tracks code exploration

## Common Issues and Quick Fixes

### Issue: "ModuleNotFoundError: No module named 'pymavlink'"
```bash
pip install pymavlink
```

### Issue: "SITL binary not found"
```bash
# Check the path in your config file
ls /path/to/ardupilot/build/sitl/bin/arducopter
```

### Issue: "Permission denied" when running SITL
```bash
chmod +x /path/to/ardupilot/build/sitl/bin/arducopter
```

### Issue: Connection timeout
```bash
# Kill any existing SITL processes
pkill -f arducopter
# Then retry
```

### Issue: XML parsing errors
```bash
# Ensure the XML file exists
ls xmls/ardupilotmega.xml
```

## Next Steps

Once you have a successful run, you can:

1. **Customize Configuration**: See [CONFIGURATION.md](CONFIGURATION.md) for advanced options
2. **Try Different Vehicles**: Test with plane or rover configurations
3. **Advanced Parameter Fuzzing**: Add parameter constraints to your config
4. **Custom Missions**: Create your own mission files
5. **Oracle Tuning**: Adjust detection thresholds for your use case

## Example Configurations

### Minimal Test (5 minutes)
```yaml
calibration_rounds: 3
fuzz_interval: 1.0
timeout: 300
```

### Intensive Testing (1 hour)
```yaml
calibration_rounds: 20
fuzz_interval: 0.5
timeout: 3600
generic_params:
  - "SIM_VIB_FREQ_X 10.0 100.0 5.0"
  - "AHRS_TRIM_X -0.1 0.1"
```

### Parameter Fuzzing Focus
```yaml
vehicle: "copter"
peripheral: "CS5"
generic_params:
  - "MOT_THST_EXPO 0.2 0.8"
  - "ATC_RAT_RLL_P 0.05 0.3"
  - "GPS_AUTO_SWITCH 0 1 1"
calibration_rounds: 15
```

## Getting Help

- Check the main [README.md](README.md) for comprehensive documentation
- Review [CONFIGURATION.md](CONFIGURATION.md) for detailed configuration options
- Look at example configurations in the `configs/` directory
- Check log files for detailed error information
- Ensure all file paths in your configuration are correct and accessible

## Performance Tips

- Start with shorter calibration rounds for testing
- Use faster fuzz intervals for quicker results
- Monitor system resources during long runs
- Use absolute paths in configuration files
- Keep log files for analysis and debugging