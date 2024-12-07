from pymavlink import mavutil
import time

# Connect to the MAVLink vehicle
connection_string = "udp:127.0.0.1:14550"  # Adjust as needed for your setup
vehicle = mavutil.mavlink_connection(connection_string)


def wait_for_heartbeat():
    """Wait for a heartbeat to ensure communication is established."""
    print("Waiting for heartbeat...")
    vehicle.wait_heartbeat()
    print(
        "Heartbeat received from system (system %u, component %u)"
        % (vehicle.target_system, vehicle.target_component)
    )


def set_mode(mode):
    """Set vehicle mode."""
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # Base mode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        mode,
        0,
        0,
        0,
        0,
        0,
    )
    ack = vehicle.recv_match(type="COMMAND_ACK", blocking=True)
    print(f"Mode set to {mode}, ACK: {ack.result}")


def arm_vehicle():
    """Arm the vehicle."""
    print("Arming vehicle...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 to arm
        0,
        0,
        0,
        0,
        0,
        0,
    )
    ack = vehicle.recv_match(type="COMMAND_ACK", blocking=True)
    if ack.result != 0:
        print("Arming failed with ACK: %s" % ack.result)
        exit(1)
    print("ARM command ACK: %s" % ack.result)


def takeoff(altitude):
    """Command the vehicle to take off."""
    print(f"Taking off to {altitude} meters...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude,
    )
    ack = vehicle.recv_match(type="COMMAND_ACK", blocking=True)
    if ack.result != 0:
        print("Takeoff command failed with ACK: %s" % ack.result)
        exit(1)
    print("Waiting for vehicle to reach takeoff altitude...")
    while True:
        msg = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        msg = msg.to_dict()
        if (msg["relative_alt"] / 1e3) >= altitude * 0.95:
            print("Value of altitude: ", msg["alt"])
            print("Reached target altitude")
            break


def approx_equal(a, b, tolerance):
    return abs(a - b) < tolerance


def go_to_waypoint(lat, lon, alt):
    """Navigate to a specified waypoint."""
    print(f"Navigating to waypoint: lat={lat}, lon={lon}, alt={alt}")
    vehicle.mav.set_position_target_global_int_send(
        0,
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,  # Bitmask: enable x, y, z
        int(lat * 1e7),  # Latitude in 1e7 degrees
        int(lon * 1e7),  # Longitude in 1e7 degrees
        alt,  # Altitude in meters
        0,
        0,
        0,  # x, y, z velocity
        0,
        0,
        0,  # x, y, z acceleration
        0,
        0,  # yaw, yaw rate
    )
    while True:
        msg = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        msg = msg.to_dict()
        current_lat = round(msg["lat"] / 1e7, ndigits=5)
        current_lon = round(msg["lon"] / 1e7, ndigits=5)
        requred_lat = round(lat, ndigits=5)
        required_lon = round(lon, ndigits=5)
        tolerance = 0.00005
        if approx_equal(current_lat, requred_lat, tolerance) and approx_equal(
            current_lon, required_lon, tolerance
        ):
            print("Reached target waypoint")
            print("Debug: ", current_lat, requred_lat, current_lon, required_lon)
            break


def land():
    """Land the vehicle."""
    print("Initiating landing...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    ack = vehicle.recv_match(type="COMMAND_ACK", blocking=True)
    print("Land command ACK: %s" % ack.result)


# Main sequence
wait_for_heartbeat()

# Set GUIDED mode (3 is usually GUIDED, but check your vehicle's documentation)
GUIDED_MODE = 4  # Adjust if needed
LOITER_MODE = vehicle.mode_mapping()["LOITER"]
set_mode(GUIDED_MODE)

# Arm the vehicle
arm_vehicle()

# Takeoff to 50 meters
takeoff(50)

go_to_waypoint(-35.3632621, 149.1652374, 50)

# Go to Point B -35.3626941, 149.166221
go_to_waypoint(-35.3626941, 149.166221, 50)

# Loiter for a while
set_mode(LOITER_MODE)
time.sleep(10)
set_mode(GUIDED_MODE)

# -35.362839699999995, 149.1646279,
go_to_waypoint(-35.362839699999995, 149.1646279, 50)

# Go to point X -35.3632621, 149.1652374,
go_to_waypoint(-35.3632621, 149.1652374, 50)

# Land
land()
