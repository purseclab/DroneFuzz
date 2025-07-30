#!/usr/bin/env python3
"""
Test script to emulate send_fuzzed_message functionality.
This script demonstrates how to send MAVLink messages with custom field values.
"""

import os
import time
import random
from contextlib import redirect_stdout

# Set MAVLink version
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

class TestMessageSender:
    def __init__(self):
        """Initialize connection to SITL."""
        print("Connecting to SITL...")
        # Suppress MAVLink connection output
        with open(os.devnull, "w") as fnull:
            with redirect_stdout(fnull):
                self.conn = mavutil.mavlink_connection(
                    "udp:localhost:14550", 
                    autoreconnect=True, 
                    retries=3
                )
        
        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.conn.wait_heartbeat()
        print(f"Connected to system {self.conn.target_system}, component {self.conn.target_component}")
        
        self.start_time = time.time()
        self.target_system = 0  # Default target system
        self.target_component = 0  # Default target component

    def apply_heuristics(self, field_values):
        """Apply heuristic replacements for specific fields."""
        for field_name in field_values.keys():
            if "time_boot_ms" in field_name:
                current_time = round((time.time() - self.start_time) * 1e3)
                field_values[field_name] = current_time
            if "time_usec" in field_name:
                field_values[field_name] = int(time.time() * 1e6)
            if "target_system" in field_name:
                field_values[field_name] = self.target_system
            if "target_component" in field_name:
                field_values[field_name] = self.target_component

    def send_fuzzed_message(self, msg_name, msg_id, field_values):
        """Send a fuzzed message using the MAVLink connection.
        
        Args:
            msg_name: Name of the message (e.g., "HEARTBEAT")
            msg_id: ID of the message 
            field_values: Dictionary of field values for the message
            
        Returns:
            List containing the message time, name, id, and field values
        """
        self.apply_heuristics(field_values)
        
        try:
            # Get the message class from mavutil
            msg_class = getattr(self.conn.mav, f"{msg_name.lower()}_send")
            
            # Send the message with fuzzed values
            msg_time = time.time() - self.start_time
            print(f"Sending {msg_name} with values: {field_values}")
            msg_class(**field_values)
            
            return [msg_time, msg_name, msg_id, field_values]
            
        except AttributeError:
            print(f"Message {msg_name} not found, falling back to COMMAND_LONG")
            
            # Fallback to COMMAND_LONG format
            while len(field_values) < 7:
                field_values[f"param{len(field_values) + 1}"] = 0
                
            # Convert to param format
            modified_field_values = {}
            for i, (_, val) in enumerate(field_values.items()):
                val_key = f"param{i + 1}"
                modified_field_values[val_key] = float(val)
                if i >= 6:  # Only use first 7 params
                    break
            
            # Create and send COMMAND_LONG message
            packed_msg = mavutil.mavlink.MAVLink_command_long_message(
                self.target_system,
                self.target_component,
                int(msg_id),
                0,  # confirmation
                **modified_field_values
            )
            
            msg_time = time.time() - self.start_time
            print(f"Sending COMMAND_LONG with ID {msg_id} and params: {modified_field_values}")
            self.conn.mav.send(packed_msg)
            
            return [msg_time, msg_name, msg_id, modified_field_values]
            
        except Exception as e:
            print(f"Error sending message {msg_name}: {e}")
            return []

    def cleanup(self):
        """Close the connection."""
        if self.conn:
            self.conn.close()
            print("Connection closed.")

def generate_test_messages():
    """Generate some test messages with random values."""
    return [
        {
            "msg_name": "HEARTBEAT",
            "msg_id": 0,
            "field_values": {
                "type": random.randint(0, 30),
                "autopilot": random.randint(0, 20),
                "base_mode": random.randint(0, 255),
                "custom_mode": random.randint(0, 4294967295),
                "system_status": random.randint(0, 8)
            }
        },
        {
            "msg_name": "SYSTEM_TIME", 
            "msg_id": 2,
            "field_values": {
                "time_unix_usec": int(time.time() * 1000000),
                "time_boot_ms": 0  # Will be set by heuristics
            }
        },
        {
            "msg_name": "PING",
            "msg_id": 4, 
            "field_values": {
                "time_usec": 0,  # Will be set by heuristics
                "seq": random.randint(0, 65535),
                "target_system": 0,  # Will be set by heuristics
                "target_component": 0  # Will be set by heuristics
            }
        },
        {
            "msg_name": "UNKNOWN_MSG",  # This will trigger fallback
            "msg_id": 999,
            "field_values": {
                "test_param1": random.uniform(-10, 10),
                "test_param2": random.randint(0, 100),
                "test_param3": random.uniform(0, 1)
            }
        }
    ]

def main():
    """Main test function."""
    print("Starting MAVLink message test...")
    
    try:
        # Initialize sender
        sender = TestMessageSender()
        
        # Generate test messages
        # test_messages = generate_test_messages()
        # <field type="uint64_t" name="time_usec" units="us">Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.</field>
        # <field type="uint8_t" name="fix_type" enum="GPS_FIX_TYPE">GPS fix type.</field>
        # <field type="int32_t" name="lat" units="degE7">Latitude (WGS84, EGM96 ellipsoid)</field>
        # <field type="int32_t" name="lon" units="degE7">Longitude (WGS84, EGM96 ellipsoid)</field>
        # <field type="int32_t" name="alt" units="mm">Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.</field>
        # <field type="uint16_t" name="eph" invalid="UINT16_MAX" multiplier="1E-2">GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX</field>
        # <field type="uint16_t" name="epv" invalid="UINT16_MAX" multiplier="1E-2">GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX</field>
        # <field type="uint16_t" name="vel" units="cm/s" invalid="UINT16_MAX">GPS ground speed. If unknown, set to: UINT16_MAX</field>
        # <field type="uint16_t" name="cog" units="cdeg" invalid="UINT16_MAX">Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX</field>
        # <field type="uint8_t" name="satellites_visible" invalid="UINT8_MAX">Number of satellites visible. If unknown, set to UINT8_MAX</field>
        # typedef struct __mavlink_gps_input_t {
        #  time_usec (uint64_t) [us]: 80773000 (80.773000 s)
        # fix_type (GPS_FIX_TYPE): GPS_FIX_TYPE_RTK_FIXED (6)
        # lat (int32_t) [degE7]: -353632621 (-35.3632621 deg)
        # lon (int32_t) [degE7]: 1491652374 (149.1652374 deg)
        # alt (int32_t) [mm]: 584090
        # eph (uint16_t*1E-2): 1.21
        # epv (uint16_t*1E-2): 2
        # vel (uint16_t) [cm/s]: 0
        # cog (uint16_t) [cdeg]: 0
        # satellites_visible (uint8_t): 10
        # alt_ellipsoid (int32_t) [mm]: 584090
        # h_acc (uint32_t) [mm]: 300
        # v_acc (uint32_t) [mm]: 300
        # vel_acc (uint32_t) [mm/s]: 0
        # hdg_acc (uint32_t) [degE5]: 0
        # yaw (uint16_t) [cdeg]: 0

        test_messages = [
            {
            "msg_name": "GPS_INPUT",
            "msg_id": 232,
            "field_values": {
                "time_usec": 0,  # Will be set by heuristics
                "time_week_ms": random.randint(0, 604800000),  # GPS time since start of week (ms)
                "lat": -353632621,  # Latitude in degE7
                "lon": 1491652374 ,  # Longitude in degE7
                "alt": 584,  # Altitude in meters
                "hdop": random.uniform(0.5, 10.0),  # Horizontal dilution of position
                "vdop": random.uniform(0.5, 10.0),  # Vertical dilution of position
                "vn": random.uniform(-100, 100),  # North velocity in m/s
                "ve": random.uniform(-100, 100),  # East velocity in m/s
                "vd": random.uniform(-20, 20),  # Down velocity in m/s
                "speed_accuracy": random.uniform(0.1, 5.0),  # Speed accuracy in m/s
                "horiz_accuracy": random.uniform(0.5, 20.0),  # Horizontal accuracy in m
                "vert_accuracy": random.uniform(0.5, 20.0),  # Vertical accuracy in m
                "ignore_flags": 0,  # Bitmap of which fields to ignore
                "time_week": random.randint(0, 1024),  # GPS week number
                "gps_id": 0,  # ID of the GPS
                "fix_type": 6,  # GPS fix type
                "satellites_visible": 10,  # Number of satellites visible
                "yaw": 0  # Yaw in centidegrees
            }
            }
        ]
        # test_messages = [
        #     {
        #         "msg_name": "GPS_RAW_INT",  # This will trigger fallback
        #         "msg_id": 24,
        #         "field_values": {
        #             "time_usec": 1200,  # Will be set by heuristics
        #             "fix_type": random.randint(0, 6),
        #             "lat": random.randint(-900000000, 900000000),  # Latitude in degE7
        #             "lon": random.randint(-1800000000, 1800000000),  # Longitude in degE7
        #             "alt": random.randint(-10000000, 10000000),  # Altitude in mm
        #             "eph": random.randint(0, 65535),  # HDOP * 100
        #             "epv": random.randint(0, 65535),  # VDOP * 100
        #             "vel": random.randint(0, 65535),  # Ground speed in cm/s
        #             "cog": random.randint(0, 35999),  # Course over ground in cdeg
        #             "satellites_visible": random.randint(0, 255)  # Number of satellites visible
        #         }
        #     }
        # ]
        
        print(f"\nSending {len(test_messages)} test messages...")
        print("-" * 50)
        
        # Send each test message
        while True:
            for i, msg_data in enumerate(test_messages):
                print(f"\nTest {i+1}/{len(test_messages)}:")
                
                result = sender.send_fuzzed_message(
                    msg_data["msg_name"],
                    msg_data["msg_id"], 
                    msg_data["field_values"]
                )
                
                if result:
                    print(f"✓ Message sent successfully: {result[0]:.3f}s")
                else:
                    print("✗ Failed to send message")
                    
                # Wait between messages
                time.sleep(1/5) # 5 Hz rate
            
            print("\n" + "=" * 50)
            print("Test completed successfully!")

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
            # except Exception as e:
            #     print(f"Error during test: {e}")
            # finally:
            #     if 'sender' in locals():
            #         sender.cleanup()

if __name__ == "__main__":
    main()