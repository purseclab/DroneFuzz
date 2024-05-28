import time
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect

# Connect to the vehicle
sensor_conn = mavutil.mavlink_connection("udp:127.0.0.1:1337")
sensor_conn.wait_heartbeat()

gcs_conn = mavutil.mavlink_connection("udp:127.0.0.1:14551")
gcs_conn.wait_heartbeat()
