1. Request GIMBAL Manager status
message COMMAND_LONG 0 0 511 0 281 1000000 0 0 0 0 0
Denied
2. Setting self in control
message COMMAND_LONG 0 0 1001 0 -2 0 0 0 0 0 0
*Unsupported?*
3. Let's request something with MAV_CMD_REQUEST_MESSAGE
message COMMAND_LONG 0 0 512 0 0 0 0 0 0 0 0
*Works, but nothing useful for us*
4. Let's request the GIMBAL_MANAGER_INFORMATION
message COMMAND_LONG 0 0 512 0 283 0 0 0 0 0 0
*Straight up fails*
