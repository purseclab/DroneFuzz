### Messages sent to request Gimbal status
1. Request GIMBAL Manager status
message COMMAND_LONG 0 0 511 0 281 1000000 0 0 0 0 0
No ap_message for mavlink id 281
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
5. Actually just ask for AUTOPILOT_STATE_FOR_GIMBAL_DEVICE?
message COMMAND_LONG 0 0 512 0 286 0 0 0 0 0 0
*Works!*
