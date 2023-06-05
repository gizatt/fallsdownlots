## Notes

The code is a misc hodge-podge of example code, library code, and custom code to get everything talking. It's overdue for a style / content cleanup with updated comments...

## BLE Comms Protocol

The robot sends a 36-byte (+COBS-encoding) packet over BLE UART at ~30hz consisting of 8 floats:
```
[pitch, yaw, d_pitch, d_yaw, odometry, l_cmd, r_cmd, max_motor_temp]
```

On-board, the robot has a registry of named control parameters that you can write with a command formatted:
```
<NULL-TERMINATED-NAME>[optional single float]
```
and the robot will respond with a similarly formatted message (+COBS-encoding) packet with a single float indicating the updated value, or the current value if you didn't include the float value.