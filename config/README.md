# Hardware configuration and firmware

This folder separates Raspberry Pi configuration from ESP firmware:

- `raspberry_pi.py` holds serial settings and joint calibration placeholders.
- `esp_controller.ino` is an Arduino-style C++ firmware skeleton for the ESP.

The two files document a proposed newline-terminated command format:

```text
MOVE,base_yaw,shoulder_pitch,elbow_pitch,wrist_pitch,wrist_roll
```

The values are angles in degrees. The ESP should validate commands, apply
calibrated joint limits, drive stepper pulses, and return `OK` or `ERROR`.
The gripper servo is a separate control from those five axes.

These are starting points, not a working hardware link yet. Fill in the serial
port, board-specific pins, motor steps-per-degree, direction, and servo values
after selecting the components and measuring the arm. The ESP sketch needs
board-specific motion, homing, and safety code before connecting motors.
