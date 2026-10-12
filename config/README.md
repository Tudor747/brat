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

## Motor layout

From base to wrist the motion sequence is **roll -> pitch -> pitch ->
pitch + roll**. Base roll uses the existing `base_yaw` field in configuration
and the serial protocol: it rotates about the upright base axis (world `z`).
Shoulder and elbow provide pitch; the differential wrist provides pitch and
tool-axis roll, giving five arm axes in total.

The arm uses five steppers: four NEMA 17 motors and one NEMA 23. The NEMA 23
drives shoulder lift. Base rotation, the elbow, and both differential wrist
motors use NEMA 17 motors.
The gripper opens and closes using a separate servo.

The wrist uses two physical motors, A and B, coupled through a differential.
Equal motion in the same calibrated direction produces wrist pitch; equal
motion in opposite directions produces wrist roll (tool spin). Unequal motion
can combine pitch and roll. Joint angles remain the inputs to kinematics and
the `MOVE` protocol; the ESP must mix them before producing motor steps:

```text
motor_a_degrees = pitch_ratio * wrist_pitch + roll_ratio * wrist_roll
motor_b_degrees = pitch_ratio * wrist_pitch - roll_ratio * wrist_roll
```

These equations assume a symmetric differential and calibrated motor directions.
Measure the pitch/roll ratios, motor directions, and zero references on the
actual mechanism. Convert each mixed motor angle into steps using its own
microstepping calibration, and coordinate both motors throughout a move.
The configuration records placeholders; the firmware does not yet implement
mixing or pulse generation. NEMA frame size does not specify step angle,
current, torque, or gear reduction.

These are starting points, not a working hardware link yet. Fill in the serial
port, board-specific pins, motor steps-per-degree, direction, and servo values
after selecting the components and measuring the arm. The ESP sketch needs
board-specific motion, homing, and safety code before connecting motors.
