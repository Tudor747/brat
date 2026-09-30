"""Raspberry Pi to ESP configuration placeholders.

Edit these values after choosing the board, serial connection, drivers, and
measuring each joint. This file is configuration only; it does not open serial.
"""

# Serial link from Raspberry Pi to ESP.
SERIAL_PORT = "/dev/serial0"  # Update for your Raspberry Pi connection.
SERIAL_BAUD_RATE = 115200
SERIAL_TIMEOUT_SECONDS = 1.0

# Human-readable command format planned for the serial protocol.
# Example line: MOVE,base_yaw,shoulder_pitch,elbow_pitch,wrist_pitch,wrist_roll\n
# Joint-to-motor calibration. Replace placeholders with measured values.
# steps_per_degree = (motor_steps_per_revolution * microsteps * belt_ratio) / 360
JOINT_STEPS_PER_DEGREE = {
    "base_yaw": None,
    "shoulder_pitch": None,
    "elbow_pitch": None,
    "wrist_pitch": None,
    "wrist_roll": None,
}

# Motor direction signs; set each to +1 or -1 after checking motion direction.
JOINT_DIRECTION = {
    "base_yaw": 1,
    "shoulder_pitch": 1,
    "elbow_pitch": 1,
    "wrist_pitch": 1,
    "wrist_roll": 1,
}

# End-effector servo is separate from the five arm axes.
GRIPPER_SERVO_PIN = None
GRIPPER_OPEN_POSITION = None
GRIPPER_CLOSED_POSITION = None
