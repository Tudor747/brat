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
# Five steppers total: four NEMA 17 and one NEMA 23 for shoulder lift.
MOTOR_FRAME_SIZE = {
    "base_yaw": "NEMA 17",
    "shoulder_pitch": "NEMA 23",
    "elbow_pitch": "NEMA 17",
    "wrist_a": "NEMA 17",
    "wrist_b": "NEMA 17",
}

# Direct-joint calibration, including reduction and microstepping.
# steps_per_degree = (motor_steps_per_revolution * microsteps * gear_ratio) / 360
JOINT_STEPS_PER_DEGREE = {
    "base_yaw": None,
    "shoulder_pitch": None,
    "elbow_pitch": None,
}

# Differential calibration in motor-shaft degrees per joint degree.
# a = pitch_ratio * wrist_pitch + roll_ratio * wrist_roll
# b = pitch_ratio * wrist_pitch - roll_ratio * wrist_roll
# Ratios must be measured; do not assume a 1:1 differential.
WRIST_PITCH_RATIO = None
WRIST_ROLL_RATIO = None
WRIST_MOTOR_STEPS_PER_DEGREE = {"wrist_a": None, "wrist_b": None}

# Physical motor direction signs, applied AFTER mixing wrist pitch and roll.
# Same/opposite motion refers to this calibrated motor convention.
MOTOR_DIRECTION = {
    "base_yaw": 1,
    "shoulder_pitch": 1,
    "elbow_pitch": 1,
    "wrist_a": 1,
    "wrist_b": 1,
}

# End-effector servo is separate from the five arm axes.
GRIPPER_SERVO_PIN = None
GRIPPER_OPEN_POSITION = None
GRIPPER_CLOSED_POSITION = None
