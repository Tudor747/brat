"""Shared host settings. Motor calibration lives in hardware.h on the ESP."""
from src.kinematics import ArmDimensions

SERIAL_PORT = "/dev/ttyUSB0"  # Windows example: COM5
SERIAL_BAUD_RATE = 115200
SERIAL_TIMEOUT_SECONDS = 0.1
MOVE_TIMEOUT_SECONDS = 180.0
ARM = ArmDimensions()
# Keep synchronized with hardware.h. Units: degrees.
JOINT_LIMITS = ((-90, 90), (-30, 120), (-120, 120), (-90, 90), (-90, 90))
