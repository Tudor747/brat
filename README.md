# Five axis robotic arm kinematics (Python)

Dependency-free educational kinematics for a five-axis arm: base yaw, shoulder
pitch, elbow pitch, wrist pitch, and wrist roll. The separate gripper servo is
outside the arm model. Shoulder and elbow rotate around parallel left-to-right
axes, so they move in one plane; base yaw rotates that plane around the vertical
axis.

## Folders

- `fixed/` — IK for a requested tool-tip position and fixed tool pitch.
- `free/` — IK for tool-tip XYZ that chooses the feasible wrist pitch closest
to the current joint pose.
- `src/kinematics.py` — shared dimensions, forward kinematics, and fixed-pitch IK.

Run from the project root with `python -m fixed.example` or
`python -m free.kinematics`.

## Edit dimensions

Change `ArmDimensions` in either example. All lengths must use the same unit;
the sample values are millimeters. Public angles are degrees. The five axes are
base yaw, shoulder pitch, elbow pitch, wrist pitch, and wrist roll.

## Coordinate conventions

- `x` points forward at zero base yaw, `y` points left, and `z` points upward.
- Base yaw rotates around the vertical axis.
- Positive shoulder/elbow pitch raises the arm.
- Wrist length plus tool length is a fixed extension aligned to tool pitch.
- Wrist roll and the gripper servo do not affect modeled tip position.

## Pi and ESP

Keep kinematics as ordinary Python calculations on the Pi. The Pi can send joint
commands over serial to the ESP, which handles timed step pulses and hardware
limits. Convert degrees to motor steps using measured belt ratios, steps per
revolution, and calibration for each joint.
