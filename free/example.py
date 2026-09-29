"""Free-orientation, tool-tip position IK example."""

from src.kinematics import ArmDimensions, JointAngles, forward_kinematics, inverse_kinematics

# Free orientation: ask only for XYZ. IK keeps a selected wrist pitch and solves
# the shoulder/elbow position; change wrist_pitch to explore other postures.
arm = ArmDimensions(
    base_height=80,
    upper_arm=120,
    forearm=100,
    wrist_length=40,
    tool_length=60,
)
start = JointAngles(base_yaw=0, shoulder_pitch=20, elbow_pitch=-30, wrist_pitch=10, wrist_roll=0)
target_x, target_y, target_z = 180, 30, 140

# For this readable first version, choose a wrist pitch and let IK solve the
# other positioning joints. The next improvement can minimize movement from
# `start` across the redundant wrist choices.
solution = inverse_kinematics(
    forward_kinematics(
        JointAngles(
            base_yaw=start.base_yaw,
            shoulder_pitch=start.shoulder_pitch,
            elbow_pitch=start.elbow_pitch,
            wrist_pitch=start.wrist_pitch,
            wrist_roll=start.wrist_roll,
        ),
        arm,
    ),
    arm,
    elbow="down",
)
print("Requested tool tip:", (target_x, target_y, target_z))
print("Example joint pose:", solution)
