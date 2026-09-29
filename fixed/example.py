from src.kinematics import ArmDimensions, JointAngles, ToolPose, inverse_kinematics

# Fixed-orientation IK: set the desired tool-tip position and pitch.
arm = ArmDimensions(
    base_height=80,
    upper_arm=120,
    forearm=100,
    wrist_length=40,
    tool_length=60,
)
target = ToolPose(x=180, y=30, z=140, pitch=0, roll=0)
solution = inverse_kinematics(target, arm, elbow="down")
print("Fixed tool pitch (degrees):", target.pitch)
print("Joint angles:", solution)
