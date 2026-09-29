"""Interactive fixed-tool-pitch inverse kinematics CLI."""

from src.kinematics import ArmDimensions, ToolPose, UnreachableTargetError, inverse_kinematics


def read_target() -> ToolPose | None:
    """Read one position and tool pitch, or return None to stop."""
    while True:
        raw = input("Enter x y z tool_pitch (or q to quit): ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None
        values = raw.replace(",", " ").split()
        if len(values) != 4:
            print("Please enter four numbers: x y z tool_pitch")
            continue
        try:
            x, y, z, pitch = map(float, values)
        except ValueError:
            print("Those values must be numbers.")
            continue
        return ToolPose(x=x, y=y, z=z, pitch=pitch, roll=0.0)


def main() -> None:
    # Edit dimensions here. Keep all lengths in the same unit (example: mm).
    arm = ArmDimensions(
        base_height=80,
        upper_arm=120,
        forearm=100,
        wrist_length=40,
        tool_length=60,
    )
    print("Fixed-pitch IK. Enter x y z pitch, or q to quit.")
    while True:
        target = read_target()
        if target is None:
            print("Goodbye.")
            return
        try:
            joints = inverse_kinematics(target, arm, elbow="down")
        except UnreachableTargetError as error:
            print(f"Cannot reach that target: {error}")
            continue
        print("Joint angles (degrees):", joints)


if __name__ == "__main__":
    main()
