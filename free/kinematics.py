"""Interactive free-orientation, tool-tip position IK CLI."""

from free.kinematics import inverse_kinematics_free
from src.kinematics import ArmDimensions, JointAngles, UnreachableTargetError


def read_position() -> tuple[float, float, float] | None:
    """Read one XYZ target, or return None to stop."""
    while True:
        raw = input("Enter x y z (or q to quit): ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None
        values = raw.replace(",", " ").split()
        if len(values) != 3:
            print("Please enter three numbers: x y z")
            continue
        try:
            x, y, z = map(float, values)
        except ValueError:
            print("Those values must be numbers.")
            continue
        return x, y, z


def main() -> None:
    # Edit dimensions and the initial pose here. Keep lengths in one unit.
    arm = ArmDimensions(
        base_height=80,
        upper_arm=120,
        forearm=100,
        wrist_length=40,
        tool_length=60,
    )
    current = JointAngles(
        base_yaw=0,
        shoulder_pitch=20,
        elbow_pitch=-30,
        wrist_pitch=10,
        wrist_roll=0,
    )
    print("Free-orientation IK. Enter x y z, or q to quit.")
    print("After each target, the solution becomes the next starting pose.")
    while True:
        position = read_position()
        if position is None:
            print("Goodbye.")
            return
        try:
            current = inverse_kinematics_free(
                *position, dimensions=arm, start=current, elbow="down"
            )
        except UnreachableTargetError as error:
            print(f"Cannot reach that target: {error}")
            continue
        print("Joint angles (degrees):", current)


if __name__ == "__main__":
    main()
