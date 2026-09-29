"""Position-only IK that selects a feasible wrist pitch near a starting pose."""

from math import inf

from src.kinematics import (
    ArmDimensions,
    JointAngles,
    ToolPose,
    UnreachableTargetError,
    inverse_kinematics,
)


def _angle_difference(a: float, b: float) -> float:
    """Smallest signed difference between degree angles."""
    return (a - b + 180.0) % 360.0 - 180.0


def inverse_kinematics_free(
    x: float,
    y: float,
    z: float,
    dimensions: ArmDimensions,
    start: JointAngles,
    *,
    elbow: str = "down",
    pitch_step: float = 2.0,
) -> JointAngles:
    """Solve tip XYZ and choose a wrist posture close to `start`.

    The wrist pitch is searched across -180..180 degrees. For each candidate,
    the shared fixed-pitch solver finds shoulder and elbow angles. The returned
    solution minimizes squared angle movement from `start` among the sampled
    feasible postures. Smaller `pitch_step` gives a finer search at higher cost.
    """
    if pitch_step <= 0:
        raise ValueError("pitch_step must be greater than zero")
    best = None
    best_cost = inf
    steps = int(360.0 / pitch_step)
    for index in range(steps + 1):
        pitch = -180.0 + index * pitch_step
        try:
            candidate = inverse_kinematics(
                ToolPose(x=x, y=y, z=z, pitch=pitch, roll=start.wrist_roll),
                dimensions,
                elbow=elbow,  # type: ignore[arg-type]
            )
        except UnreachableTargetError:
            continue
        changes = (
            _angle_difference(candidate.base_yaw, start.base_yaw),
            _angle_difference(candidate.shoulder_pitch, start.shoulder_pitch),
            _angle_difference(candidate.elbow_pitch, start.elbow_pitch),
            _angle_difference(candidate.wrist_pitch, start.wrist_pitch),
        )
        cost = sum(change * change for change in changes)
        if cost < best_cost:
            best, best_cost = candidate, cost
    if best is None:
        raise UnreachableTargetError("No sampled wrist pitch can reach this tool-tip position.")
    return best


if __name__ == "__main__":
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
    target_x, target_y, target_z = 180, 30, 140
    solution = inverse_kinematics_free(
        target_x, target_y, target_z, arm, current, elbow="down"
    )
    print("Requested tool tip:", (target_x, target_y, target_z))
    print("Joint angles:", solution)
