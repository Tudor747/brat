"""Position-only IK that chooses a wrist posture near the current pose."""

from math import inf
from typing import Literal

from src.kinematics import (
    ArmDimensions,
    JointAngles,
    ToolPose,
    UnreachableTargetError,
    inverse_kinematics,
)


def _angle_difference(first: float, second: float) -> float:
    """Return the shortest signed difference between two degree angles."""
    return (first - second + 180.0) % 360.0 - 180.0


def inverse_kinematics_free(
    x: float,
    y: float,
    z: float,
    dimensions: ArmDimensions,
    start: JointAngles,
    *,
    elbow: Literal["up", "down"] = "down",
    pitch_step: float = 2.0,
) -> JointAngles:
    """Solve tool-tip XYZ and select the sampled pose closest to `start`.

    The function searches wrist pitches from -180 to +180 degrees, uses the
    fixed-pitch IK for each candidate, then minimizes total squared angular
    movement over base, shoulder, elbow, and wrist pitch.
    """
    if pitch_step <= 0:
        raise ValueError("pitch_step must be greater than zero")

    best_pose = None
    best_cost = inf
    pitch_count = int(360.0 / pitch_step)

    for index in range(pitch_count + 1):
        pitch = -180.0 + index * pitch_step
        try:
            candidate = inverse_kinematics(
                ToolPose(x=x, y=y, z=z, pitch=pitch, roll=start.wrist_roll),
                dimensions,
                elbow=elbow,
            )
        except UnreachableTargetError:
            continue

        angle_changes = (
            _angle_difference(candidate.base_yaw, start.base_yaw),
            _angle_difference(candidate.shoulder_pitch, start.shoulder_pitch),
            _angle_difference(candidate.elbow_pitch, start.elbow_pitch),
            _angle_difference(candidate.wrist_pitch, start.wrist_pitch),
        )
        movement_cost = sum(change * change for change in angle_changes)
        if movement_cost < best_cost:
            best_pose = candidate
            best_cost = movement_cost

    if best_pose is None:
        raise UnreachableTargetError(
            "No sampled wrist pitch can reach this tool-tip position."
        )
    return best_pose
