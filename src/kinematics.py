"""Readable forward and inverse kinematics for a simple 5-axis arm.

Axes: base yaw, shoulder pitch, elbow pitch, wrist pitch, wrist roll.
Angles are degrees at the public API. Internal calculations use radians.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, degrees, hypot, radians, sin
from typing import Literal


@dataclass(frozen=True)
class ArmDimensions:
    """All lengths use one unit (millimeters by default)."""

    base_height: float = 90.0
    upper_arm: float = 230.0
    forearm: float = 175.0
    wrist_length: float = 42.0
    tool_length: float = 141.0


@dataclass(frozen=True)
class JointAngles:
    """Five commanded joint angles, in degrees."""

    base_yaw: float = 0.0
    shoulder_pitch: float = 0.0
    elbow_pitch: float = 0.0
    wrist_pitch: float = 0.0
    wrist_roll: float = 0.0


@dataclass(frozen=True)
class ToolPose:
    """Tool tip position and its pitch/roll, lengths in arm units, angles in degrees."""

    x: float
    y: float
    z: float
    pitch: float
    roll: float


ElbowMode = Literal["up", "down"]


class UnreachableTargetError(ValueError):
    """Raised when inverse kinematics cannot reach a requested position."""


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def forward_kinematics(angles: JointAngles, dimensions: ArmDimensions) -> ToolPose:
    """Calculate tool tip pose from joint angles.

    Pitch is measured from horizontal. Positive shoulder/elbow pitches lift the
    arm. Base yaw rotates the arm's horizontal reach around the vertical axis.
    """
    shoulder = radians(angles.shoulder_pitch)
    elbow = radians(angles.elbow_pitch)
    wrist = radians(angles.wrist_pitch)
    yaw = radians(angles.base_yaw)

    elbow_pitch = shoulder + elbow
    wrist_pitch = elbow_pitch + wrist
    radial = (
        dimensions.upper_arm * cos(shoulder)
        + dimensions.forearm * cos(elbow_pitch)
        + (dimensions.wrist_length + dimensions.tool_length) * cos(wrist_pitch)
    )
    z = (
        dimensions.base_height
        + dimensions.upper_arm * sin(shoulder)
        + dimensions.forearm * sin(elbow_pitch)
        + (dimensions.wrist_length + dimensions.tool_length) * sin(wrist_pitch)
    )
    return ToolPose(
        x=radial * cos(yaw),
        y=radial * sin(yaw),
        z=z,
        pitch=degrees(wrist_pitch),
        roll=angles.wrist_roll,
    )


def inverse_kinematics(
    target: ToolPose,
    dimensions: ArmDimensions,
    *,
    elbow: ElbowMode = "down",
) -> JointAngles:
    """Solve a target tool pose using a two-link planar analytic solution.

    The requested tool pitch fixes the wrist orientation. The shoulder and
    elbow solve for the wrist center; wrist pitch is then the remaining angle.
    Wrist roll is independent of this simplified position model.
    """
    if elbow not in ("up", "down"):
        raise ValueError("elbow must be 'up' or 'down'")

    yaw = atan2(target.y, target.x)
    radial = hypot(target.x, target.y)
    pitch = radians(target.pitch)

    # Remove the wrist and tool extension to find the point solved by the
    # two-link shoulder/elbow mechanism.
    extension = dimensions.wrist_length + dimensions.tool_length
    wrist_radial = radial - extension * cos(pitch)
    wrist_z = target.z - dimensions.base_height - extension * sin(pitch)

    upper = dimensions.upper_arm
    forearm = dimensions.forearm
    cos_elbow = (wrist_radial**2 + wrist_z**2 - upper**2 - forearm**2) / (2 * upper * forearm)
    if cos_elbow < -1.0 - 1e-9 or cos_elbow > 1.0 + 1e-9:
        distance = hypot(wrist_radial, wrist_z)
        raise UnreachableTargetError(
            f"Target wrist center is {distance:.2f} units from the shoulder; "
            f"reachable range is {abs(upper - forearm):.2f} to {upper + forearm:.2f}."
        )
    cos_elbow = _clamp(cos_elbow, -1.0, 1.0)
    elbow_sign = 1.0 if elbow == "down" else -1.0
    elbow_angle = elbow_sign * acos(cos_elbow)
    shoulder_angle = atan2(wrist_z, wrist_radial) - atan2(
        forearm * sin(elbow_angle), upper + forearm * cos(elbow_angle)
    )
    wrist_angle = pitch - shoulder_angle - elbow_angle

    return JointAngles(
        base_yaw=degrees(yaw),
        shoulder_pitch=degrees(shoulder_angle),
        elbow_pitch=degrees(elbow_angle),
        wrist_pitch=degrees(wrist_angle),
        wrist_roll=target.roll,
    )
