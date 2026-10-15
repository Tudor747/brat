"""Run with python -m src.cli [--port COM5]. Default: simulation."""
import argparse
from dataclasses import astuple

from config.raspberry_pi import ARM, JOINT_LIMITS, SERIAL_BAUD_RATE
from free.solver import inverse_kinematics_free
from src.controller import ControllerError, SerialController, SimulatedController, joint_command
from src.kinematics import JointAngles, ToolPose, forward_kinematics, inverse_kinematics, within_limits

HELP = """Commands (angles in degrees; lengths in mm):
  status | enable | clear | stop | disable
  ref base shoulder elbow wrist_pitch wrist_roll  (declare measured pose; NO motion)
  joints base shoulder elbow wrist_pitch wrist_roll
  xyz x y z [tool_pitch [tool_roll]]
  grip fraction   (0..1 between calibrated servo endpoints)
  fk | help | quit
Every startup: enable, then ref at the actual measured pose. No automatic homing.
"""


def current_pose(controller):
    status = controller.command("STATUS").split(",")
    if len(status) != 11 or status[2] != "IDLE" or status[3] != "1":
        raise ControllerError("Idle referenced controller required")
    return JointAngles(*map(float, status[6:]))


def execute(controller, line):
    parts = line.replace(",", " ").split()
    if not parts:
        return ""
    command = parts[0].lower()
    if command == "help":
        return HELP
    if command in {"status", "enable", "clear", "stop", "disable"} and len(parts) == 1:
        return controller.command(command.upper())
    values = list(map(float, parts[1:]))
    if command in {"ref", "joints"} and len(values) == 5:
        return controller.command(joint_command("REF" if command == "ref" else "MOVE", JointAngles(*values)))
    if command == "grip" and len(values) == 1:
        if not 0 <= values[0] <= 1:
            raise ValueError("Gripper value must be between 0 and 1")
        return controller.command(f"GRIP,{values[0]:.8f}")
    if command == "fk" and not values:
        return str(forward_kinematics(current_pose(controller), ARM))
    if command == "xyz" and len(values) in (3, 4, 5):
        start = current_pose(controller)
        candidates = []
        for elbow in ("up", "down"):
            try:
                if len(values) == 3:
                    angles = inverse_kinematics_free(*values, ARM, start, elbow=elbow, limits=JOINT_LIMITS)
                else:
                    angles = inverse_kinematics(ToolPose(*values[:4], values[4] if len(values) == 5 else start.wrist_roll), ARM, elbow=elbow)
                if within_limits(angles, JOINT_LIMITS):
                    candidates.append(angles)
            except ValueError:
                continue
        if not candidates:
            raise ValueError("No IK solution within joint limits")
        target = min(candidates, key=lambda a: sum((x-y)**2 for x,y in zip(astuple(a), astuple(start))))
        return f"{target}\n{controller.command(joint_command('MOVE', target))}"
    raise ValueError("Unknown command or wrong number of values; enter help")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Hardware serial port; omit for simulation")
    args = parser.parse_args()
    controller = SerialController(args.port, SERIAL_BAUD_RATE) if args.port else SimulatedController()
    print("SERIAL: " + args.port if args.port else "SIMULATION: no hardware connected")
    print(HELP)
    try:
        while True:
            try:
                line = input("arm> ").strip()
                if line.lower() in {"q", "quit", "exit"}:
                    break
                print(execute(controller, line))
            except (ValueError, ControllerError) as error:
                print(error)
    except (EOFError, KeyboardInterrupt):
        print("\nStopping.")
    finally:
        try:
            controller.command("STOP")
        finally:
            controller.close()


if __name__ == "__main__":
    main()
