"""Serial transport and matching in-memory controller for commissioning."""
from dataclasses import astuple
import time

from config.raspberry_pi import JOINT_LIMITS, MOVE_TIMEOUT_SECONDS
from src.kinematics import JointAngles, within_limits


class ControllerError(RuntimeError):
    pass


def joint_command(name, angles):
    if not within_limits(angles, JOINT_LIMITS):
        raise ValueError("Joint target exceeds configured limits")
    return name + "," + ",".join(format(value, ".8f") for value in astuple(angles))


class SerialController:
    def __init__(self, port, baud=115200, timeout=MOVE_TIMEOUT_SECONDS):
        import serial
        self.serial = serial.Serial(port, baud, timeout=0.1, write_timeout=1)
        self.timeout = timeout
        # Opening USB serial may reset the ESP. Do not send motion at startup.
        try:
            time.sleep(2)
            self.serial.reset_input_buffer()
            self.command("PING")
        except BaseException:
            self.serial.close()
            raise

    def _write(self, line):
        self.serial.write((line + "\n").encode("ascii"))

    def command(self, line):
        if "\n" in line or "\r" in line or len(line) >= 191:
            raise ValueError("Invalid command line")
        name = line.split(",")[0]
        self._write(line)
        deadline = time.monotonic() + (self.timeout if name == "MOVE" else 3)
        heartbeat = time.monotonic() + 0.5
        try:
            while time.monotonic() < deadline:
                response = self.serial.readline().decode("ascii", errors="replace").strip()
                if response.startswith("READY,"):
                    raise ControllerError("ESP restarted; reference required")
                if response.startswith("ERROR,"):
                    raise ControllerError(response)
                if response.startswith("FAULT,") and name not in ("STOP", "DISABLE"):
                    raise ControllerError(response)
                if name == "STATUS" and response.startswith("STATUS,"):
                    return response
                if response == ("DONE,MOVE" if name == "MOVE" else "OK," + name):
                    return response
                if name == "MOVE" and time.monotonic() >= heartbeat:
                    self._write("PING")
                    heartbeat = time.monotonic() + 0.5
            raise ControllerError("Controller response timed out")
        except BaseException:
            if name == "MOVE":
                # Best effort only; firmware also has an independent link watchdog.
                try:
                    self._write("STOP")
                except Exception:
                    pass
            raise

    def close(self):
        self.serial.close()


class SimulatedController:
    """Instantaneous joint-space simulation; no collision or torque model."""
    def __init__(self):
        self.joints = JointAngles()
        self.enabled = self.referenced = self.faulted = False

    def command(self, line):
        fields = line.split(",")
        name, args = fields[0], fields[1:]
        counts = {"PING": 0, "STATUS": 0, "STOP": 0, "DISABLE": 0, "CLEAR": 0,
                  "ENABLE": 0, "REF": 5, "MOVE": 5, "GRIP": 1}
        if name not in counts or len(args) != counts[name]:
            raise ControllerError("ERROR,FORMAT")
        if name == "STATUS":
            return "STATUS,SIM,IDLE,{},{},{},{}".format(int(self.referenced), int(self.enabled), int(self.faulted), ",".join(map(str, astuple(self.joints))))
        if name in ("STOP", "DISABLE"):
            self.referenced = False
            self.faulted = True
            if name == "DISABLE":
                self.enabled = False
        elif name == "CLEAR":
            self.faulted = False
        elif name != "PING":
            if self.faulted:
                raise ControllerError("ERROR,INTERLOCK_OR_FAULT")
            if name == "ENABLE":
                self.enabled = True
            else:
                if not self.enabled:
                    raise ControllerError("ERROR,DISABLED")
                if name in ("MOVE", "REF"):
                    angles = JointAngles(*map(float, args))
                    joint_command(name, angles)
                    if name == "MOVE" and not self.referenced:
                        raise ControllerError("ERROR,REFERENCE_REQUIRED")
                    self.joints = angles
                    self.referenced = True
                elif not 0 <= float(args[0]) <= 1:
                    raise ValueError("Gripper value must be between 0 and 1")
        return "DONE,MOVE" if name == "MOVE" else "OK," + name

    def close(self):
        pass
