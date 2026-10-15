import unittest
from dataclasses import astuple
from math import nan, inf

from config.raspberry_pi import ARM, JOINT_LIMITS
from free.solver import inverse_kinematics_free
from src.cli import execute
from src.controller import ControllerError, SimulatedController, joint_command
from src.kinematics import ArmDimensions, JointAngles, ToolPose, forward_kinematics, inverse_kinematics, within_limits


class KinematicsTests(unittest.TestCase):
    def test_round_trip_both_elbows(self):
        for elbow, angle in (("up", -60), ("down", 60)):
            for yaw in (-80, 0, 80):
                original = JointAngles(yaw, 35, angle, 10, 25)
                pose = forward_kinematics(original, ARM)
                solved = inverse_kinematics(pose, ARM, elbow=elbow)
                for a, b in zip(astuple(pose), astuple(forward_kinematics(solved, ARM))):
                    self.assertAlmostEqual(a, b, places=7)

    def test_invalid_inputs(self):
        for value in (nan, inf):
            with self.assertRaises(ValueError):
                JointAngles(value)
            with self.assertRaises(ValueError):
                ToolPose(value, 0, 0, 0, 0)
            with self.assertRaises(ValueError):
                inverse_kinematics_free(100, 0, 100, ARM, JointAngles(), pitch_step=value)
        with self.assertRaises(ValueError):
            ArmDimensions(upper_arm=0)
        with self.assertRaises(ValueError):
            inverse_kinematics(ToolPose(9999, 0, 0, 0, 0), ARM)

    def test_free_respects_limits_and_reaches_target(self):
        start = JointAngles(10, 40, -50, 20, 5)
        pose = forward_kinematics(start, ARM)
        solved = inverse_kinematics_free(pose.x, pose.y, pose.z, ARM, start, elbow="up", limits=JOINT_LIMITS)
        self.assertTrue(within_limits(solved, JOINT_LIMITS))
        result = forward_kinematics(solved, ARM)
        for a, b in zip(astuple(pose)[:3], astuple(result)[:3]):
            self.assertAlmostEqual(a, b, places=7)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = SimulatedController()

    def test_reference_and_fault_lifecycle(self):
        c = self.controller
        with self.assertRaises(ControllerError):
            execute(c, "joints 0 0 0 0 0")
        execute(c, "enable")
        with self.assertRaises(ControllerError):
            execute(c, "joints 0 0 0 0 0")
        execute(c, "ref 0 40 -50 20 0")
        execute(c, "joints 5 40 -50 20 0")
        pose = forward_kinematics(c.joints, ARM)
        execute(c, f"xyz {pose.x} {pose.y} {pose.z} {pose.pitch}")
        execute(c, "stop")
        with self.assertRaises(ControllerError):
            execute(c, "joints 0 0 0 0 0")
        execute(c, "clear")
        with self.assertRaises(ControllerError):
            execute(c, "joints 0 0 0 0 0")
        execute(c, "ref 5 40 -50 20 0")
        execute(c, "disable")
        self.assertFalse(c.enabled)
        self.assertFalse(c.referenced)

    def test_reject_invalid_commands_without_position_change(self):
        c = self.controller
        execute(c, "enable")
        execute(c, "ref 0 0 0 0 0")
        for line in ("joints 999 0 0 0 0", "joints nan 0 0 0 0", "grip nan", "grip 2", "xyz 9999 0 0", "joints 1 2"):
            with self.assertRaises((ValueError, ControllerError)):
                execute(c, line)
            self.assertEqual(c.joints, JointAngles())


if __name__ == "__main__":
    unittest.main()
