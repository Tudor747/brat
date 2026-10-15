import unittest
from unittest.mock import patch

from src.controller import ControllerError, SerialController


class FakeSerial:
    def __init__(self, replies):
        self.replies = list(replies)
        self.sent = []

    def write(self, data):
        self.sent.append(data)

    def readline(self):
        return self.replies.pop(0) if self.replies else b""


def controller(replies):
    c = SerialController.__new__(SerialController)
    c.serial = FakeSerial(replies)
    c.timeout = 3
    return c


class SerialTests(unittest.TestCase):
    def test_move_waits_for_completion_and_sends_heartbeat(self):
        c = controller([b"OK,MOVE\n", b"OK,PING\n", b"DONE,MOVE\n"])
        times = iter(i * 0.3 for i in range(100))
        with patch("src.controller.time.monotonic", side_effect=lambda: next(times)):
            self.assertEqual(c.command("MOVE,0,0,0,0,0"), "DONE,MOVE")
        self.assertIn(b"PING\n", c.serial.sent)

    def test_fault_and_reset_abort_move(self):
        for reply in (b"FAULT,INTERLOCK\n", b"READY,1,HW\n", b"ERROR,LIMIT\n"):
            c = controller([reply])
            with self.assertRaises(ControllerError):
                c.command("MOVE,0,0,0,0,0")
            self.assertEqual(c.serial.sent[-1], b"STOP\n")

    def test_timeout_requests_stop(self):
        c = controller([])
        times = iter(range(20))
        with patch("src.controller.time.monotonic", side_effect=lambda: next(times)):
            with self.assertRaisesRegex(ControllerError, "timed out"):
                c.command("MOVE,0,0,0,0,0")
        self.assertEqual(c.serial.sent[-1], b"STOP\n")

    def test_stop_consumes_its_fault_then_ack(self):
        c = controller([b"FAULT,STOP\n", b"OK,STOP\n"])
        self.assertEqual(c.command("STOP"), "OK,STOP")
