"""Native tests execute the real sketch with mocked time/GPIO/serial."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


@unittest.skipUnless(shutil.which("g++"), "g++ required for native firmware tests")
class FirmwareTests(unittest.TestCase):
    def test_sketch_simulation_and_gpio_modes(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            for mode, calibrated in ((0, 0), (0, 1), (1, 0)):
                executable = Path(directory) / f"firmware-{mode}.exe"
                subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                                f"-DROBOARM_SIMULATION={mode}", f"-DROBOARM_CALIBRATED={calibrated}",
                                "-I", str(root / "tests/firmware"),
                                str(root / "tests/firmware/test_controller.cpp"),
                                "-o", str(executable)], check=True)
                subprocess.run([str(executable)], check=True, timeout=10)
