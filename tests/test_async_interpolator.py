import time

from supre_robot_sdk.hardware.base import HardwareInterface
from supre_robot_sdk.hardware.interpolation import AsyncInterpolator


class FakeHardware(HardwareInterface):
    def __init__(self):
        self.writes = []

    def init(self, config):
        return True

    def activate(self):
        return True

    def deactivate(self):
        return None

    def read(self):
        return [(0.0, 0.0), (0.0, 0.0)]

    def write(self, commands_positions):
        self.writes.append(list(commands_positions))

    def get_joint_count(self):
        return 2


def test_async_interpolator_pass_through():
    hardware = FakeHardware()
    interpolator = AsyncInterpolator(hardware, {"interpolation_n": 1, "control_frequency": 10})
    interpolator.write([1.0, 2.0])
    assert hardware.writes == [[1.0, 2.0]]


def test_async_interpolator_background_writer():
    hardware = FakeHardware()
    interpolator = AsyncInterpolator(hardware, {"interpolation_n": 2, "control_frequency": 20})
    assert interpolator.activate() is True
    interpolator.write([1.0, 2.0])
    time.sleep(0.1)
    interpolator.deactivate()
    assert hardware.writes
    assert any(write[0] > 0.0 for write in hardware.writes)

