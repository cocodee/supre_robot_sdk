import pytest

from supre_robot_sdk.core.interfaces import RobotInterface
from supre_robot_sdk.hardware.base import HARDWARE_PLUGIN_MAP, HardwareInterface, register_hardware


def test_robot_interface_is_abstract():
    with pytest.raises(TypeError):
        RobotInterface()


def test_register_hardware_decorator():
    @register_hardware("TestHardware")
    class TestHardware(HardwareInterface):
        def init(self, config):
            return True

        def activate(self):
            return True

        def deactivate(self):
            return None

        def read(self):
            return [(0.0, 0.0)]

        def write(self, commands_positions):
            return None

        def get_joint_count(self):
            return 1

    assert HARDWARE_PLUGIN_MAP["TestHardware"] is TestHardware

