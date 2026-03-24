import textwrap

from supre_robot_sdk.core.hardware_manager import HardwareManager
from supre_robot_sdk.hardware.base import HardwareInterface, register_hardware


@register_hardware("FakeArmHardware")
class FakeArmHardware(HardwareInterface):
    def __init__(self):
        self.commands = []

    def init(self, config):
        self.config = config
        return True

    def activate(self):
        return True

    def deactivate(self):
        return None

    def read(self):
        return [(1.0, 0.1), (2.0, 0.2)]

    def write(self, commands_positions):
        self.commands.append(list(commands_positions))

    def get_joint_count(self):
        return 2


@register_hardware("FakeGripperHardware")
class FakeGripperHardware(HardwareInterface):
    def __init__(self):
        self.commands = []

    def init(self, config):
        self.config = config
        return True

    def activate(self):
        return True

    def deactivate(self):
        return None

    def read(self):
        return [(0.5, 0.9)]

    def write(self, commands_positions):
        self.commands.append(list(commands_positions))

    def get_joint_count(self):
        return 1


def write_config(tmp_path):
    path = tmp_path / "robot_config.yaml"
    path.write_text(
        textwrap.dedent(
            """
            joint_order:
              - joint_1
              - joint_2
              - joint_3
            hardware_interfaces:
              - name: arm
                type: FakeArmHardware
                config:
                  joints:
                    - { name: joint_1, parameters: { node_id: 1 } }
                    - { name: joint_2, parameters: { node_id: 2 } }
              - name: gripper
                type: FakeGripperHardware
                config:
                  joints:
                    - { name: joint_3, parameters: { slave_id: 9 } }
            """
        ),
        encoding="utf-8",
    )
    return path


def test_hardware_manager_read_and_write(tmp_path):
    manager = HardwareManager(write_config(tmp_path))
    manager.init()
    manager.activate()
    positions, forces = manager.read()
    assert positions == [1.0, 2.0, 0.5]
    assert forces == [0.1, 0.2, 0.9]
    manager.write([10.0, 20.0, 0.8])
    assert manager._hardware_instances[0].commands[-1] == [10.0, 20.0]
    assert manager._hardware_instances[1].commands[-1] == [0.8]

