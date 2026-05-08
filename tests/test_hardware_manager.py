import textwrap

import pytest

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


@register_hardware("FakeTorqueArmHardware")
class FakeTorqueArmHardware(FakeArmHardware):
    def __init__(self):
        super().__init__()
        self.torque_configs = []
        self.torque_commands = []

    def supports_torque_control(self):
        return True

    def configure_torque_control(self, interpolation_period_ms=4, use_sync=True):
        self.torque_configs.append((interpolation_period_ms, use_sync))

    def write_torques(self, commands_torque_milli):
        self.torque_commands.append(list(commands_torque_milli))


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


def test_hardware_manager_applies_joint_direction_and_calibration(tmp_path):
    path = tmp_path / "robot_config.yaml"
    path.write_text(
        textwrap.dedent(
            """
            joint_order:
              - joint_1
              - joint_2
              - joint_3
            joint_direction: [1, -1, 1]
            calibration:
              - joint_name: joint_2
                min_position: -5.0
                max_position: 5.0
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

    manager = HardwareManager(path)
    manager.init()
    manager.activate()

    positions, _ = manager.read()
    assert positions == [1.0, -2.0, 0.5]

    manager.write([3.0, -4.0, 0.8])
    assert manager._hardware_instances[0].commands[-1] == [3.0, 4.0]
    assert manager._hardware_instances[1].commands[-1] == [0.8]

    with pytest.raises(ValueError, match="outside calibration range"):
        manager.write([3.0, -6.0, 0.8])


def test_hardware_manager_writes_torques_with_joint_direction(tmp_path):
    path = tmp_path / "robot_config.yaml"
    path.write_text(
        textwrap.dedent(
            """
            joint_order:
              - joint_1
              - joint_2
              - joint_3
            joint_direction: [1, -1, 1]
            hardware_interfaces:
              - name: arm
                type: FakeTorqueArmHardware
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

    manager = HardwareManager(path)
    manager.init()
    manager.activate()

    assert manager.supports_torque_control() is True
    assert manager.supports_torque_control("joint_1") is True
    assert manager.supports_torque_control("joint_3") is False
    manager.configure_torque_control(interpolation_period_ms=4, use_sync=False)
    manager.write_torques([10.0, -20.0, None])

    arm = manager._hardware_instances[0]
    assert arm.torque_configs == [(4, False)]
    assert arm.torque_commands == [[10.0, 20.0]]

    with pytest.raises(RuntimeError, match="does not support torque control"):
        manager.write_torques([None, None, 1.0])
