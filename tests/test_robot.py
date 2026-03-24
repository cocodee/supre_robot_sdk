import textwrap

from supre_robot_sdk import SupreRobot
from supre_robot_sdk.hardware.base import HardwareInterface, register_hardware


@register_hardware("RobotArmHardware")
class RobotArmHardware(HardwareInterface):
    def __init__(self):
        self.state = [1.0, 2.0]
        self.commands = []

    def init(self, config):
        self.config = config
        return True

    def activate(self):
        return True

    def deactivate(self):
        return None

    def read(self):
        return [(self.state[0], 0.1), (self.state[1], 0.2)]

    def write(self, commands_positions):
        self.commands.append(list(commands_positions))
        self.state = [float(value) for value in commands_positions]

    def get_joint_count(self):
        return 2


@register_hardware("RobotGripperHardware")
class RobotGripperHardware(HardwareInterface):
    def __init__(self):
        self.state = [0.5]
        self.commands = []

    def init(self, config):
        self.config = config
        return True

    def activate(self):
        return True

    def deactivate(self):
        return None

    def read(self):
        return [(self.state[0], 0.3)]

    def write(self, commands_positions):
        self.commands.append(list(commands_positions))
        self.state = [float(value) for value in commands_positions if value is not None]

    def get_joint_count(self):
        return 1


def write_config(tmp_path):
    path = tmp_path / "robot_config.yaml"
    path.write_text(
        textwrap.dedent(
            """
            joint_order:
              - left_arm_joint_1
              - right_arm_joint_1
              - left_arm_joint_7
            hardware_interfaces:
              - name: arm
                type: RobotArmHardware
                config:
                  joints:
                    - { name: left_arm_joint_1, parameters: { node_id: 1 } }
                    - { name: right_arm_joint_1, parameters: { node_id: 2 } }
              - name: gripper
                type: RobotGripperHardware
                config:
                  joints:
                    - { name: left_arm_joint_7, parameters: { slave_id: 3 } }
            """
        ),
        encoding="utf-8",
    )
    return path


def test_supre_robot_facade(tmp_path, monkeypatch):
    monkeypatch.setattr("supre_robot_sdk.core.robot.time.sleep", lambda *_args: None)
    robot = SupreRobot(write_config(tmp_path), control_frequency=20)
    robot.connect()
    assert robot.is_connected is True
    assert robot.get_joint_positions()["left_arm_joint_1"] == 1.0
    robot.move_joint("right_arm_joint_1", 9.0)
    assert robot.get_joint_positions()["right_arm_joint_1"] == 9.0
    robot.open_gripper("left")
    assert robot.get_joint_positions()["left_arm_joint_7"] == 1.0
    robot.execute_trajectory({"left_arm_joint_1": 3.0}, duration=0.05)
    assert robot.get_joint_positions()["left_arm_joint_1"] == 3.0
    robot.disconnect()
    assert robot.is_connected is False

