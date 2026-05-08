import textwrap

from supre_robot_sdk import SupreRobot
from supre_robot_sdk.hardware.base import HardwareInterface, register_hardware


@register_hardware("RobotArmHardware")
class RobotArmHardware(HardwareInterface):
    def __init__(self):
        self.state = [1.0, 2.0]
        self.commands = []
        self.torque_configs = []
        self.torque_commands = []
        self.control_mode = "position"

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

    def supports_torque_control(self):
        return True

    def configure_torque_control(self, interpolation_period_ms=4, use_sync=True):
        self.control_mode = "torque"
        self.torque_configs.append((interpolation_period_ms, use_sync))

    def write_torques(self, commands_torque_milli):
        self.torque_commands.append(list(commands_torque_milli))

    def get_control_mode(self):
        return self.control_mode


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
    assert robot.supports_torque_control() is True
    assert robot.supports_torque_control("right_arm_joint_1") is True
    assert robot.supports_torque_control("left_arm_joint_7") is False
    assert robot.get_control_mode("right_arm_joint_1") == "position"
    assert robot.get_control_mode() == {
        "left_arm_joint_1": "position",
        "right_arm_joint_1": "position",
        "left_arm_joint_7": "position",
    }
    robot.configure_torque_control(interpolation_period_ms=4, use_sync=False)
    assert robot.get_control_mode("right_arm_joint_1") == "torque"
    assert robot.get_control_mode() == {
        "left_arm_joint_1": "torque",
        "right_arm_joint_1": "torque",
        "left_arm_joint_7": "position",
    }
    robot.send_joint_torques({"left_arm_joint_1": 10.0, "right_arm_joint_1": -20.0})
    assert robot._manager._hardware_instances[0].torque_configs == [(4, False)]
    assert robot._manager._hardware_instances[0].torque_commands[-1] == [10.0, -20.0]
    robot.send_joint_torque("right_arm_joint_1", 5.0)
    assert robot._manager._hardware_instances[0].torque_commands[-1] == [0.0, 5.0]
    robot.disconnect()
    assert robot.is_connected is False
