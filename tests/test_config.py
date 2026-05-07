import textwrap

import pytest

from supre_robot_sdk.core.config import load_robot_config
from supre_robot_sdk.exceptions import ConfigurationError


def write_config(tmp_path, content: str):
    path = tmp_path / "robot_config.yaml"
    path.write_text(textwrap.dedent(content), encoding="utf-8")
    return path


def test_load_valid_config(tmp_path):
    path = write_config(
        tmp_path,
        """
        joint_order:
          - joint_1
          - joint_2
        hardware_interfaces:
          - name: motors
            type: EyouMotorHardware
            config:
              can_device_index: 1
              can_baud_rate: "1M"
              joints:
                - { name: joint_1, parameters: { node_id: 1 } }
                - { name: joint_2, parameters: { node_id: 2 } }
        """,
    )

    config = load_robot_config(path)
    assert config.joint_order == ["joint_1", "joint_2"]
    assert config.joint_direction == [1.0, 1.0]
    assert len(config.hardware_interfaces) == 1


def test_load_config_reads_joint_direction_and_calibration(tmp_path):
    path = write_config(
        tmp_path,
        """
        joint_order: [joint_1, joint_2]
        joint_direction: [1, -1]
        calibration:
          - joint_name: joint_1
            min_position: -10.0
            max_position: 20.0
          - joint_name: joint_2
            min_position: 0.0
            max_position: 1.0
        hardware_interfaces:
          - name: motors
            type: EyouMotorHardware
            config:
              can_device_index: 1
              can_baud_rate: "1M"
              joints:
                - { name: joint_1, parameters: { node_id: 1 } }
                - { name: joint_2, parameters: { node_id: 2 } }
        """,
    )

    config = load_robot_config(path)

    assert config.joint_direction == [1.0, -1.0]
    assert config.calibration["joint_1"].min_position == -10.0
    assert config.calibration["joint_2"].max_position == 1.0


def test_joint_direction_must_match_joint_order_length(tmp_path):
    path = write_config(
        tmp_path,
        """
        joint_order: [joint_1, joint_2]
        joint_direction: [1]
        hardware_interfaces:
          - name: motors
            type: EyouMotorHardware
            config:
              joints:
                - { name: joint_1, parameters: { node_id: 1 } }
                - { name: joint_2, parameters: { node_id: 2 } }
        """,
    )

    with pytest.raises(ConfigurationError, match="joint_direction"):
        load_robot_config(path)


def test_calibration_rejects_unknown_joint(tmp_path):
    path = write_config(
        tmp_path,
        """
        joint_order: [joint_1]
        calibration:
          - joint_name: joint_2
            min_position: 0.0
            max_position: 1.0
        hardware_interfaces:
          - name: motors
            type: EyouMotorHardware
            config:
              joints:
                - { name: joint_1, parameters: { node_id: 1 } }
        """,
    )

    with pytest.raises(ConfigurationError, match="unknown joint"):
        load_robot_config(path)


def test_unknown_hardware_type_raises(tmp_path):
    path = write_config(
        tmp_path,
        """
        joint_order: [joint_1]
        hardware_interfaces:
          - name: motors
            type: UnknownHardware
            config:
              joints:
                - { name: joint_1, parameters: { node_id: 1 } }
        """,
    )
    with pytest.raises(ConfigurationError, match="Unknown hardware type"):
        load_robot_config(path)


def test_duplicate_joint_mapping_raises(tmp_path):
    path = write_config(
        tmp_path,
        """
        joint_order: [joint_1]
        hardware_interfaces:
          - name: motors_a
            type: EyouMotorHardware
            config:
              can_device_index: 1
              can_baud_rate: "1M"
              joints:
                - { name: joint_1, parameters: { node_id: 1 } }
          - name: motors_b
            type: EyouMotorHardware
            config:
              can_device_index: 1
              can_baud_rate: "1M"
              joints:
                - { name: joint_1, parameters: { node_id: 2 } }
        """,
    )
    with pytest.raises(ConfigurationError, match="assigned by both"):
        load_robot_config(path)


def test_unmapped_joint_raises(tmp_path):
    path = write_config(
        tmp_path,
        """
        joint_order: [joint_1, joint_2]
        hardware_interfaces:
          - name: motors
            type: EyouMotorHardware
            config:
              can_device_index: 1
              can_baud_rate: "1M"
              joints:
                - { name: joint_1, parameters: { node_id: 1 } }
        """,
    )
    with pytest.raises(ConfigurationError, match="Unmapped joints"):
        load_robot_config(path)
