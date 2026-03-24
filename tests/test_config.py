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
    assert len(config.hardware_interfaces) == 1


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

