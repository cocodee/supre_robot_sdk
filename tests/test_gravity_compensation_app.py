import textwrap

import pytest

from app.gravity_compensation import (
    GravityCompensationConfig,
    build_pinocchio_q,
    compute_gravity_torques_milli,
    load_gravity_compensation_config,
    validate_robot_supports_torque_control,
)


class FakeModel:
    names = ["universe", "joint_1", "joint_2"]
    idx_qs = [0, 0, 1]
    idx_vs = [0, 0, 1]
    nqs = [0, 1, 1]
    nvs = [0, 1, 1]

    def createData(self):
        return object()

    def getJointId(self, joint_name):
        try:
            return self.names.index(joint_name)
        except ValueError:
            return len(self.names)


class FakePinocchio:
    @staticmethod
    def neutral(model):
        return [0.0] * 2

    @staticmethod
    def computeGeneralizedGravity(model, data, q):
        return [2.0, -8.0]


class FakeRobot:
    joint_order = ["joint_1", "joint_2", "gripper"]

    def supports_torque_control(self, joint_name=None):
        return joint_name in {"joint_1", "joint_2"}


def test_load_gravity_compensation_config(tmp_path):
    config_path = tmp_path / "gravity.yaml"
    config_path.write_text(
        textwrap.dedent(
            """
            urdf_path: robot.urdf
            frequency: 200
            interpolation_period_ms: 5
            max_abs_torque_milli: 250
            controlled_joints: [joint_1, joint_2]
            rated_torque_nm:
              joint_1: 20
              joint_2: 40
            """
        ),
        encoding="utf-8",
    )

    config = load_gravity_compensation_config(config_path)

    assert str(config.urdf_path) == "robot.urdf"
    assert config.frequency == 200.0
    assert config.interpolation_period_ms == 5
    assert config.max_abs_torque_milli == 250.0
    assert config.controlled_joints == ["joint_1", "joint_2"]
    assert config.rated_torque_nm == {"joint_1": 20.0, "joint_2": 40.0}


def test_load_gravity_compensation_config_requires_rated_torque(tmp_path):
    config_path = tmp_path / "gravity.yaml"
    config_path.write_text(
        textwrap.dedent(
            """
            urdf_path: robot.urdf
            controlled_joints: [joint_1]
            rated_torque_nm: {}
            """
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="rated_torque_nm"):
        load_gravity_compensation_config(config_path)


def test_build_pinocchio_q_converts_degrees_to_radians():
    q = build_pinocchio_q(FakePinocchio, FakeModel(), {"joint_1": 90.0, "joint_2": -180.0})

    assert q == pytest.approx([1.57079632679, -3.14159265359])


def test_compute_gravity_torques_milli_converts_and_clamps():
    config = GravityCompensationConfig(
        urdf_path="robot.urdf",
        frequency=250.0,
        interpolation_period_ms=4,
        max_abs_torque_milli=150.0,
        controlled_joints=["joint_1", "joint_2"],
        rated_torque_nm={"joint_1": 20.0, "joint_2": 40.0},
    )

    commands = compute_gravity_torques_milli(
        FakePinocchio,
        FakeModel(),
        object(),
        {"joint_1": 0.0, "joint_2": 0.0},
        config,
    )

    assert commands == {"joint_1": 100.0, "joint_2": -150.0}


def test_validate_robot_supports_torque_control_rejects_unsupported_joint():
    with pytest.raises(ValueError, match="do not support torque control"):
        validate_robot_supports_torque_control(FakeRobot(), ["joint_1", "gripper"])
