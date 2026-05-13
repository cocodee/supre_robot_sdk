import textwrap

import pytest

from app.gravity_compensation import (
    GravityCompensationConfig,
    apply_startup_ramp,
    build_initial_position_targets,
    build_joint_trajectory,
    build_pinocchio_q,
    compute_gravity_torques_milli,
    limit_torque_step,
    load_gravity_compensation_config,
    validate_initial_pose_window,
    validate_position_watchdog,
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
            max_torque_step_milli: 12
            ramp_duration_s: 3
            max_position_step_deg: 4
            max_velocity_deg_s: 50
            initial_move_duration_s: 8
            initial_move_frequency: 20
            initial_move_max_step_deg: 1
            max_initial_deviation_deg:
              joint_1: 8
              joint_2: 9
            controlled_joints: [joint_1, joint_2]
            rated_torque_nm:
              joint_1: 20
              joint_2: 40
            torque_scale:
              joint_1: 0.5
              joint_2: 0.25
            initial_positions_deg:
              joint_1: 10
              joint_2: -20
            position_limits_deg:
              joint_1: {min: -90, max: 90}
              joint_2: {min: -45, max: 45}
            """
        ),
        encoding="utf-8",
    )

    config = load_gravity_compensation_config(config_path)

    assert str(config.urdf_path) == "robot.urdf"
    assert config.frequency == 200.0
    assert config.interpolation_period_ms == 5
    assert config.max_abs_torque_milli == 250.0
    assert config.max_torque_step_milli == 12.0
    assert config.ramp_duration_s == 3.0
    assert config.max_position_step_deg == 4.0
    assert config.max_velocity_deg_s == 50.0
    assert config.initial_move_duration_s == 8.0
    assert config.initial_move_frequency == 20.0
    assert config.initial_move_max_step_deg == 1.0
    assert config.controlled_joints == ["joint_1", "joint_2"]
    assert config.rated_torque_nm == {"joint_1": 20.0, "joint_2": 40.0}
    assert config.torque_scale == {"joint_1": 0.5, "joint_2": 0.25}
    assert config.initial_positions_deg == {"joint_1": 10.0, "joint_2": -20.0}
    assert config.max_initial_deviation_deg == {"joint_1": 8.0, "joint_2": 9.0}
    assert config.position_limits_deg == {"joint_1": (-90.0, 90.0), "joint_2": (-45.0, 45.0)}


def test_load_gravity_compensation_config_requires_rated_torque(tmp_path):
    config_path = tmp_path / "gravity.yaml"
    config_path.write_text(
        textwrap.dedent(
            """
            urdf_path: robot.urdf
            controlled_joints: [joint_1]
            rated_torque_nm: {}
            initial_positions_deg:
              joint_1: 0
            """
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="rated_torque_nm"):
        load_gravity_compensation_config(config_path)


def test_load_gravity_compensation_config_requires_urdf_path(tmp_path):
    config_path = tmp_path / "gravity.yaml"
    config_path.write_text(
        textwrap.dedent(
            """
            controlled_joints: [joint_1]
            rated_torque_nm:
              joint_1: 20
            """
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="urdf_path"):
        load_gravity_compensation_config(config_path)


def test_load_gravity_compensation_config_rejects_duplicate_controlled_joints(tmp_path):
    config_path = tmp_path / "gravity.yaml"
    config_path.write_text(
        textwrap.dedent(
            """
            urdf_path: robot.urdf
            controlled_joints: [joint_1, joint_1]
            rated_torque_nm:
              joint_1: 20
            """
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="duplicate"):
        load_gravity_compensation_config(config_path)


def test_load_gravity_compensation_config_rejects_unknown_torque_scale_joint(tmp_path):
    config_path = tmp_path / "gravity.yaml"
    config_path.write_text(
        textwrap.dedent(
            """
            urdf_path: robot.urdf
            controlled_joints: [joint_1]
            rated_torque_nm:
              joint_1: 20
            torque_scale:
              joint_2: 0.1
            """
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="not controlled"):
        load_gravity_compensation_config(config_path)


def test_load_gravity_compensation_config_rejects_non_finite_safety_values(tmp_path):
    config_path = tmp_path / "gravity.yaml"
    config_path.write_text(
        textwrap.dedent(
            """
            urdf_path: robot.urdf
            max_torque_step_milli: .inf
            controlled_joints: [joint_1]
            rated_torque_nm:
              joint_1: 20
            """
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="max_torque_step_milli"):
        load_gravity_compensation_config(config_path)


def test_load_gravity_compensation_config_rejects_invalid_position_limit(tmp_path):
    config_path = tmp_path / "gravity.yaml"
    config_path.write_text(
        textwrap.dedent(
            """
            urdf_path: robot.urdf
            controlled_joints: [joint_1]
            rated_torque_nm:
              joint_1: 20
            position_limits_deg:
              joint_1: {min: 10, max: -10}
            """
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="min greater than max"):
        load_gravity_compensation_config(config_path)


def test_load_gravity_compensation_config_rejects_unknown_position_limit_joint(tmp_path):
    config_path = tmp_path / "gravity.yaml"
    config_path.write_text(
        textwrap.dedent(
            """
            urdf_path: robot.urdf
            controlled_joints: [joint_1]
            rated_torque_nm:
              joint_1: 20
            position_limits_deg:
              joint_2: {min: -10, max: 10}
            """
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="not controlled"):
        load_gravity_compensation_config(config_path)


def test_load_gravity_compensation_config_requires_initial_position(tmp_path):
    config_path = tmp_path / "gravity.yaml"
    config_path.write_text(
        textwrap.dedent(
            """
            urdf_path: robot.urdf
            controlled_joints: [joint_1]
            rated_torque_nm:
              joint_1: 20
            initial_positions_deg: {}
            """
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="initial_positions_deg"):
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
        max_torque_step_milli=10.0,
        ramp_duration_s=5.0,
        max_position_step_deg=5.0,
        max_velocity_deg_s=90.0,
        controlled_joints=["joint_1", "joint_2"],
        rated_torque_nm={"joint_1": 20.0, "joint_2": 40.0},
        torque_scale={"joint_1": 1.0, "joint_2": 1.0},
        position_limits_deg={},
    )

    commands = compute_gravity_torques_milli(
        FakePinocchio,
        FakeModel(),
        object(),
        {"joint_1": 0.0, "joint_2": 0.0},
        config,
    )

    assert commands == {"joint_1": 100.0, "joint_2": -150.0}


def test_compute_gravity_torques_milli_applies_torque_scale():
    config = GravityCompensationConfig(
        urdf_path="robot.urdf",
        frequency=250.0,
        interpolation_period_ms=4,
        max_abs_torque_milli=150.0,
        max_torque_step_milli=10.0,
        ramp_duration_s=5.0,
        max_position_step_deg=5.0,
        max_velocity_deg_s=90.0,
        controlled_joints=["joint_1", "joint_2"],
        rated_torque_nm={"joint_1": 20.0, "joint_2": 40.0},
        torque_scale={"joint_1": 0.5, "joint_2": 0.25},
        position_limits_deg={},
    )

    commands = compute_gravity_torques_milli(
        FakePinocchio,
        FakeModel(),
        object(),
        {"joint_1": 0.0, "joint_2": 0.0},
        config,
    )

    assert commands == {"joint_1": 50.0, "joint_2": -50.0}


def test_compute_gravity_torques_milli_rejects_missing_urdf_joint():
    config = GravityCompensationConfig(
        urdf_path="robot.urdf",
        frequency=250.0,
        interpolation_period_ms=4,
        max_abs_torque_milli=150.0,
        max_torque_step_milli=10.0,
        ramp_duration_s=5.0,
        max_position_step_deg=5.0,
        max_velocity_deg_s=90.0,
        controlled_joints=["missing_joint"],
        rated_torque_nm={"missing_joint": 20.0},
        torque_scale={"missing_joint": 1.0},
        position_limits_deg={},
    )

    with pytest.raises(ValueError, match="URDF model does not contain"):
        compute_gravity_torques_milli(
            FakePinocchio,
            FakeModel(),
            object(),
            {"missing_joint": 0.0},
            config,
        )


def test_compute_gravity_torques_milli_requires_position_feedback():
    config = GravityCompensationConfig(
        urdf_path="robot.urdf",
        frequency=250.0,
        interpolation_period_ms=4,
        max_abs_torque_milli=150.0,
        max_torque_step_milli=10.0,
        ramp_duration_s=5.0,
        max_position_step_deg=5.0,
        max_velocity_deg_s=90.0,
        controlled_joints=["joint_1"],
        rated_torque_nm={"joint_1": 20.0},
        torque_scale={"joint_1": 1.0},
        position_limits_deg={},
        initial_positions_deg={"joint_1": 0.0},
        max_initial_deviation_deg={"joint_1": 100.0},
    )

    with pytest.raises(KeyError, match="missing controlled joint"):
        compute_gravity_torques_milli(
            FakePinocchio,
            FakeModel(),
            object(),
            {},
            config,
        )


def test_apply_startup_ramp_scales_until_duration():
    assert apply_startup_ramp({"joint_1": 100.0}, elapsed_s=2.5, ramp_duration_s=5.0) == {"joint_1": 50.0}
    assert apply_startup_ramp({"joint_1": 100.0}, elapsed_s=6.0, ramp_duration_s=5.0) == {"joint_1": 100.0}


def test_limit_torque_step_clamps_per_joint_delta():
    commands = limit_torque_step(
        {"joint_1": 50.0, "joint_2": -50.0},
        {"joint_1": 20.0, "joint_2": -20.0},
        max_torque_step_milli=10.0,
    )

    assert commands == {"joint_1": 30.0, "joint_2": -30.0}


def test_validate_position_watchdog_rejects_large_position_step():
    config = GravityCompensationConfig(
        urdf_path="robot.urdf",
        frequency=250.0,
        interpolation_period_ms=4,
        max_abs_torque_milli=150.0,
        max_torque_step_milli=10.0,
        ramp_duration_s=5.0,
        max_position_step_deg=5.0,
        max_velocity_deg_s=1000.0,
        controlled_joints=["joint_1"],
        rated_torque_nm={"joint_1": 20.0},
        torque_scale={"joint_1": 1.0},
        position_limits_deg={},
        initial_positions_deg={"joint_1": 0.0},
        max_initial_deviation_deg={"joint_1": 100.0},
    )

    with pytest.raises(RuntimeError, match="Position watchdog"):
        validate_position_watchdog({"joint_1": 6.0}, {"joint_1": 0.0}, elapsed_s=1.0, config=config)


def test_validate_position_watchdog_rejects_large_velocity():
    config = GravityCompensationConfig(
        urdf_path="robot.urdf",
        frequency=250.0,
        interpolation_period_ms=4,
        max_abs_torque_milli=150.0,
        max_torque_step_milli=10.0,
        ramp_duration_s=5.0,
        max_position_step_deg=5.0,
        max_velocity_deg_s=20.0,
        controlled_joints=["joint_1"],
        rated_torque_nm={"joint_1": 20.0},
        torque_scale={"joint_1": 1.0},
        position_limits_deg={},
        initial_positions_deg={"joint_1": 0.0},
        max_initial_deviation_deg={"joint_1": 100.0},
    )

    with pytest.raises(RuntimeError, match="Velocity watchdog"):
        validate_position_watchdog({"joint_1": 3.0}, {"joint_1": 0.0}, elapsed_s=0.1, config=config)


def test_validate_position_watchdog_rejects_absolute_position_limit():
    config = GravityCompensationConfig(
        urdf_path="robot.urdf",
        frequency=250.0,
        interpolation_period_ms=4,
        max_abs_torque_milli=150.0,
        max_torque_step_milli=10.0,
        ramp_duration_s=5.0,
        max_position_step_deg=5.0,
        max_velocity_deg_s=90.0,
        controlled_joints=["joint_1"],
        rated_torque_nm={"joint_1": 20.0},
        torque_scale={"joint_1": 1.0},
        position_limits_deg={"joint_1": (-10.0, 10.0)},
        initial_positions_deg={"joint_1": 0.0},
        max_initial_deviation_deg={"joint_1": 100.0},
    )

    with pytest.raises(RuntimeError, match="Position limit"):
        validate_position_watchdog({"joint_1": 12.0}, None, elapsed_s=1.0, config=config)


def test_build_initial_position_targets_only_changes_controlled_joints():
    config = GravityCompensationConfig(
        urdf_path="robot.urdf",
        frequency=250.0,
        interpolation_period_ms=4,
        max_abs_torque_milli=150.0,
        max_torque_step_milli=10.0,
        ramp_duration_s=5.0,
        max_position_step_deg=5.0,
        max_velocity_deg_s=90.0,
        controlled_joints=["joint_1"],
        rated_torque_nm={"joint_1": 20.0},
        torque_scale={"joint_1": 1.0},
        initial_positions_deg={"joint_1": 15.0},
        max_initial_deviation_deg={"joint_1": 5.0},
    )

    targets = build_initial_position_targets({"joint_1": 0.0, "joint_2": 30.0}, config)

    assert targets == {"joint_1": 15.0, "joint_2": 30.0}


def test_build_joint_trajectory_respects_duration_and_max_step():
    trajectory = build_joint_trajectory(
        {"joint_1": 0.0, "joint_2": 5.0},
        {"joint_1": 10.0, "joint_2": 5.0},
        duration=1.0,
        frequency=2.0,
        max_step=3.0,
    )

    assert len(trajectory) == 4
    assert trajectory[-1] == {"joint_1": 10.0, "joint_2": 5.0}
    assert max(
        abs(step["joint_1"] - previous["joint_1"])
        for previous, step in zip([{"joint_1": 0.0}] + trajectory[:-1], trajectory)
    ) <= 3.0


def test_validate_initial_pose_window_rejects_large_deviation():
    config = GravityCompensationConfig(
        urdf_path="robot.urdf",
        frequency=250.0,
        interpolation_period_ms=4,
        max_abs_torque_milli=150.0,
        max_torque_step_milli=10.0,
        ramp_duration_s=5.0,
        max_position_step_deg=5.0,
        max_velocity_deg_s=90.0,
        controlled_joints=["joint_1"],
        rated_torque_nm={"joint_1": 20.0},
        torque_scale={"joint_1": 1.0},
        initial_positions_deg={"joint_1": 10.0},
        max_initial_deviation_deg={"joint_1": 3.0},
    )

    with pytest.raises(RuntimeError, match="Initial pose window"):
        validate_initial_pose_window({"joint_1": 15.0}, config)


def test_validate_robot_supports_torque_control_rejects_unsupported_joint():
    with pytest.raises(ValueError, match="do not support torque control"):
        validate_robot_supports_torque_control(FakeRobot(), ["joint_1", "gripper"])


def test_validate_robot_supports_torque_control_rejects_missing_joint():
    with pytest.raises(ValueError, match="missing from robot joint_order"):
        validate_robot_supports_torque_control(FakeRobot(), ["joint_1", "missing_joint"])
