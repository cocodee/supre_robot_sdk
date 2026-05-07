import pytest

from app.zero_all_joints import (
    build_joint_trajectory,
    build_zero_trajectory,
    load_motion_config,
    parse_position_array,
    validate_positions,
)


def test_build_zero_trajectory_ends_at_zero_and_limits_step_size():
    trajectory = build_zero_trajectory(
        {"joint_a": 5.0, "joint_b": -2.5},
        duration=1.0,
        frequency=2.0,
        max_step=1.0,
    )

    assert len(trajectory) == 5
    assert trajectory[-1] == {"joint_a": 0.0, "joint_b": -0.0}

    previous = {"joint_a": 5.0, "joint_b": -2.5}
    for command in trajectory:
        assert abs(command["joint_a"] - previous["joint_a"]) <= 1.0 + 1e-12
        assert abs(command["joint_b"] - previous["joint_b"]) <= 1.0 + 1e-12
        previous = command


def test_validate_positions_rejects_unsafe_values():
    with pytest.raises(ValueError, match="exceeds safety limit"):
        validate_positions({"joint_a": 361.0}, max_abs_position=360.0)

    with pytest.raises(ValueError, match="not finite"):
        validate_positions({"joint_a": float("nan")}, max_abs_position=360.0)


def test_parse_position_array_uses_joint_order():
    positions = parse_position_array([1.0, -2.5, 3], ["j1", "j2", "j3"])

    assert positions == {"j1": 1.0, "j2": -2.5, "j3": 3.0}


def test_parse_position_array_rejects_wrong_length():
    with pytest.raises(ValueError, match="Expected 3 target positions"):
        parse_position_array([1, 2], ["j1", "j2", "j3"])


def test_load_motion_config_reads_array_of_arrays(tmp_path):
    config_path = tmp_path / "motion.yaml"
    config_path.write_text(
        """
        target_duration: 4.5
        target_positions:
          - [1, 2, 3]
          - [4, 5, 6]
        """,
        encoding="utf-8",
    )

    duration, targets = load_motion_config(config_path, ["j1", "j2", "j3"])

    assert duration == 4.5
    assert targets == [
        {"j1": 1.0, "j2": 2.0, "j3": 3.0},
        {"j1": 4.0, "j2": 5.0, "j3": 6.0},
    ]


def test_load_motion_config_rejects_non_array_target(tmp_path):
    config_path = tmp_path / "motion.yaml"
    config_path.write_text(
        """
        target_positions:
          - 1
        """,
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match=r"target_positions\[0\]"):
        load_motion_config(config_path, ["j1"])


def test_load_motion_config_rejects_missing_yaml_sequence_space(tmp_path):
    config_path = tmp_path / "motion.yaml"
    config_path.write_text(
        """
        target_positions: -[1]
        """,
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match=r"use '- \[\.\.\]'"):
        load_motion_config(config_path, ["j1"])


def test_build_joint_trajectory_moves_from_zero_to_target():
    trajectory = build_joint_trajectory(
        {"joint_a": 0.0, "joint_b": 0.0},
        {"joint_a": 3.0, "joint_b": -1.5},
        duration=1.0,
        frequency=2.0,
        max_step=1.0,
    )

    assert len(trajectory) == 3
    assert trajectory[-1] == {"joint_a": 3.0, "joint_b": -1.5}

    previous = {"joint_a": 0.0, "joint_b": 0.0}
    for command in trajectory:
        assert abs(command["joint_a"] - previous["joint_a"]) <= 1.0 + 1e-12
        assert abs(command["joint_b"] - previous["joint_b"]) <= 1.0 + 1e-12
        previous = command
