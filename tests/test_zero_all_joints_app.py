import pytest

from app.zero_all_joints import build_zero_trajectory, validate_positions


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
