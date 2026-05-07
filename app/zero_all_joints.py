#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import yaml

from supre_robot_sdk import SupreRobot


def validate_positions(
    positions: dict[str, float],
    *,
    max_abs_position: float,
) -> None:
    for joint_name, value in positions.items():
        if not math.isfinite(value):
            raise ValueError(f"{joint_name} position is not finite: {value!r}")
        if abs(value) > max_abs_position:
            raise ValueError(
                f"{joint_name} position {value:.3f} exceeds safety limit "
                f"+/-{max_abs_position:.3f}; inspect the robot before moving."
            )


def parse_position_array(raw_positions: list[float], joint_order: list[str]) -> dict[str, float]:
    values = [float(value) for value in raw_positions]
    if len(values) != len(joint_order):
        raise ValueError(
            f"Expected {len(joint_order)} target positions for joint_order, got {len(values)}."
        )
    return dict(zip(joint_order, values))


def load_motion_config(path: str | Path, joint_order: list[str]) -> tuple[float | None, list[dict[str, float]]]:
    config_path = Path(path)
    if not config_path.exists():
        return None, []

    with config_path.open("r", encoding="utf-8") as file:
        raw = yaml.safe_load(file) or {}
    if not isinstance(raw, dict):
        raise ValueError("Motion config root must be a mapping.")

    target_duration = raw.get("target_duration")
    if target_duration is not None:
        target_duration = float(target_duration)
        if target_duration <= 0:
            raise ValueError("target_duration must be greater than 0")

    raw_targets = raw.get("target_positions", [])
    if raw_targets is None:
        raw_targets = []
    if not isinstance(raw_targets, list):
        raise ValueError("target_positions must be an array of joint position arrays.")

    target_positions = []
    for index, raw_target in enumerate(raw_targets, start=1):
        if not isinstance(raw_target, list):
            raise ValueError(f"target_positions[{index}] must be a joint position array.")
        target_positions.append(parse_position_array(raw_target, joint_order))
    return target_duration, target_positions


def build_joint_trajectory(
    start_positions: dict[str, float],
    target_positions: dict[str, float],
    *,
    duration: float,
    frequency: float,
    max_step: float,
) -> list[dict[str, float]]:
    if duration <= 0:
        raise ValueError("duration must be greater than 0")
    if frequency <= 0:
        raise ValueError("frequency must be greater than 0")
    if max_step <= 0:
        raise ValueError("max_step must be greater than 0")
    if set(start_positions) != set(target_positions):
        raise ValueError("start and target positions must contain the same joints")

    largest_move = max(
        (abs(target_positions[joint] - value) for joint, value in start_positions.items()),
        default=0.0,
    )
    min_steps_for_delta = max(1, math.ceil(largest_move / max_step))
    requested_steps = max(1, math.ceil(duration * frequency))
    steps = max(requested_steps, min_steps_for_delta)

    trajectory: list[dict[str, float]] = []
    for step in range(1, steps + 1):
        ratio = step / steps
        trajectory.append(
            {
                joint: value + (target_positions[joint] - value) * ratio
                for joint, value in start_positions.items()
            }
        )
    return trajectory


def build_zero_trajectory(
    start_positions: dict[str, float],
    *,
    duration: float,
    frequency: float,
    max_step: float,
) -> list[dict[str, float]]:
    return build_joint_trajectory(
        start_positions,
        {joint: 0.0 for joint in start_positions},
        duration=duration,
        frequency=frequency,
        max_step=max_step,
    )


def execute_trajectory(
    robot: SupreRobot,
    trajectory: list[dict[str, float]],
    *,
    frequency: float,
) -> None:
    period = 1.0 / frequency
    for command in trajectory:
        started_at = time.perf_counter()
        robot.send_joint_positions(command)
        elapsed = time.perf_counter() - started_at
        if elapsed < period:
            time.sleep(period - elapsed)


def move_all_joints_to_zero_then_target(args: argparse.Namespace) -> None:
    config_path = Path(args.config)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file does not exist: {config_path}")

    robot = SupreRobot(
        config_path,
        control_frequency=args.frequency,
        use_interpolation=True,
    )

    print(f"Connecting with config: {config_path}")
    robot.connect()
    try:
        robot.set_enable_torque(True)
        start_positions = robot.get_joint_positions()
        validate_positions(start_positions, max_abs_position=args.max_abs_position)
        trajectory = build_zero_trajectory(
            start_positions,
            duration=args.duration,
            frequency=args.frequency,
            max_step=args.max_step,
        )

        print(
            f"Moving {len(start_positions)} joints to zero over "
            f"{len(trajectory) / args.frequency:.2f}s at {args.frequency:.1f} Hz"
        )
        execute_trajectory(robot, trajectory, frequency=args.frequency)
        robot.send_joint_positions({joint: 0.0 for joint in start_positions})
        print("All commanded joint positions are zero.")

        configured_duration, target_sequence = load_motion_config(args.motion_config, robot.joint_order)
        target_duration = configured_duration if configured_duration is not None else args.target_duration
        for target_index, target_positions in enumerate(target_sequence, start=1):
            validate_positions(target_positions, max_abs_position=args.max_abs_position)
            target_trajectory = build_joint_trajectory(
                robot.get_joint_positions(),
                target_positions,
                duration=target_duration,
                frequency=args.frequency,
                max_step=args.max_step,
            )
            print(
                f"Moving to configured target {target_index}/{len(target_sequence)} over "
                f"{len(target_trajectory) / args.frequency:.2f}s"
            )
            execute_trajectory(robot, target_trajectory, frequency=args.frequency)
            robot.send_joint_positions(target_positions)
            print(f"Configured target {target_index} reached.")
    finally:
        robot.disconnect()
        print("Robot disconnected.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Connect, activate, and slowly command every configured joint to zero."
    )
    parser.add_argument(
        "--config",
        default="examples/robot_config.yaml",
        help="Robot YAML config path. Default: examples/robot_config.yaml",
    )
    parser.add_argument("--duration", type=float, default=10.0, help="Minimum move duration in seconds.")
    parser.add_argument(
        "--target-duration",
        type=float,
        default=10.0,
        help="Fallback target move duration if motion config does not set target_duration.",
    )
    parser.add_argument("--frequency", type=float, default=30.0, help="Command frequency in Hz.")
    parser.add_argument(
        "--motion-config",
        default="app/zero_all_joints_config.yaml",
        help=(
            "YAML config containing target_positions as an array of joint position arrays. "
            "Default: app/zero_all_joints_config.yaml"
        ),
    )
    parser.add_argument(
        "--max-abs-position",
        type=float,
        default=360.0,
        help="Abort if any starting joint position magnitude exceeds this value.",
    )
    parser.add_argument(
        "--max-step",
        type=float,
        default=2.0,
        help="Maximum per-command joint delta in the same units reported by hardware.",
    )
    return parser.parse_args()


def main() -> None:
    move_all_joints_to_zero_then_target(parse_args())


if __name__ == "__main__":
    main()
