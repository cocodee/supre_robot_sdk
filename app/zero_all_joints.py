#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

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


def build_zero_trajectory(
    start_positions: dict[str, float],
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

    largest_move = max((abs(value) for value in start_positions.values()), default=0.0)
    min_steps_for_delta = max(1, math.ceil(largest_move / max_step))
    requested_steps = max(1, math.ceil(duration * frequency))
    steps = max(requested_steps, min_steps_for_delta)

    trajectory: list[dict[str, float]] = []
    for step in range(1, steps + 1):
        ratio = 1.0 - (step / steps)
        trajectory.append({joint: value * ratio for joint, value in start_positions.items()})
    return trajectory


def move_all_joints_to_zero(args: argparse.Namespace) -> None:
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
        period = 1.0 / args.frequency
        for command in trajectory:
            started_at = time.perf_counter()
            robot.send_joint_positions(command)
            elapsed = time.perf_counter() - started_at
            if elapsed < period:
                time.sleep(period - elapsed)

        robot.send_joint_positions({joint: 0.0 for joint in start_positions})
        print("All commanded joint positions are zero.")
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
    parser.add_argument("--frequency", type=float, default=30.0, help="Command frequency in Hz.")
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
    move_all_joints_to_zero(parse_args())


if __name__ == "__main__":
    main()
