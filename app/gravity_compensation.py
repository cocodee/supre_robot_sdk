#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Any

import yaml

from supre_robot_sdk import SupreRobot


@dataclass(slots=True)
class GravityCompensationConfig:
    urdf_path: Path
    frequency: float
    interpolation_period_ms: int
    max_abs_torque_milli: float
    controlled_joints: list[str]
    rated_torque_nm: dict[str, float]


def load_gravity_compensation_config(path: str | Path) -> GravityCompensationConfig:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as file:
        raw = yaml.safe_load(file) or {}
    if not isinstance(raw, dict):
        raise ValueError("Gravity compensation config root must be a mapping.")

    urdf_path = Path(str(raw["urdf_path"]))
    frequency = float(raw.get("frequency", 250.0))
    interpolation_period_ms = int(raw.get("interpolation_period_ms", 4))
    max_abs_torque_milli = float(raw.get("max_abs_torque_milli", 300.0))
    controlled_joints = raw.get("controlled_joints")
    rated_torque_nm = raw.get("rated_torque_nm")

    if frequency <= 0:
        raise ValueError("frequency must be greater than 0.")
    if interpolation_period_ms <= 0:
        raise ValueError("interpolation_period_ms must be greater than 0.")
    if max_abs_torque_milli <= 0:
        raise ValueError("max_abs_torque_milli must be greater than 0.")
    if not isinstance(controlled_joints, list) or not controlled_joints:
        raise ValueError("controlled_joints must be a non-empty list.")
    if len(set(controlled_joints)) != len(controlled_joints):
        raise ValueError("controlled_joints contains duplicate joint names.")
    if not isinstance(rated_torque_nm, dict):
        raise ValueError("rated_torque_nm must be a mapping.")

    clean_rated_torque_nm: dict[str, float] = {}
    for joint_name in controlled_joints:
        if joint_name not in rated_torque_nm:
            raise ValueError(f"rated_torque_nm is missing joint '{joint_name}'.")
        rated = float(rated_torque_nm[joint_name])
        if rated <= 0 or not math.isfinite(rated):
            raise ValueError(f"rated_torque_nm[{joint_name!r}] must be finite and greater than 0.")
        clean_rated_torque_nm[str(joint_name)] = rated

    return GravityCompensationConfig(
        urdf_path=urdf_path,
        frequency=frequency,
        interpolation_period_ms=interpolation_period_ms,
        max_abs_torque_milli=max_abs_torque_milli,
        controlled_joints=[str(joint_name) for joint_name in controlled_joints],
        rated_torque_nm=clean_rated_torque_nm,
    )


def import_pinocchio() -> Any:
    try:
        import pinocchio as pin  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "Pinocchio is required for URDF gravity compensation. Install it in the runtime environment first."
        ) from exc
    return pin


def build_pinocchio_model(pin: Any, urdf_path: str | Path) -> tuple[Any, Any]:
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    return model, data


def build_pinocchio_q(
    pin: Any,
    model: Any,
    positions_deg: dict[str, float],
) -> Any:
    q = pin.neutral(model)
    for joint_name, position_deg in positions_deg.items():
        joint_id = model.getJointId(joint_name)
        if joint_id >= len(model.names):
            continue
        if model.nqs[joint_id] != 1:
            continue
        q[model.idx_qs[joint_id]] = math.radians(float(position_deg))
    return q


def compute_gravity_torques_milli(
    pin: Any,
    model: Any,
    data: Any,
    positions_deg: dict[str, float],
    config: GravityCompensationConfig,
) -> dict[str, float]:
    q = build_pinocchio_q(pin, model, positions_deg)
    gravity_torque_nm = pin.computeGeneralizedGravity(model, data, q)

    commands: dict[str, float] = {}
    for joint_name in config.controlled_joints:
        if joint_name not in positions_deg:
            raise KeyError(f"Robot position feedback is missing controlled joint: {joint_name}")
        joint_id = model.getJointId(joint_name)
        if joint_id >= len(model.names):
            raise ValueError(f"URDF model does not contain controlled joint: {joint_name}")
        if model.nvs[joint_id] != 1:
            raise ValueError(f"Controlled joint '{joint_name}' must have exactly one velocity dimension.")
        torque_nm = float(gravity_torque_nm[model.idx_vs[joint_id]])
        torque_milli = torque_nm / config.rated_torque_nm[joint_name] * 1000.0
        commands[joint_name] = max(
            -config.max_abs_torque_milli,
            min(config.max_abs_torque_milli, torque_milli),
        )
    return commands


def validate_robot_supports_torque_control(robot: SupreRobot, controlled_joints: list[str]) -> None:
    missing = [joint for joint in controlled_joints if joint not in robot.joint_order]
    if missing:
        raise ValueError(f"Controlled joints are missing from robot joint_order: {missing}")
    unsupported = [joint for joint in controlled_joints if not robot.supports_torque_control(joint)]
    if unsupported:
        raise ValueError(f"Controlled joints do not support torque control: {unsupported}")


def run_gravity_compensation(args: argparse.Namespace) -> None:
    config = load_gravity_compensation_config(args.gravity_config)
    pin = import_pinocchio()
    model, data = build_pinocchio_model(pin, config.urdf_path)

    robot = SupreRobot(args.robot_config, control_frequency=config.frequency, use_interpolation=False)
    period = 1.0 / config.frequency
    deadline = None if args.duration <= 0 else time.monotonic() + args.duration

    print(f"Connecting with config: {args.robot_config}")
    robot.connect()
    try:
        validate_robot_supports_torque_control(robot, config.controlled_joints)
        robot.set_enable_torque(True)
        robot.configure_torque_control(
            interpolation_period_ms=config.interpolation_period_ms,
            use_sync=not args.no_sync,
        )
        print(
            f"Running gravity compensation for {len(config.controlled_joints)} joints "
            f"at {config.frequency:.1f} Hz"
        )
        while deadline is None or time.monotonic() < deadline:
            started_at = time.perf_counter()
            positions = robot.get_joint_positions()
            torque_commands = compute_gravity_torques_milli(pin, model, data, positions, config)
            if args.dry_run:
                print(torque_commands)
            else:
                robot.send_joint_torques(torque_commands)
            elapsed = time.perf_counter() - started_at
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        try:
            if robot.is_connected and not args.dry_run:
                robot.send_joint_torques({joint: 0.0 for joint in config.controlled_joints})
        finally:
            robot.disconnect()
            print("Robot disconnected.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run URDF-based gravity compensation through torque control.")
    parser.add_argument(
        "--robot-config",
        default="examples/robot_config.yaml",
        help="Robot YAML config path. Default: examples/robot_config.yaml",
    )
    parser.add_argument(
        "--gravity-config",
        default="app/gravity_compensation_config.yaml",
        help="Gravity compensation YAML config path. Default: app/gravity_compensation_config.yaml",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Run duration in seconds. Use 0 or a negative value to run until interrupted.",
    )
    parser.add_argument("--no-sync", action="store_true", help="Configure CST without SYNC frames.")
    parser.add_argument("--dry-run", action="store_true", help="Compute and print torques without sending commands.")
    return parser.parse_args()


def main() -> None:
    run_gravity_compensation(parse_args())


if __name__ == "__main__":
    main()
