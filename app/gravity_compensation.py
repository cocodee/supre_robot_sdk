#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
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
    max_torque_step_milli: float
    ramp_duration_s: float
    max_position_step_deg: float
    max_velocity_deg_s: float
    controlled_joints: list[str]
    rated_torque_nm: dict[str, float]
    torque_scale: dict[str, float]
    position_limits_deg: dict[str, tuple[float, float]] = field(default_factory=dict)
    initial_positions_deg: dict[str, float] = field(default_factory=dict)
    initial_move_duration_s: float = 10.0
    initial_move_frequency: float = 30.0
    initial_move_max_step_deg: float = 2.0
    max_initial_deviation_deg: dict[str, float] = field(default_factory=dict)


def load_gravity_compensation_config(path: str | Path) -> GravityCompensationConfig:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as file:
        raw = yaml.safe_load(file) or {}
    if not isinstance(raw, dict):
        raise ValueError("Gravity compensation config root must be a mapping.")

    if "urdf_path" not in raw:
        raise ValueError("urdf_path is required.")
    urdf_path = Path(str(raw["urdf_path"]))
    frequency = float(raw.get("frequency", 250.0))
    interpolation_period_ms = int(raw.get("interpolation_period_ms", 4))
    max_abs_torque_milli = float(raw.get("max_abs_torque_milli", 300.0))
    max_torque_step_milli = float(raw.get("max_torque_step_milli", 10.0))
    ramp_duration_s = float(raw.get("ramp_duration_s", 5.0))
    max_position_step_deg = float(raw.get("max_position_step_deg", 5.0))
    max_velocity_deg_s = float(raw.get("max_velocity_deg_s", 90.0))
    initial_move_duration_s = float(raw.get("initial_move_duration_s", 10.0))
    initial_move_frequency = float(raw.get("initial_move_frequency", 30.0))
    initial_move_max_step_deg = float(raw.get("initial_move_max_step_deg", 2.0))
    controlled_joints = raw.get("controlled_joints")
    rated_torque_nm = raw.get("rated_torque_nm")

    if frequency <= 0 or not math.isfinite(frequency):
        raise ValueError("frequency must be finite and greater than 0.")
    if interpolation_period_ms <= 0:
        raise ValueError("interpolation_period_ms must be greater than 0.")
    if max_abs_torque_milli <= 0 or not math.isfinite(max_abs_torque_milli):
        raise ValueError("max_abs_torque_milli must be finite and greater than 0.")
    if max_torque_step_milli <= 0 or not math.isfinite(max_torque_step_milli):
        raise ValueError("max_torque_step_milli must be finite and greater than 0.")
    if ramp_duration_s < 0 or not math.isfinite(ramp_duration_s):
        raise ValueError("ramp_duration_s must be finite and greater than or equal to 0.")
    if max_position_step_deg <= 0 or not math.isfinite(max_position_step_deg):
        raise ValueError("max_position_step_deg must be finite and greater than 0.")
    if max_velocity_deg_s <= 0 or not math.isfinite(max_velocity_deg_s):
        raise ValueError("max_velocity_deg_s must be finite and greater than 0.")
    if initial_move_duration_s <= 0 or not math.isfinite(initial_move_duration_s):
        raise ValueError("initial_move_duration_s must be finite and greater than 0.")
    if initial_move_frequency <= 0 or not math.isfinite(initial_move_frequency):
        raise ValueError("initial_move_frequency must be finite and greater than 0.")
    if initial_move_max_step_deg <= 0 or not math.isfinite(initial_move_max_step_deg):
        raise ValueError("initial_move_max_step_deg must be finite and greater than 0.")
    if not isinstance(controlled_joints, list) or not controlled_joints:
        raise ValueError("controlled_joints must be a non-empty list.")
    clean_controlled_joints = [str(joint_name) for joint_name in controlled_joints]
    if len(set(clean_controlled_joints)) != len(clean_controlled_joints):
        raise ValueError("controlled_joints contains duplicate joint names.")
    if not isinstance(rated_torque_nm, dict):
        raise ValueError("rated_torque_nm must be a mapping.")

    clean_rated_torque_nm: dict[str, float] = {}
    clean_torque_scale = _parse_joint_float_map(
        raw.get("torque_scale"),
        clean_controlled_joints,
        default=1.0,
        field_name="torque_scale",
        min_value=0.0,
    )
    clean_position_limits_deg = _parse_position_limits_deg(raw.get("position_limits_deg"), clean_controlled_joints)
    clean_initial_positions_deg = _parse_required_joint_float_map(
        raw.get("initial_positions_deg"),
        clean_controlled_joints,
        field_name="initial_positions_deg",
    )
    clean_max_initial_deviation_deg = _parse_joint_float_or_map(
        raw.get("max_initial_deviation_deg", 10.0),
        clean_controlled_joints,
        default=10.0,
        field_name="max_initial_deviation_deg",
        min_value=0.0,
        allow_zero=False,
    )
    for joint_name in clean_controlled_joints:
        if joint_name not in rated_torque_nm:
            raise ValueError(f"rated_torque_nm is missing joint '{joint_name}'.")
        rated = float(rated_torque_nm[joint_name])
        if rated <= 0 or not math.isfinite(rated):
            raise ValueError(f"rated_torque_nm[{joint_name!r}] must be finite and greater than 0.")
        clean_rated_torque_nm[joint_name] = rated

    return GravityCompensationConfig(
        urdf_path=urdf_path,
        frequency=frequency,
        interpolation_period_ms=interpolation_period_ms,
        max_abs_torque_milli=max_abs_torque_milli,
        max_torque_step_milli=max_torque_step_milli,
        ramp_duration_s=ramp_duration_s,
        max_position_step_deg=max_position_step_deg,
        max_velocity_deg_s=max_velocity_deg_s,
        controlled_joints=clean_controlled_joints,
        rated_torque_nm=clean_rated_torque_nm,
        torque_scale=clean_torque_scale,
        position_limits_deg=clean_position_limits_deg,
        initial_positions_deg=clean_initial_positions_deg,
        initial_move_duration_s=initial_move_duration_s,
        initial_move_frequency=initial_move_frequency,
        initial_move_max_step_deg=initial_move_max_step_deg,
        max_initial_deviation_deg=clean_max_initial_deviation_deg,
    )


def _parse_joint_float_map(
    raw_map: Any,
    controlled_joints: list[str],
    *,
    default: float,
    field_name: str,
    min_value: float,
) -> dict[str, float]:
    if raw_map is None:
        return {joint_name: default for joint_name in controlled_joints}
    if not isinstance(raw_map, dict):
        raise ValueError(f"{field_name} must be a mapping.")

    clean: dict[str, float] = {}
    for joint_name in controlled_joints:
        value = float(raw_map.get(joint_name, default))
        if not math.isfinite(value) or value < min_value:
            raise ValueError(f"{field_name}[{joint_name!r}] must be finite and at least {min_value}.")
        clean[joint_name] = value
    unknown = sorted(str(joint_name) for joint_name in raw_map if str(joint_name) not in controlled_joints)
    if unknown:
        raise ValueError(f"{field_name} contains joints that are not controlled: {unknown}")
    return clean


def _parse_required_joint_float_map(
    raw_map: Any,
    controlled_joints: list[str],
    *,
    field_name: str,
) -> dict[str, float]:
    if not isinstance(raw_map, dict):
        raise ValueError(f"{field_name} must be a mapping.")

    clean: dict[str, float] = {}
    for joint_name in controlled_joints:
        if joint_name not in raw_map:
            raise ValueError(f"{field_name} is missing joint '{joint_name}'.")
        value = float(raw_map[joint_name])
        if not math.isfinite(value):
            raise ValueError(f"{field_name}[{joint_name!r}] must be finite.")
        clean[joint_name] = value
    unknown = sorted(str(joint_name) for joint_name in raw_map if str(joint_name) not in controlled_joints)
    if unknown:
        raise ValueError(f"{field_name} contains joints that are not controlled: {unknown}")
    return clean


def _parse_joint_float_or_map(
    raw_value: Any,
    controlled_joints: list[str],
    *,
    default: float,
    field_name: str,
    min_value: float,
    allow_zero: bool,
) -> dict[str, float]:
    if raw_value is None:
        raw_value = default
    if isinstance(raw_value, dict):
        clean = _parse_joint_float_map(
            raw_value,
            controlled_joints,
            default=default,
            field_name=field_name,
            min_value=min_value,
        )
    else:
        value = float(raw_value)
        clean = {joint_name: value for joint_name in controlled_joints}

    for joint_name, value in clean.items():
        if not math.isfinite(value):
            raise ValueError(f"{field_name}[{joint_name!r}] must be finite.")
        if allow_zero:
            valid = value >= min_value
        else:
            valid = value > min_value
        if not valid:
            relation = "at least" if allow_zero else "greater than"
            raise ValueError(f"{field_name}[{joint_name!r}] must be {relation} {min_value}.")
    return clean


def _parse_position_limits_deg(
    raw_limits: Any,
    controlled_joints: list[str],
) -> dict[str, tuple[float, float]]:
    if raw_limits is None:
        return {}
    if not isinstance(raw_limits, dict):
        raise ValueError("position_limits_deg must be a mapping.")

    clean: dict[str, tuple[float, float]] = {}
    for joint_name, raw_limit in raw_limits.items():
        clean_joint_name = str(joint_name)
        if clean_joint_name not in controlled_joints:
            raise ValueError(f"position_limits_deg contains joint that is not controlled: {clean_joint_name}")
        if not isinstance(raw_limit, dict):
            raise ValueError(f"position_limits_deg[{clean_joint_name!r}] must be a mapping.")
        try:
            min_position = float(raw_limit["min"])
            max_position = float(raw_limit["max"])
        except KeyError as exc:
            raise ValueError(
                f"position_limits_deg[{clean_joint_name!r}] must contain 'min' and 'max'."
            ) from exc
        if not math.isfinite(min_position) or not math.isfinite(max_position):
            raise ValueError(f"position_limits_deg[{clean_joint_name!r}] must use finite values.")
        if min_position > max_position:
            raise ValueError(f"position_limits_deg[{clean_joint_name!r}] has min greater than max.")
        clean[clean_joint_name] = (min_position, max_position)
    return clean


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
        torque_milli *= config.torque_scale[joint_name]
        commands[joint_name] = max(
            -config.max_abs_torque_milli,
            min(config.max_abs_torque_milli, torque_milli),
        )
    return commands


def apply_startup_ramp(
    commands: dict[str, float],
    *,
    elapsed_s: float,
    ramp_duration_s: float,
) -> dict[str, float]:
    if ramp_duration_s <= 0:
        return dict(commands)
    scale = max(0.0, min(1.0, elapsed_s / ramp_duration_s))
    return {joint_name: torque * scale for joint_name, torque in commands.items()}


def limit_torque_step(
    commands: dict[str, float],
    previous_commands: dict[str, float],
    max_torque_step_milli: float,
) -> dict[str, float]:
    limited: dict[str, float] = {}
    for joint_name, target in commands.items():
        previous = previous_commands.get(joint_name, 0.0)
        delta = target - previous
        if delta > max_torque_step_milli:
            limited[joint_name] = previous + max_torque_step_milli
        elif delta < -max_torque_step_milli:
            limited[joint_name] = previous - max_torque_step_milli
        else:
            limited[joint_name] = target
    return limited


def build_initial_position_targets(
    start_positions: dict[str, float],
    config: GravityCompensationConfig,
) -> dict[str, float]:
    targets = dict(start_positions)
    for joint_name in config.controlled_joints:
        if joint_name not in start_positions:
            raise KeyError(f"Robot position feedback is missing controlled joint: {joint_name}")
        targets[joint_name] = config.initial_positions_deg[joint_name]
    return targets


def build_joint_trajectory(
    start_positions: dict[str, float],
    target_positions: dict[str, float],
    *,
    duration: float,
    frequency: float,
    max_step: float,
) -> list[dict[str, float]]:
    if duration <= 0:
        raise ValueError("duration must be greater than 0.")
    if frequency <= 0:
        raise ValueError("frequency must be greater than 0.")
    if max_step <= 0:
        raise ValueError("max_step must be greater than 0.")
    if set(start_positions) != set(target_positions):
        raise ValueError("start and target positions must contain the same joints.")

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


def execute_position_trajectory(
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


def validate_initial_pose_window(
    positions: dict[str, float],
    config: GravityCompensationConfig,
) -> None:
    for joint_name in config.controlled_joints:
        if joint_name not in positions:
            raise KeyError(f"Robot position feedback is missing controlled joint: {joint_name}")
        position = float(positions[joint_name])
        target = config.initial_positions_deg[joint_name]
        max_deviation = config.max_initial_deviation_deg[joint_name]
        deviation = abs(position - target)
        if deviation > max_deviation:
            raise RuntimeError(
                f"Initial pose window tripped for joint '{joint_name}': "
                f"deviation {deviation:.3f} deg exceeds {max_deviation:.3f} deg "
                f"from initial position {target:.3f} deg."
            )


def validate_position_watchdog(
    positions: dict[str, float],
    previous_positions: dict[str, float] | None,
    *,
    elapsed_s: float,
    config: GravityCompensationConfig,
) -> None:
    for joint_name in config.controlled_joints:
        if joint_name not in positions:
            raise KeyError(f"Robot position feedback is missing controlled joint: {joint_name}")
        limit = config.position_limits_deg.get(joint_name)
        if limit is not None:
            position = float(positions[joint_name])
            min_position, max_position = limit
            if not min_position <= position <= max_position:
                raise RuntimeError(
                    f"Position limit tripped for joint '{joint_name}': "
                    f"{position:.3f} deg is outside [{min_position:.3f}, {max_position:.3f}] deg."
                )
    validate_initial_pose_window(positions, config)
    if previous_positions is None:
        return
    if elapsed_s <= 0:
        return

    for joint_name in config.controlled_joints:
        delta = abs(float(positions[joint_name]) - float(previous_positions[joint_name]))
        if delta > config.max_position_step_deg:
            raise RuntimeError(
                f"Position watchdog tripped for joint '{joint_name}': "
                f"step {delta:.3f} deg exceeds {config.max_position_step_deg:.3f} deg."
            )
        velocity = delta / elapsed_s
        if velocity > config.max_velocity_deg_s:
            raise RuntimeError(
                f"Velocity watchdog tripped for joint '{joint_name}': "
                f"{velocity:.3f} deg/s exceeds {config.max_velocity_deg_s:.3f} deg/s."
            )


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
    torque_control_configured = False

    print(f"Connecting with config: {args.robot_config}")
    robot.connect()
    try:
        validate_robot_supports_torque_control(robot, config.controlled_joints)
        start_positions = robot.get_joint_positions()
        initial_targets = build_initial_position_targets(start_positions, config)
        initial_trajectory = build_joint_trajectory(
            start_positions,
            initial_targets,
            duration=config.initial_move_duration_s,
            frequency=config.initial_move_frequency,
            max_step=config.initial_move_max_step_deg,
        )
        largest_initial_move = max(
            abs(initial_targets[joint] - start_positions[joint])
            for joint in config.controlled_joints
        )

        print(
            f"Initial pose move: max controlled-joint delta {largest_initial_move:.3f} deg, "
            f"{len(initial_trajectory)} steps over {len(initial_trajectory) / config.initial_move_frequency:.2f}s"
        )
        if not args.dry_run:
            robot.set_enable_torque(True)
            execute_position_trajectory(
                robot,
                initial_trajectory,
                frequency=config.initial_move_frequency,
            )
            robot.send_joint_positions(initial_targets)
            reached_positions = robot.get_joint_positions()
            validate_initial_pose_window(reached_positions, config)
            robot.configure_torque_control(
                interpolation_period_ms=config.interpolation_period_ms,
                use_sync=not args.no_sync,
            )
            torque_control_configured = True
        print(
            f"Running gravity compensation for {len(config.controlled_joints)} joints "
            f"at {config.frequency:.1f} Hz"
        )
        started_running_at = time.monotonic()
        deadline = None if args.duration <= 0 else started_running_at + args.duration
        previous_positions: dict[str, float] | None = None
        previous_commands = {joint: 0.0 for joint in config.controlled_joints}
        previous_loop_at = time.monotonic()
        while deadline is None or time.monotonic() < deadline:
            started_at = time.perf_counter()
            loop_at = time.monotonic()
            positions = robot.get_joint_positions()
            if not args.dry_run:
                validate_position_watchdog(
                    positions,
                    previous_positions,
                    elapsed_s=loop_at - previous_loop_at,
                    config=config,
                )
            torque_commands = compute_gravity_torques_milli(pin, model, data, positions, config)
            torque_commands = apply_startup_ramp(
                torque_commands,
                elapsed_s=loop_at - started_running_at,
                ramp_duration_s=config.ramp_duration_s,
            )
            torque_commands = limit_torque_step(
                torque_commands,
                previous_commands,
                config.max_torque_step_milli,
            )
            if args.dry_run:
                print(torque_commands)
            else:
                robot.send_joint_torques(torque_commands)
            previous_positions = {joint: float(positions[joint]) for joint in config.controlled_joints}
            previous_commands = torque_commands
            previous_loop_at = loop_at
            elapsed = time.perf_counter() - started_at
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        try:
            if robot.is_connected and torque_control_configured:
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
