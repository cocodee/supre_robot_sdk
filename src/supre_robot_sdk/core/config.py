from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from supre_robot_sdk.exceptions import ConfigurationError
from supre_robot_sdk.hardware import ensure_builtin_hardware_registered
from supre_robot_sdk.hardware.base import HARDWARE_PLUGIN_MAP


@dataclass(slots=True)
class JointBinding:
    name: str
    parameters: dict[str, Any]


@dataclass(slots=True)
class InterpolationConfig:
    interpolation_n: int = 1


@dataclass(slots=True)
class HardwareInterfaceConfig:
    name: str
    type: str
    config: dict[str, Any]
    joints: list[JointBinding]
    interpolation: InterpolationConfig = field(default_factory=InterpolationConfig)


@dataclass(slots=True)
class SupreRobotConfig:
    joint_order: list[str]
    hardware_interfaces: list[HardwareInterfaceConfig]
    path: Path | None = None


def load_robot_config(path: str | Path) -> SupreRobotConfig:
    ensure_builtin_hardware_registered()
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as file:
        raw = yaml.safe_load(file)

    if not isinstance(raw, dict):
        raise ConfigurationError("Config root must be a mapping.")

    joint_order = raw.get("joint_order")
    if not isinstance(joint_order, list) or not joint_order:
        raise ConfigurationError("'joint_order' must be a non-empty list.")
    if len(set(joint_order)) != len(joint_order):
        raise ConfigurationError("'joint_order' contains duplicate names.")

    raw_interfaces = raw.get("hardware_interfaces")
    if not isinstance(raw_interfaces, list) or not raw_interfaces:
        raise ConfigurationError("'hardware_interfaces' must be a non-empty list.")

    hardware_interfaces: list[HardwareInterfaceConfig] = []
    seen_assignments: dict[str, str] = {}
    for raw_interface in raw_interfaces:
        if not isinstance(raw_interface, dict):
            raise ConfigurationError("Each hardware interface must be a mapping.")
        try:
            name = str(raw_interface["name"])
            hardware_type = str(raw_interface["type"])
            if hardware_type not in HARDWARE_PLUGIN_MAP:
                raise ConfigurationError(f"Unknown hardware type: {hardware_type}")
            config = dict(raw_interface["config"])
            raw_joints = config.get("joints")
            if not isinstance(raw_joints, list) or not raw_joints:
                raise ConfigurationError(f"Hardware interface '{name}' must define non-empty joints.")
            joints = []
            for raw_joint in raw_joints:
                joint_name = str(raw_joint["name"])
                parameters = dict(raw_joint.get("parameters", {}))
                if joint_name in seen_assignments:
                    raise ConfigurationError(
                        f"Joint '{joint_name}' is assigned by both '{seen_assignments[joint_name]}' and '{name}'."
                    )
                seen_assignments[joint_name] = name
                joints.append(JointBinding(name=joint_name, parameters=parameters))
            interpolation = InterpolationConfig(
                interpolation_n=int(raw_interface.get("interpolation", {}).get("interpolation_n", 1))
            )
        except KeyError as exc:
            raise ConfigurationError(f"Missing required key {exc!s} in hardware interface config.") from exc

        hardware_interfaces.append(
            HardwareInterfaceConfig(
                name=name,
                type=hardware_type,
                config=config,
                joints=joints,
                interpolation=interpolation,
            )
        )

    missing_joints = [joint for joint in joint_order if joint not in seen_assignments]
    extra_joints = [joint for joint in seen_assignments if joint not in joint_order]
    if missing_joints:
        raise ConfigurationError(f"Unmapped joints in joint_order: {missing_joints}")
    if extra_joints:
        raise ConfigurationError(f"Joints mapped by hardware but missing from joint_order: {extra_joints}")

    return SupreRobotConfig(joint_order=list(joint_order), hardware_interfaces=hardware_interfaces, path=config_path)

