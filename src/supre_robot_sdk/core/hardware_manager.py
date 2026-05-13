from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any

from supre_robot_sdk.core.config import SupreRobotConfig, load_robot_config
from supre_robot_sdk.exceptions import ConfigurationError, HardwareActivationError, HardwareInitError
from supre_robot_sdk.hardware import AsyncInterpolator, ensure_builtin_hardware_registered
from supre_robot_sdk.hardware.base import HardwareInterface, resolve_hardware_class


class HardwareManager:
    def __init__(
        self,
        config: str | Path | SupreRobotConfig,
        *,
        control_frequency: float = 30.0,
        use_interpolation: bool = False,
    ):
        ensure_builtin_hardware_registered()
        self.config = load_robot_config(config) if isinstance(config, (str, Path)) else config
        self.control_frequency = float(control_frequency)
        self.use_interpolation = use_interpolation
        self.joint_order = list(self.config.joint_order)
        self.joint_direction = list(self.config.joint_direction) or [1.0] * len(self.joint_order)
        self.calibration = dict(self.config.calibration)
        self.num_joints = len(self.joint_order)
        self._hardware_instances: list[HardwareInterface] = []
        self._joint_map: dict[int, dict[str, Any]] = {}
        self.positions = [0.0] * self.num_joints
        self.forces = [0.0] * self.num_joints
        self.commands = [0.0] * self.num_joints

    def init(self) -> None:
        self._hardware_instances.clear()
        self._joint_map.clear()
        for interface_cfg in self.config.hardware_interfaces:
            hardware_class = resolve_hardware_class(interface_cfg.type)
            instance: HardwareInterface = hardware_class()
            if self.use_interpolation and interface_cfg.interpolation.interpolation_n > 1:
                instance = AsyncInterpolator(
                    instance,
                    {
                        "control_frequency": self.control_frequency,
                        "interpolation_n": interface_cfg.interpolation.interpolation_n,
                    },
                )

            if not instance.init(interface_cfg.config):
                raise HardwareInitError(f"Failed to initialize hardware '{interface_cfg.name}'")

            self._hardware_instances.append(instance)
            for hw_index, joint in enumerate(interface_cfg.joints):
                global_index = self.joint_order.index(joint.name)
                if global_index in self._joint_map:
                    raise ConfigurationError(f"Joint '{joint.name}' has been mapped more than once.")
                self._joint_map[global_index] = {"instance": instance, "hw_index": hw_index}

        if len(self._joint_map) != self.num_joints:
            missing = [joint for index, joint in enumerate(self.joint_order) if index not in self._joint_map]
            raise ConfigurationError(f"Not all joints were mapped: {missing}")

    def activate(self) -> None:
        for instance in self._hardware_instances:
            if not instance.activate():
                raise HardwareActivationError(f"Failed to activate hardware {instance.__class__.__name__}")
        self.read()
        self.commands = list(self.positions)

    def deactivate(self) -> None:
        for instance in self._hardware_instances:
            instance.deactivate()

    def diagnose(self) -> dict[str, Any]:
        interfaces = []
        for index, instance in enumerate(self._hardware_instances):
            interface_cfg = self.config.hardware_interfaces[index]
            try:
                diagnostics = instance.diagnose()
            except Exception as exc:
                diagnostics = {
                    "type": instance.__class__.__name__,
                    "ok": False,
                    "message": f"Failed to collect diagnostics: {exc}",
                    "suggestion": "Check hardware driver state and retry diagnostics.",
                    "joints": [],
                    "events": [],
                }
            diagnostics["name"] = interface_cfg.name
            diagnostics["configured_type"] = interface_cfg.type
            interfaces.append(diagnostics)
        return {
            "ok": all(interface.get("ok", False) for interface in interfaces),
            "joint_order": list(self.joint_order),
            "interfaces": interfaces,
        }

    def read(self) -> tuple[list[float], list[float]]:
        hw_results = {instance: instance.read() for instance in self._hardware_instances}
        for global_index in range(self.num_joints):
            mapping = self._joint_map[global_index]
            result = hw_results[mapping["instance"]][mapping["hw_index"]]
            pos, force = result
            if pos is not None:
                self.positions[global_index] = float(pos) * self.joint_direction[global_index]
            if force is not None:
                self.forces[global_index] = float(force)
        return list(self.positions), list(self.forces)

    def write(self, command_positions: list[float]) -> None:
        if len(command_positions) != self.num_joints:
            raise ValueError(
                f"Command vector length ({len(command_positions)}) does not match number of joints ({self.num_joints})."
            )

        self._validate_command_positions(command_positions)
        self.commands = list(command_positions)
        hw_commands: dict[HardwareInterface, list[float | None]] = {}
        for instance in self._hardware_instances:
            hw_commands[instance] = [None] * instance.get_joint_count()
        for global_index, command_value in enumerate(self.commands):
            mapping = self._joint_map[global_index]
            instance = mapping["instance"]
            hw_index = mapping["hw_index"]
            hw_commands[instance][hw_index] = command_value * self.joint_direction[global_index]
        for instance, commands in hw_commands.items():
            instance.write(commands)

    def set_enable_torque(self, enable: bool) -> None:
        for instance in self._hardware_instances:
            instance.set_enable_torque(enable)

    def _validate_command_positions(self, command_positions: list[float]) -> None:
        for joint_name, command_position in zip(self.joint_order, command_positions):
            calibration = self.calibration.get(joint_name)
            if calibration is None:
                continue
            if not calibration.min_position <= command_position <= calibration.max_position:
                raise ValueError(
                    f"Command for joint '{joint_name}' ({command_position:.3f}) is outside calibration range "
                    f"[{calibration.min_position:.3f}, {calibration.max_position:.3f}]."
                )
