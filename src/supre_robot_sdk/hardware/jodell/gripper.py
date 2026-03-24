from __future__ import annotations

import time
from typing import Any

from supre_robot_sdk.exceptions import DependencyUnavailableError
from supre_robot_sdk.hardware.base import HardwareInterface, register_hardware

try:
    import jodell_gripper_py  # type: ignore
except ImportError:  # pragma: no cover - exercised through mocks
    jodell_gripper_py = None


def convert_to_gripper_position(position_float: float) -> int:
    position_float = max(0.0, min(1.0, position_float))
    return int(position_float * 255.0)


def convert_from_gripper_position(position_uint8: int) -> float:
    return float(position_uint8) / 255.0


def convert_from_gripper_force(force_uint8: int) -> float:
    return float(force_uint8) / 255.0


def convert_to_gripper_percentage(percentage: int) -> int:
    percentage = max(0, min(100, percentage))
    return int(percentage * 255 / 100)


@register_hardware("JodellGripperHardware")
class JodellGripperHardware(HardwareInterface):
    def __init__(self):
        self.config: dict[str, Any] | None = None
        self.gripper_bus: Any | None = None
        self.gripper_clients: list[Any] = []
        self.slave_ids: list[int] = []
        self.default_speed_percent = 50
        self.default_force_percent = 50
        self.hw_commands_position: list[float | None] = []
        self.hw_states_position: list[float | None] = []
        self.hw_states_force: list[float | None] = []
        self._cache_duration_seconds = 0.034
        self._last_read_time = 0.0
        self._cached_values: list[tuple[float | None, float | None]] = []

    def init(self, config: dict[str, Any]) -> bool:
        if jodell_gripper_py is None:
            raise DependencyUnavailableError("jodell_gripper_py is required to use JodellGripperHardware")
        try:
            self.config = dict(config)
            self.config["device"]
            self.default_speed_percent = int(self.config.get("default_speed_percent", 50))
            self.default_force_percent = int(self.config.get("default_torque_percent", 50))
            self._cache_duration_seconds = float(self.config.get("cache_duration_seconds", 0.034))
            joints = self.config["joints"]
            if not joints:
                return False
            num_joints = len(joints)
            self.slave_ids = [int(joint["parameters"]["slave_id"]) for joint in joints]
            self.hw_commands_position = [None] * num_joints
            self.hw_states_position = [0.0] * num_joints
            self.hw_states_force = [0.0] * num_joints
            self._cached_values = [(None, None)] * num_joints
            return True
        except Exception:
            return False

    def activate(self) -> bool:
        if not self.config:
            return False
        try:
            self.gripper_bus = jodell_gripper_py.GripperBus(
                self.config["device"], self.config.get("baud_rate", 115200)
            )
            if not self.gripper_bus.connect():
                self.gripper_bus = None
                return False
            self.gripper_clients.clear()
            for slave_id in self.slave_ids:
                client = jodell_gripper_py.JodellGripper(self.gripper_bus, slave_id)
                if not client.enable():
                    self.deactivate()
                    return False
                self.gripper_clients.append(client)
            return True
        except Exception:
            if self.gripper_bus is not None:
                self.deactivate()
            return False

    def deactivate(self) -> None:
        for client in self.gripper_clients:
            try:
                client.disable()
            except Exception:
                pass
        self.gripper_clients.clear()
        if self.gripper_bus is not None:
            try:
                self.gripper_bus.disconnect()
            finally:
                self.gripper_bus = None

    def read(self) -> list[tuple[float | None, float | None]]:
        now = time.monotonic()
        if (now - self._last_read_time) < self._cache_duration_seconds:
            return list(self._cached_values)
        if not self.gripper_clients:
            return [(None, None)] * len(self.slave_ids)

        for index, client in enumerate(self.gripper_clients):
            try:
                status = client.get_status()
                self.hw_states_position[index] = convert_from_gripper_position(status.position)
                self.hw_states_force[index] = convert_from_gripper_force(status.force_current)
            except Exception:
                self.hw_states_position[index] = None
                self.hw_states_force[index] = None

        self._cached_values = list(zip(self.hw_states_position, self.hw_states_force))
        self._last_read_time = now
        return list(self._cached_values)

    def write(self, commands: list[float | None]) -> None:
        if len(commands) != len(self.gripper_clients):
            raise ValueError("Command length does not match number of grippers.")

        self.hw_commands_position = list(commands)
        speed_8bit = convert_to_gripper_percentage(self.default_speed_percent)
        force_8bit = convert_to_gripper_percentage(self.default_force_percent)
        for index, command in enumerate(self.hw_commands_position):
            if command is None:
                continue
            position_8bit = convert_to_gripper_position(float(command))
            client = self.gripper_clients[index]
            if not client.move(position_8bit, speed_8bit, force_8bit):
                raise RuntimeError(f"Failed to send move command to gripper {self.slave_ids[index]}")
        self.hw_commands_position = [None] * len(self.gripper_clients)

    def get_joint_count(self) -> int:
        return len(self.slave_ids)

