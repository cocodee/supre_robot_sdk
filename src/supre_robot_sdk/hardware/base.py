from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Optional

from supre_robot_sdk.exceptions import ConfigurationError

HARDWARE_PLUGIN_MAP: dict[str, type["HardwareInterface"]] = {}


def register_hardware(hardware_type: str):
    def decorator(cls: type["HardwareInterface"]) -> type["HardwareInterface"]:
        HARDWARE_PLUGIN_MAP[hardware_type] = cls
        return cls

    return decorator


def resolve_hardware_class(hardware_type: str) -> type["HardwareInterface"]:
    try:
        return HARDWARE_PLUGIN_MAP[hardware_type]
    except KeyError as exc:
        raise ConfigurationError(f"Unknown hardware type: {hardware_type}") from exc


class HardwareInterface(ABC):
    @abstractmethod
    def init(self, config: dict[str, Any]) -> bool:
        ...

    @abstractmethod
    def activate(self) -> bool:
        ...

    @abstractmethod
    def deactivate(self) -> None:
        ...

    @abstractmethod
    def read(self) -> list[tuple[Optional[float], Optional[float]]]:
        ...

    @abstractmethod
    def write(self, commands_positions: list[float | None]) -> None:
        ...

    @abstractmethod
    def get_joint_count(self) -> int:
        ...

    def set_enable_torque(self, enable: bool) -> None:
        del enable

