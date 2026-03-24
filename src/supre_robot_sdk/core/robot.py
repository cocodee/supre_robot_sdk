from __future__ import annotations

import time
from pathlib import Path

from supre_robot_sdk.core.hardware_manager import HardwareManager
from supre_robot_sdk.core.interfaces import RobotInterface
from supre_robot_sdk.exceptions import NotConnectedError


class SupreRobot(RobotInterface):
    def __init__(
        self,
        config_path: str | Path,
        *,
        control_frequency: float = 30.0,
        use_interpolation: bool = False,
    ):
        self.config_path = Path(config_path)
        self.control_frequency = float(control_frequency)
        self.use_interpolation = use_interpolation
        self._manager: HardwareManager | None = None
        self._is_connected = False

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def joint_order(self) -> list[str]:
        self._require_manager()
        assert self._manager is not None
        return list(self._manager.joint_order)

    def connect(self) -> None:
        if self._is_connected:
            return
        self._manager = HardwareManager(
            self.config_path,
            control_frequency=self.control_frequency,
            use_interpolation=self.use_interpolation,
        )
        self._manager.init()
        self._manager.activate()
        self._is_connected = True

    def disconnect(self) -> None:
        if self._manager is not None:
            self._manager.deactivate()
        self._manager = None
        self._is_connected = False

    def get_joint_positions(self) -> dict[str, float]:
        manager = self._require_manager()
        positions, _ = manager.read()
        return dict(zip(manager.joint_order, positions))

    def get_joint_forces(self) -> dict[str, float]:
        manager = self._require_manager()
        _, forces = manager.read()
        return dict(zip(manager.joint_order, forces))

    def send_joint_positions(self, positions: dict[str, float]) -> None:
        manager = self._require_manager()
        current_positions, _ = manager.read()
        targets = dict(zip(manager.joint_order, current_positions))
        for joint_name, target in positions.items():
            if joint_name not in targets:
                raise KeyError(f"Unknown joint: {joint_name}")
            targets[joint_name] = float(target)
        manager.write([targets[joint_name] for joint_name in manager.joint_order])

    def set_enable_torque(self, enable: bool) -> None:
        manager = self._require_manager()
        manager.set_enable_torque(enable)

    def move_joint(self, joint_name: str, target: float) -> None:
        self.send_joint_positions({joint_name: target})

    def move_joints(self, targets: dict[str, float]) -> None:
        self.send_joint_positions(targets)

    def open_gripper(self, arm: str) -> None:
        self.move_joint(self._gripper_joint_name(arm), 1.0)

    def close_gripper(self, arm: str) -> None:
        self.move_joint(self._gripper_joint_name(arm), 0.0)

    def execute_trajectory(self, goal_positions: dict[str, float], duration: float = 1.0) -> None:
        manager = self._require_manager()
        if duration <= 0:
            self.send_joint_positions(goal_positions)
            return

        start_positions_map = self.get_joint_positions()
        end_positions = dict(start_positions_map)
        for joint_name, target in goal_positions.items():
            if joint_name not in end_positions:
                raise KeyError(f"Unknown joint: {joint_name}")
            end_positions[joint_name] = float(target)

        control_period = 1.0 / self.control_frequency
        num_steps = max(1, int(duration / control_period))
        if num_steps == 1:
            manager.write([end_positions[name] for name in manager.joint_order])
            time.sleep(duration)
            return

        for step in range(num_steps):
            alpha = (step + 1) / num_steps
            interpolated = []
            for joint_name in manager.joint_order:
                start = start_positions_map[joint_name]
                end = end_positions[joint_name]
                interpolated.append(start + alpha * (end - start))
            step_start = time.perf_counter()
            manager.write(interpolated)
            sleep_time = control_period - (time.perf_counter() - step_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _gripper_joint_name(self, arm: str) -> str:
        arm_name = arm.lower()
        if arm_name not in {"left", "right"}:
            raise ValueError("arm must be 'left' or 'right'")
        return f"{arm_name}_arm_joint_7"

    def _require_manager(self) -> HardwareManager:
        if self._manager is None or not self._is_connected:
            raise NotConnectedError("Robot is not connected.")
        return self._manager

