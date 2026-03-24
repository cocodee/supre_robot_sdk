from __future__ import annotations

import datetime
import time
from typing import Any

from supre_robot_sdk.exceptions import DependencyUnavailableError
from supre_robot_sdk.hardware.base import HardwareInterface, register_hardware

try:
    import eu_motor_py  # type: ignore
except ImportError:  # pragma: no cover - exercised through mocks
    eu_motor_py = None


@register_hardware("EyouMotorHardware")
class EyouMotorHardware(HardwareInterface):
    def __init__(self):
        self.can_manager_: Any | None = None
        self.feedback_manager_: Any | None = None
        self.motor_nodes_: list[Any] = []
        self.joint_names_: list[str] = []
        self.hw_states_positions_: list[float] = []
        self.hw_states_velocities_: list[float] = []
        self.hw_states_torques_: list[float] = []
        self.hw_commands_positions_: list[float] = []
        self.hw_start_enabled_: list[bool] = []
        self._config: dict[str, Any] = {}
        self._last_log_time = time.monotonic()
        self._max_write_duration_us = 0.0

    def init(self, config: dict[str, Any]) -> bool:
        if eu_motor_py is None:
            raise DependencyUnavailableError("eu_motor_py is required to use EyouMotorHardware")

        self._config = dict(config)
        try:
            can_device_index = int(self._config["can_device_index"])
            baud_rate_str = self._config["can_baud_rate"]
            baud_rate_map = {
                "1M": eu_motor_py.Baudrate.BPS_1M,
                "500K": eu_motor_py.Baudrate.BPS_500K,
                "250K": eu_motor_py.Baudrate.BPS_250K,
            }
            can_baud_rate = baud_rate_map[baud_rate_str]
            joints = self._config["joints"]
            if not joints:
                raise KeyError("config must contain a non-empty 'joints' list")

            num_joints = len(joints)
            self.hw_states_positions_ = [0.0] * num_joints
            self.hw_states_velocities_ = [0.0] * num_joints
            self.hw_states_torques_ = [0.0] * num_joints
            self.hw_commands_positions_ = [0.0] * num_joints
            self.hw_start_enabled_ = [True] * num_joints

            self.can_manager_ = eu_motor_py.CanNetworkManager()
            self.can_manager_.init_device(eu_motor_py.DeviceType.Canable, can_device_index, can_baud_rate)

            self.motor_nodes_.clear()
            self.joint_names_.clear()
            for index, joint_info in enumerate(joints):
                joint_name = joint_info["name"]
                parameters = joint_info["parameters"]
                node_id = int(parameters["node_id"])
                start_enabled = parameters.get("start_enabled", True)
                self.joint_names_.append(joint_name)
                self.motor_nodes_.append(eu_motor_py.EuMotorNode(can_device_index, node_id))
                self.hw_start_enabled_[index] = str(start_enabled).lower() != "false"
            return True
        except Exception:
            return False

    def activate(self) -> bool:
        try:
            for index, motor in enumerate(self.motor_nodes_):
                pos = float(motor.get_position())
                vel = float(motor.get_velocity())
                torque = float(motor.get_torque())
                self.hw_states_positions_[index] = pos
                self.hw_states_velocities_[index] = vel
                self.hw_states_torques_[index] = torque
                self.hw_commands_positions_[index] = pos

            for index, motor in enumerate(self.motor_nodes_):
                if self.hw_start_enabled_[index]:
                    if not all(
                        [
                            motor.clear_fault(),
                            motor.configure_csp_mode(0, False),
                            motor.start_auto_feedback(0, 255, 20),
                            motor.start_error_feedback_tpdo(1, 255, 60),
                        ]
                    ):
                        return False
                else:
                    motor.disable()
                    motor.clear_fault()
                    motor.start_auto_feedback(0, 255, 20)
                    motor.start_error_feedback_tpdo(1, 255, 60)

            self.feedback_manager_ = eu_motor_py.MotorFeedbackManager.get_instance()
            self.feedback_manager_.register_callback()
            return True
        except Exception:
            return False

    def read(self) -> list[tuple[float | None, float | None]]:
        for index, motor in enumerate(self.motor_nodes_):
            feedback = motor.get_latest_feedback()
            if feedback.last_update_time > datetime.timedelta(0):
                self.hw_states_positions_[index] = float(feedback.position_deg)
                self.hw_states_velocities_[index] = float(feedback.velocity_dps)
                self.hw_states_torques_[index] = float(feedback.torque_milli) / 1000.0
        return list(zip(self.hw_states_positions_, self.hw_states_torques_))

    def write(self, commands_positions: list[float | None]) -> None:
        if len(commands_positions) != len(self.motor_nodes_):
            raise ValueError("Command length does not match number of motors.")
        if any(value is None for value in commands_positions):
            raise ValueError("EyouMotorHardware requires dense float command vectors.")

        start_time = time.perf_counter()
        self.hw_commands_positions_ = [float(value) for value in commands_positions]  # type: ignore[arg-type]
        for index, motor in enumerate(self.motor_nodes_):
            if self.hw_start_enabled_[index]:
                result = motor.send_csp_target_position(self.hw_commands_positions_[index], 0, False)
                if result != 0:
                    raise RuntimeError(f"Failed to send command to joint {self.joint_names_[index]}")

        current_duration_us = (time.perf_counter() - start_time) * 1_000_000
        if current_duration_us > self._max_write_duration_us:
            self._max_write_duration_us = current_duration_us
        now = time.monotonic()
        if (now - self._last_log_time) >= 1.0:
            self._max_write_duration_us = 0.0
            self._last_log_time = now

    def deactivate(self) -> None:
        for motor in self.motor_nodes_:
            try:
                motor.disable()
            except Exception:
                pass

    def get_joint_count(self) -> int:
        return len(self.motor_nodes_)

    def set_enable_torque(self, enable: bool) -> None:
        for index, motor in enumerate(self.motor_nodes_):
            if enable:
                current_real_pos = float(motor.get_position())
                self.hw_commands_positions_[index] = current_real_pos
                self.hw_states_positions_[index] = current_real_pos
                motor.clear_fault()
                motor.configure_csp_mode(0, False)
                self.hw_start_enabled_[index] = True
            else:
                motor.disable()
                self.hw_start_enabled_[index] = False

