from __future__ import annotations

import queue
import threading
import time
from typing import Any

from supre_robot_sdk.hardware.base import HardwareInterface


class AsyncInterpolator(HardwareInterface):
    """Wrap a hardware implementation with async linear interpolation."""

    def __init__(self, hardware: HardwareInterface, config: dict[str, Any]):
        self._base_hardware = hardware
        self._config = dict(config)
        interpolation_n = int(self._config.get("interpolation_n", 2))
        self._interpolation_enabled = interpolation_n > 1

        self._command_queue: queue.Queue[list[float] | list[float | None]] | None = None
        self._writer_thread: threading.Thread | None = None
        self._stop_event: threading.Event | None = None
        self._writer_frequency = 0.0
        self._interp_duration = 0.0
        self._interp_start_pos: list[float] = []
        self._interp_end_pos: list[float] = []
        self._interp_start_time = 0.0

        if self._interpolation_enabled:
            self._command_queue = queue.Queue(maxsize=1)
            self._stop_event = threading.Event()
            control_frequency = float(self._config.get("control_frequency", 30.0))
            self._writer_frequency = control_frequency * interpolation_n
            self._interp_duration = 1.0 / control_frequency

    def init(self, config: dict[str, Any]) -> bool:
        return self._base_hardware.init(config)

    def activate(self) -> bool:
        if not self._base_hardware.activate():
            return False
        if not self._interpolation_enabled:
            return True

        initial_state = self._base_hardware.read()
        initial_positions = []
        for idx, item in enumerate(initial_state):
            if not isinstance(item, tuple) or item[0] is None:
                raise RuntimeError(f"Invalid initial state at joint {idx}: {item!r}")
            initial_positions.append(float(item[0]))

        self._interp_start_pos = initial_positions.copy()
        self._interp_end_pos = initial_positions.copy()
        self._interp_start_time = time.monotonic()
        assert self._stop_event is not None
        self._stop_event.clear()
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()
        return True

    def deactivate(self) -> None:
        if self._interpolation_enabled and self._writer_thread and self._writer_thread.is_alive():
            assert self._stop_event is not None
            self._stop_event.set()
            if self._command_queue is not None:
                try:
                    self._command_queue.put_nowait([])
                except queue.Full:
                    pass
            self._writer_thread.join(timeout=1.0)
        self._base_hardware.deactivate()

    def read(self) -> list[tuple[float | None, float | None]]:
        return self._base_hardware.read()

    def write(self, commands_positions: list[float | None]) -> None:
        if not self._interpolation_enabled:
            self._base_hardware.write(commands_positions)
            return
        assert self._command_queue is not None
        clean_positions = [float(x) for x in commands_positions if x is not None]
        if len(clean_positions) != len(commands_positions):
            raise ValueError("AsyncInterpolator requires dense float command vectors.")
        try:
            while not self._command_queue.empty():
                self._command_queue.get_nowait()
            self._command_queue.put_nowait(clean_positions)
        except queue.Full:
            pass

    def get_joint_count(self) -> int:
        return self._base_hardware.get_joint_count()

    def set_enable_torque(self, enable: bool) -> None:
        self._base_hardware.set_enable_torque(enable)

    def _writer_loop(self) -> None:
        assert self._stop_event is not None
        assert self._command_queue is not None
        period = 1.0 / self._writer_frequency
        while not self._stop_event.is_set():
            loop_start = time.perf_counter()
            now = time.monotonic()
            alpha = min(1.0, max(0.0, (now - self._interp_start_time) / self._interp_duration))
            interpolated_positions = [
                start + alpha * (end - start)
                for start, end in zip(self._interp_start_pos, self._interp_end_pos)
            ]
            try:
                new_target = self._command_queue.get_nowait()
                self._interp_start_pos = interpolated_positions.copy()
                self._interp_end_pos = [float(value) for value in new_target]
                self._interp_start_time = time.monotonic()
            except queue.Empty:
                pass

            self._base_hardware.write(interpolated_positions)
            sleep_time = period - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

