from types import SimpleNamespace

import pytest

import supre_robot_sdk.hardware.jodell.gripper as gripper_module
from supre_robot_sdk.exceptions import DependencyUnavailableError
from supre_robot_sdk.hardware.jodell import JodellGripperHardware


class FakeStatus:
    def __init__(self, position=128, force_current=64):
        self.position = position
        self.force_current = force_current


class FakeClient:
    def __init__(self, _bus, slave_id):
        self.slave_id = slave_id
        self.moves = []

    def enable(self):
        return True

    def disable(self):
        return True

    def get_status(self):
        return FakeStatus()

    def move(self, position, speed, force):
        self.moves.append((position, speed, force))
        return True


class FakeBus:
    def __init__(self, device, baud_rate):
        self.device = device
        self.baud_rate = baud_rate
        self.connected = False

    def connect(self):
        self.connected = True
        return True

    def disconnect(self):
        self.connected = False

    def is_connected(self):
        return self.connected


def build_fake_module():
    return SimpleNamespace(GripperBus=FakeBus, JodellGripper=FakeClient)


def test_jodell_requires_native_binding(monkeypatch):
    monkeypatch.setattr(gripper_module, "jodell_gripper_py", None)
    with pytest.raises(DependencyUnavailableError):
        JodellGripperHardware().init({"device": "/dev/null", "joints": []})


def test_jodell_init_activate_read_write(monkeypatch):
    monkeypatch.setattr(gripper_module, "jodell_gripper_py", build_fake_module())
    hardware = JodellGripperHardware()
    assert hardware.init(
        {
            "device": "/dev/ttyTHS1",
            "baud_rate": 115200,
            "default_speed_percent": 100,
            "default_torque_percent": 100,
            "joints": [{"name": "gripper", "parameters": {"slave_id": 27}}],
        }
    )
    assert hardware.activate() is True
    result = hardware.read()
    assert result[0][0] == pytest.approx(128 / 255.0)
    hardware.write([1.0])
    assert hardware.gripper_clients[0].moves
    hardware.deactivate()
    assert hardware.gripper_bus is None

