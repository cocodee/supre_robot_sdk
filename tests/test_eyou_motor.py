import datetime
from types import SimpleNamespace

import pytest

import supre_robot_sdk.hardware.eyou.motor as motor_module
from supre_robot_sdk.exceptions import DependencyUnavailableError
from supre_robot_sdk.hardware.eyou import EyouMotorHardware


class FakeFeedback:
    def __init__(self):
        self.last_update_time = datetime.timedelta(milliseconds=1)
        self.position_deg = 12.5
        self.velocity_dps = 1.2
        self.torque_milli = 345.0


class FakeMotorNode:
    def __init__(self, can_device_index, node_id):
        self.can_device_index = can_device_index
        self.node_id = node_id
        self.disabled = False
        self.position = float(node_id)
        self.commands = []

    def get_position(self):
        return self.position

    def get_velocity(self):
        return 0.0

    def get_torque(self):
        return 0.5

    def clear_fault(self):
        return True

    def configure_csp_mode(self, *_args):
        return True

    def start_auto_feedback(self, *_args):
        return True

    def start_error_feedback_tpdo(self, *_args):
        return True

    def get_latest_feedback(self):
        return FakeFeedback()

    def send_csp_target_position(self, position, *_args):
        self.commands.append(position)
        return 0

    def disable(self):
        self.disabled = True


class FakeCanManager:
    def init_device(self, *_args):
        return True


class FakeFeedbackManager:
    @staticmethod
    def get_instance():
        return FakeFeedbackManager()

    def register_callback(self):
        return True


def build_fake_module():
    return SimpleNamespace(
        Baudrate=SimpleNamespace(BPS_1M=1, BPS_500K=2, BPS_250K=3),
        DeviceType=SimpleNamespace(Canable=0),
        CanNetworkManager=FakeCanManager,
        EuMotorNode=FakeMotorNode,
        MotorFeedbackManager=FakeFeedbackManager,
    )


def test_eyou_requires_native_binding(monkeypatch):
    monkeypatch.setattr(motor_module, "eu_motor_py", None)
    with pytest.raises(DependencyUnavailableError):
        EyouMotorHardware().init({"can_device_index": 1, "can_baud_rate": "1M", "joints": []})


def test_eyou_init_activate_read_write(monkeypatch):
    monkeypatch.setattr(motor_module, "eu_motor_py", build_fake_module())
    hardware = EyouMotorHardware()
    assert hardware.init(
        {
            "can_device_index": 1,
            "can_baud_rate": "1M",
            "joints": [
                {"name": "joint_1", "parameters": {"node_id": 21}},
                {"name": "joint_2", "parameters": {"node_id": 22, "start_enabled": False}},
            ],
        }
    )
    assert hardware.activate() is True
    assert hardware.get_joint_count() == 2
    assert hardware.read()[0][0] == 12.5
    hardware.write([1.0, 2.0])
    assert hardware.motor_nodes_[0].commands == [1.0]
    assert hardware.motor_nodes_[1].commands == []
    hardware.set_enable_torque(True)
    assert hardware.hw_start_enabled_ == [True, True]

